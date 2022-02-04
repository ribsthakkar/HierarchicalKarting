using KartGame.AI;
using KartGame.KartSystems;
using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Threading;
using Unity.MLAgents;
using UnityEngine;



public enum Event
{
    HitWall = 0,
    HitOpponent = 1,
    ReachNonGoalSection = 2,
    ReachGoalSection = 3,
    DroveReverseLimit = 4,
    FellOffWorld = 5
}

public enum EnvironmentMode
{
    Race = 0,
    Training =1,
    Experiment = 2
}

[System.Serializable]
public class RacingTeam
{
    public List<KartAgent> Racers;
}

public class RacingEnvController : MonoBehaviour
{
    [Tooltip("What are the teams in the game?")]
    public RacingTeam[] Teams;

    [Tooltip("Who are the Racers in the game?")]
    public KartAgent[] Agents;

    [Header("Checkpoints"), Tooltip("What are the series of checkpoints for the agent to seek and pass through?")]
    public DiscretePositionTracker[] Sections;
    public float[] initialTireWears;



    [Header("Environment Mode"), Tooltip("Mode of the environment")]
    public EnvironmentMode mode = EnvironmentMode.Training;

    [Header("Experiment Params"), Tooltip("Parameters if running in experiment mode")]
    public string ExperimentName;
    public int TotalExperiments;
    int experimentNum = 0;

    #region Rewards
    [Header("Rewards"), Tooltip("What penatly is given when the agent crashes with a wall?")]
    public float WallHitPenalty = -0.05f;
    [Tooltip("What penatly is given when the agent crashes into another agent?")]
    public float OpponentHitPenalty = -2f;
    [Tooltip("What penatly is given when the agent is crashed by another agent?")]
    public float HitByOpponentPenalty = -2f;
    [Tooltip("How much reward is given when the agent successfully passes the checkpoint with desired state?")]
    public float PassCheckpointStateReward = 0.1f;
    [Tooltip("How much reward is given when the agent successfully passes the checkpoints?")]
    public float PassCheckpointBase = 5f;
    [Tooltip("How much reward is given when the agent successfully passes the checkpoints in faster time")]
    public float PassCheckpointTimeMultiplier = 10f;
    [Tooltip("How much penalty is given for being behind first agent?")]
    public float BeingBehindOpponentCheckpointPenalty = -0.06f;
    [Tooltip("How much penalty is given for being behind first agent?")]
    public float BeingBehindTeammateCheckpointPenalty = -0.02f;
    [Tooltip("How much emphasis on performance of teammates?")]
    public float TeamScoreRewardMultiplier = 0.75f;
    [Tooltip("How much reward is given when the agent travels backwards through the checkpoints?")]
    public float ReversePenalty = -0.5f;
    [Tooltip("How much reward is given when the agent switches lanes multiple times on a straightaway?")]
    public float SwervingPenalty = -0.5f;
    [Tooltip("How much the normalized remaining time is multiplied by to provide as a reward for the agent reaching the goal checkpoint?")]
    public float ReachGoalCheckpointRewardMultplier = 5.0f;
    [Tooltip("How much reward base reward is given to the agent for reaching the goal checkpoint?")]
    public float ReachGoalCheckpointRewardBase = 3.0f;
    [Tooltip("Should typically be a small value, but we reward the agent for moving in the right direction.")]
    public float TowardsCheckpointReward = 0.008f;
    [Tooltip("Typically if the agent moves faster, we want to reward it for finishing the track quickly.")]
    public float SpeedReward = 0.07f;
    [Tooltip("If the agent is moving too slowly, we want to penalize it.")]
    public float SlowMovingPenalty = -3.0f;
    [Tooltip("Reward the agent when it keeps accelerating")]
    public float AccelerationReward = 0.002f;
    [Tooltip("Reward the agent for being at the goal accelerating")]
    public float AtGoalReward = 0.5f;
    #endregion

    #region Rules
    [Header("Rules"), Tooltip("How many lane changes is allowed for an agent on a straight?")]
    public int MaxLaneChanges = 4;
    [Tooltip("How many laps is considered reaching the goal")]
    public int laps = 2;
    #endregion

    public Dictionary<Rigidbody, KartAgent> AgentBodies = new Dictionary<Rigidbody, KartAgent>();
    public HashSet<KartAgent> inactiveAgents = new HashSet<KartAgent>();
    [HideInInspector] public int goalSection;
    [HideInInspector] public Dictionary<int, Dictionary<int, float>> minSectionTimes = new Dictionary<int, Dictionary<int, float>>();
    [HideInInspector] public Dictionary<int, Dictionary<int, int>> agentsPastSection = new Dictionary<int, Dictionary<int, int>>();
    List<SimpleMultiAgentGroup> m_AgentGroups;

    [Header("Training Params")]
    public int episodeSteps;
    public int maxEpisodeSteps;
    public bool disableOnEnd;
    [Tooltip("How many sections ahead should the agents be looking ahead/driving to")]
    public int sectionHorizon;

    bool initialStarted = false;
    bool coroStarted = false;

    // Start is called before the first frame update
    void Start()
    {
        m_AgentGroups = new List<SimpleMultiAgentGroup>();
        for (int i = 0; i < Agents.Length; i++)
        {
            inactiveAgents.Add(Agents[i]);
            AgentBodies[Agents[i].GetComponent<Rigidbody>()] = Agents[i];
        }
        for (int i = 0; i < Teams.Length; i++)
        {
            minSectionTimes[i] = new Dictionary<int, float>();
            agentsPastSection[i] = new Dictionary<int, int>();
            m_AgentGroups.Add(new SimpleMultiAgentGroup());
            foreach (KartAgent k in Teams[i].Racers)
                m_AgentGroups[i].RegisterAgent(k);
        }
        goalSection = laps * Sections.Length + 1;
        //ResetGame();
    }

    void AddGoalTimingRewards()
    {
        if(Agents.Length == 1)
        {
            if (Agents[0].m_timeSteps != 0)
            {
                Agents[0].AddReward(ReachGoalCheckpointRewardMultplier * (1.0f-Agents[0].m_timeSteps*1.0f/maxEpisodeSteps) + ReachGoalCheckpointRewardBase);
            }
            return;
        }
        List<float> gtRewards = new List<float>();
        float maxReward = 1f;
        float minReward = -1f;
        float maxGTReward = -1000f;
        float minGTReward = 1000f;
        for (int i = 0; i < Agents.Length;i++)
        {
            if (Agents[i].m_timeSteps == 0)
                Agents[i].m_timeSteps = 2*maxEpisodeSteps;
        }
        for (int i = 0; i < Agents.Length; i++)
        {
            int opponentAgents = Agents[i].otherAgents.Length;
            int teamAgents = Agents[i].teamAgents.Length;
            int currAgentTeamScore = 0;
            int currAgentScore = Agents[i].m_timeSteps;
            int oppAgentScore = 0;
            for (int j = 0; j < Agents[i].otherAgents.Length; j++)
            {
                oppAgentScore += Agents[i].otherAgents[j].m_timeSteps;
            }
            for (int j = 0; j < Agents[i].teamAgents.Length; j++)
            {
                currAgentTeamScore += Agents[i].teamAgents[j].m_timeSteps;
            }
            float finalCurrAgentScore = currAgentScore + currAgentTeamScore * TeamScoreRewardMultiplier;
            float finalOpponentScore = oppAgentScore * (1f + teamAgents * TeamScoreRewardMultiplier)/ (opponentAgents * 1f);
            float reward = ((finalOpponentScore - finalCurrAgentScore)/(1f + teamAgents * TeamScoreRewardMultiplier))/maxEpisodeSteps;
            // maxReward = Mathf.Max(maxReward, reward);
            // minReward = Mathf.Min(minReward, reward);
            print(i + " individual reward is " + reward +" opp agent score " + finalOpponentScore + " curr agent team score " + finalCurrAgentScore);
            gtRewards.Add(reward);
        } 
        float[] groupRewards = new float[m_AgentGroups.Count];
        for (int i = 0; i < Agents.Length; i++)
        {
            if (Agents[i].Mode != AgentMode.Training) continue;
            if (maxReward != minReward)
            {
                float s = (ReachGoalCheckpointRewardBase + ReachGoalCheckpointRewardMultplier * ((gtRewards[i] - minReward) * 1.0f / (maxReward - minReward)));
                print("Goal checkpoint Agent " + i + " score is " + s + " with cumulative reward " + Agents[i].GetCumulativeReward());
                groupRewards[getTeamID(Agents[i])] += s;
            }
            else
            {
                groupRewards[getTeamID(Agents[i])] += 0f;
            }
        }
        for (int i = 0; i < m_AgentGroups.Count; i++)
        {
            m_AgentGroups[i].AddGroupReward(groupRewards[i]);
            m_AgentGroups[i].EndGroupEpisode();
        }
        
    }

    void FixedUpdate()
    {
        if (inactiveAgents.Count == Agents.Length)
        {

            // print("from here 1");
            if (initialStarted && mode == EnvironmentMode.Experiment && experimentNum < TotalExperiments)
            {
                TelemetryViewer tm = null;
                foreach(TelemetryViewer viewer in FindObjectsOfType<TelemetryViewer>())
                {
                    if (viewer.envController == this)
                    {
                        tm = viewer;
                        break;
                    }
                }
                print(tm.uiText.text);
                StreamWriter writer = new StreamWriter("ExperimentLogs/" + ExperimentName + ".txt", true);
                writer.WriteLine("Experiment " + experimentNum);
                writer.WriteLine(tm.uiText.text);
                writer.Close();
                experimentNum += 1;
            }

            AddGoalTimingRewards();
            ResetGame();
            if (!initialStarted && mode == EnvironmentMode.Experiment)
            {
                StreamWriter writer = new StreamWriter("ExperimentLogs/" + ExperimentName + ".txt", false);
                writer.Close();
            }
            initialStarted = true;
        }
        else
        {
            episodeSteps += 1;
            if (episodeSteps >= maxEpisodeSteps)
            {
                foreach (KartAgent agent in Agents)
                {
                    if (agent.is_active)
                        agent.Deactivate(disableOnEnd);
                }
                if (initialStarted && mode == EnvironmentMode.Experiment && experimentNum < TotalExperiments)
                {
                    TelemetryViewer tm = null;
                    foreach (TelemetryViewer viewer in FindObjectsOfType<TelemetryViewer>())
                    {
                        if (viewer.envController == this)
                        {
                            tm = viewer;
                            break;
                        }
                    }
                    print(tm.uiText.text);
                    StreamWriter writer = new StreamWriter("ExperimentLogs/" + ExperimentName + ".txt", true);
                    writer.WriteLine("Experiment " + experimentNum);
                    writer.WriteLine(tm.uiText.text);
                    writer.Close();
                    experimentNum += 1;
                }
                AddGoalTimingRewards();
                //print("from here 2");
                ResetGame();
            }
        }
        if (!coroStarted && mode == EnvironmentMode.Experiment)
        {
            StartCoroutine(checkAllExperimentsDone());
        }
    }

    IEnumerator checkAllExperimentsDone()
    {
        while (experimentNum == TotalExperiments)
        {
            coroStarted = true;
            bool quit = true;
            foreach (RacingEnvController controller in FindObjectsOfType<RacingEnvController>())
            {
                if (controller.experimentNum != controller.TotalExperiments)
                {
                    quit = false;
                }

            }
            if (quit)
            {
#if UNITY_EDITOR
                // Application.Quit() does not work in the editor so
                // UnityEditor.EditorApplication.isPlaying need to be set to false to end the game
                UnityEditor.EditorApplication.isPlaying = false;
#else
            Application.Quit();
#endif
            }
            yield return new WaitForSeconds(30f);
        }
    }

    public void timeDifferenceAtSectionPenalty(KartAgent agent)
    {
        int agentTeam = getTeamID(agent);
        int totalAgentsPastSection = 0;
        if (!minSectionTimes[agentTeam].ContainsKey(agent.m_SectionIndex))
        {
            //print(agent.name + "reached " + agent.m_SectionIndex +  " soonest");
            minSectionTimes[agentTeam][agent.m_SectionIndex] = episodeSteps;
            agentsPastSection[agentTeam][agent.m_SectionIndex] = 1;
            for (int i = 0; i < Teams.Length; i++)
            {
                if (i != agentTeam && minSectionTimes[i].ContainsKey(agent.m_SectionIndex))
                {
                    agent.AddReward(BeingBehindOpponentCheckpointPenalty * (episodeSteps - minSectionTimes[i][agent.m_SectionIndex]) * agentsPastSection[i][agent.m_SectionIndex]);
                    totalAgentsPastSection += agentsPastSection[i][agent.m_SectionIndex];
                }
            }
            totalAgentsPastSection += 1;
        } else
        {
            //print(agent.name + "reached " + agent.m_SectionIndex + " late by " + (episodeSteps - minSectionTimes[agent.m_SectionIndex]));
            for (int i = 0; i < Teams.Length; i++)
            {
                if (i == agentTeam)
                {
                    agent.AddReward(BeingBehindTeammateCheckpointPenalty * (episodeSteps - minSectionTimes[i][agent.m_SectionIndex]) * agentsPastSection[i][agent.m_SectionIndex]);
                    m_AgentGroups[i].AddGroupReward((1f - (episodeSteps * 1f) / (maxEpisodeSteps*m_AgentGroups[i].GetRegisteredAgents().Count)));
                }
                else if (minSectionTimes[i].ContainsKey(agent.m_SectionIndex))
                {
                    agent.AddReward(BeingBehindOpponentCheckpointPenalty * (episodeSteps - minSectionTimes[i][agent.m_SectionIndex]) * agentsPastSection[i][agent.m_SectionIndex]);
                    totalAgentsPastSection += agentsPastSection[i][agent.m_SectionIndex];
                } else
                {
                    m_AgentGroups[i].AddGroupReward(-(1f - (episodeSteps * 1f) / (maxEpisodeSteps * (Agents.Length - m_AgentGroups[i].GetRegisteredAgents().Count))));
                }
                
            }
            agentsPastSection[agentTeam][agent.m_SectionIndex] += 1;
            totalAgentsPastSection += 1;
        }
        for (int i = 0; i < Teams.Length; i++)
        {
            if (i == agentTeam)
            {
                m_AgentGroups[i].AddGroupReward((1f - (episodeSteps * 1f) / (maxEpisodeSteps * m_AgentGroups[i].GetRegisteredAgents().Count)));
            } else
            {
                m_AgentGroups[i].AddGroupReward(-(1f - (episodeSteps * 1f) / (maxEpisodeSteps * (Agents.Length - m_AgentGroups[i].GetRegisteredAgents().Count))));
            }
        }
        //float[] groupRewardMultipliers = {1f, 0.6f, 0.4f, 0.1f};
        //float groupRewardMult = groupRewardMultipliers[Math.Min(totalAgentsPastSection - 1, 3)];
        //m_AgentGroups[agentTeam].AddGroupReward(groupRewardMult * (groupRewardMult > 0 ? (1 - (episodeSteps * 1f)/maxEpisodeSteps) : (episodeSteps * 1f) / maxEpisodeSteps));

    }

    public void ResolveEvent(Event triggeringEvent, KartAgent triggeringAgent, HashSet<KartAgent> otherInvolvedAgents)
    {
        //print("Resolving event");
        //print(triggeringEvent);
        if (!triggeringAgent.is_active)
            return;
        switch (triggeringEvent)
        {
            case Event.HitWall:
                triggeringAgent.ApplyHitWallPenalty();
                break;
            case Event.HitOpponent:
                triggeringAgent.ApplyHitOpponentPenalty();
                foreach (KartAgent agent in otherInvolvedAgents)
                {
                    agent.ApplyHitByOpponentPenalty();
                }
                otherInvolvedAgents.Clear();
                break;
            case Event.ReachNonGoalSection:
                triggeringAgent.ApplySectionReward();
                triggeringAgent.AddReward(PassCheckpointBase);
                triggeringAgent.AddReward(PassCheckpointTimeMultiplier * (maxEpisodeSteps - episodeSteps)/(1f*maxEpisodeSteps));
                timeDifferenceAtSectionPenalty(triggeringAgent);
                break;
            case Event.ReachGoalSection:
                triggeringAgent.m_timeSteps = episodeSteps;
                triggeringAgent.Deactivate(disableOnEnd);
                inactiveAgents.Add(triggeringAgent);
                break;
            case Event.DroveReverseLimit:
                triggeringAgent.m_timeSteps = maxEpisodeSteps*3;
                triggeringAgent.Deactivate(disableOnEnd);
                inactiveAgents.Add(triggeringAgent);
                break;
            case Event.FellOffWorld:
                triggeringAgent.m_timeSteps = maxEpisodeSteps*3;
                triggeringAgent.Deactivate(disableOnEnd);
                inactiveAgents.Add(triggeringAgent);
                break;
        }
    }

    public void ResetSectionHighlighting()
    {
        foreach (DiscretePositionTracker section in Sections)
        {
            section.resetColors();
        }
    }

    void ResetGame()
    {
        float minTirewearProportion = mode == EnvironmentMode.Training ? 0.0f : 0.25f;
        float maxTirewearProportion = mode == EnvironmentMode.Training ? 1.0f : 0.25f;
        float minDistFromSpawn = mode == EnvironmentMode.Training ? 1.0f : 3.0f;
        float maxDistFromSpawn = mode == EnvironmentMode.Training ? 4.0f : 3.0f;
        episodeSteps = 0;
        inactiveAgents.Clear();

        for (int i = 0; i < Teams.Length; i++)
        {
            minSectionTimes[i].Clear();
            agentsPastSection[i].Clear();
        }
        // For each agent
        var furthestForwardSection = -1;
        var furthestBackSection = 100000;
        HashSet< Collider > addedColliders = new HashSet<Collider>();
        bool headToHead = mode == EnvironmentMode.Training ? UnityEngine.Random.Range(0, 9) != 1 : true;
        var initialSection = -1;
        var minSectionIndex = mode == EnvironmentMode.Training ? 2 : 0;
        var maxSectionIndex = mode == EnvironmentMode.Training? goalSection-2 : 0;
        int laneDirection = experimentNum % 2 == 0 ? 1: -1;
        var expLaneChoices = new int[] { 2, 3 };
        int lastPickedLaneIdx = 0;
        for (int i = 0; i < Agents.Length; i++)
        {
            if (!headToHead)
            {
                // Randomly Set Iniital Track Position
                while (true)
                {
                    Agents[i].m_SectionIndex = UnityEngine.Random.Range(minSectionIndex, maxSectionIndex);
                    Agents[i].InitCheckpointIndex = Agents[i].m_SectionIndex;
                    Agents[i].m_Lane = UnityEngine.Random.Range(1, 5);
                    Agents[i].m_LaneChanges = 0;
                    Agents[i].m_IllegalLaneChanges = 0;
                    var collider = Sections[Agents[i].m_SectionIndex % Sections.Length].getBoxColliderForLane(Agents[i].m_Lane);
                    if (!addedColliders.Contains(collider))
                    {
                        Agents[i].m_Kart.m_AccumulatedAngularV = UnityEngine.Random.Range(Agents[i].m_Kart.TireWearRate*minTirewearProportion, Agents[i].m_Kart.TireWearRate * maxTirewearProportion);
                        furthestForwardSection = Math.Max(Agents[i].m_SectionIndex, furthestForwardSection);
                        furthestBackSection = Math.Min(Agents[i].m_SectionIndex, furthestBackSection);
                        Agents[i].transform.rotation = collider.transform.rotation;
                        Agents[i].transform.position = collider.transform.position + (collider.transform.rotation * Vector3.forward).normalized * UnityEngine.Random.Range(minDistFromSpawn, maxDistFromSpawn);
                        Agents[i].GetComponent<Rigidbody>().transform.rotation = collider.transform.rotation;
                        Agents[i].GetComponent<Rigidbody>().transform.position = collider.transform.position + (collider.transform.rotation * Vector3.forward).normalized * UnityEngine.Random.Range(minDistFromSpawn, maxDistFromSpawn);
                        Agents[i].GetComponent<Rigidbody>().velocity = Vector3.zero;
                        Agents[i].sectionTimes.Clear();
                        Agents[i].m_UpcomingLanes.Clear();
                        Agents[i].m_UpcomingVelocities.Clear();
                        addedColliders.Add(collider);
                        break;
                    }
                }
            }
            else
            {
                if (addedColliders.Count == 0)
                {
                    Agents[i].m_SectionIndex = UnityEngine.Random.Range(minSectionIndex, maxSectionIndex);
                    initialSection = Agents[i].m_SectionIndex;
                    Agents[i].InitCheckpointIndex = Agents[i].m_SectionIndex;
                    Agents[i].m_Kart.m_AccumulatedAngularV = UnityEngine.Random.Range(Agents[i].m_Kart.TireWearRate * minTirewearProportion, Agents[i].m_Kart.TireWearRate * maxTirewearProportion);
                    furthestForwardSection = Math.Max(Agents[i].m_SectionIndex, furthestForwardSection);
                    furthestBackSection = Math.Min(Agents[i].m_SectionIndex, furthestBackSection);
                    if (mode == EnvironmentMode.Experiment)
                    {
                        if (laneDirection == 1)
                        {
                            Agents[i].m_Lane = expLaneChoices[0];
                            lastPickedLaneIdx = 0;
                        } else
                        {
                            Agents[i].m_Lane = expLaneChoices[expLaneChoices.Length - 1];
                            lastPickedLaneIdx = expLaneChoices.Length - 1;
                        }
                    }
                    else
                    {
                        Agents[i].m_Lane = UnityEngine.Random.Range(1, 5);
                    }
                    Agents[i].m_LaneChanges = 0;
                    Agents[i].m_IllegalLaneChanges = 0;
                    var collider = Sections[Agents[i].m_SectionIndex % Sections.Length].getBoxColliderForLane(Agents[i].m_Lane);
                    Agents[i].transform.rotation = collider.transform.rotation;
                    Agents[i].transform.position = collider.transform.position + (collider.transform.rotation * Vector3.forward).normalized * UnityEngine.Random.Range(minDistFromSpawn, maxDistFromSpawn);
                    Agents[i].GetComponent<Rigidbody>().transform.rotation = collider.transform.rotation;
                    Agents[i].GetComponent<Rigidbody>().transform.position = collider.transform.position + (collider.transform.rotation * Vector3.forward).normalized * UnityEngine.Random.Range(minDistFromSpawn, maxDistFromSpawn);
                    Agents[i].GetComponent<Rigidbody>().velocity = Vector3.zero;
                    Agents[i].sectionTimes.Clear();
                    Agents[i].m_UpcomingLanes.Clear();
                    Agents[i].m_UpcomingVelocities.Clear();
                    addedColliders.Add(collider);
                }
                else
                {
                    while (true)
                    {
                        // print("Trying to find a spot for " + Agents[i].name + " near section " + initialSection);
                        if (mode == EnvironmentMode.Training)
                            Agents[i].m_SectionIndex = UnityEngine.Random.Range(Math.Max(initialSection - 1, 0), initialSection + 1);
                        else
                            Agents[i].m_SectionIndex = UnityEngine.Random.Range(initialSection, initialSection);
                        Agents[i].InitCheckpointIndex = Agents[i].m_SectionIndex;
                        if (mode == EnvironmentMode.Experiment)
                        {
                            Agents[i].m_Lane = expLaneChoices[lastPickedLaneIdx + laneDirection];
                            lastPickedLaneIdx += laneDirection;
                        }
                        else
                        {
                            Agents[i].m_Lane = UnityEngine.Random.Range(1, 5);
                        }
                        Agents[i].m_LaneChanges = 0;
                        Agents[i].m_IllegalLaneChanges = 0;
                        var collider = Sections[Agents[i].m_SectionIndex % Sections.Length].getBoxColliderForLane(Agents[i].m_Lane);
                        if (!addedColliders.Contains(collider))
                        {
                            Agents[i].m_Kart.m_AccumulatedAngularV = UnityEngine.Random.Range(Agents[i].m_Kart.TireWearRate * minTirewearProportion, Agents[i].m_Kart.TireWearRate * maxTirewearProportion);
                            furthestForwardSection = Math.Max(Agents[i].m_SectionIndex, furthestForwardSection);
                            furthestBackSection = Math.Min(Agents[i].m_SectionIndex, furthestBackSection);
                            Agents[i].transform.rotation = collider.transform.rotation;
                            Agents[i].transform.position = collider.transform.position + (collider.transform.rotation * Vector3.forward).normalized * UnityEngine.Random.Range(minDistFromSpawn, maxDistFromSpawn);
                            Agents[i].GetComponent<Rigidbody>().transform.rotation = collider.transform.rotation;
                            Agents[i].GetComponent<Rigidbody>().transform.position = collider.transform.position + (collider.transform.rotation * Vector3.forward).normalized * UnityEngine.Random.Range(minDistFromSpawn, maxDistFromSpawn);
                            Agents[i].GetComponent<Rigidbody>().velocity = Vector3.zero;
                            Agents[i].sectionTimes.Clear();
                            Agents[i].m_UpcomingLanes.Clear();
                            Agents[i].m_UpcomingVelocities.Clear();
                            addedColliders.Add(collider);
                            break;
                        }
                    }
                }
            }
        }

        // Use the furthest forward agent to determine the final goal track section by adding a fixed amount of sections to it
        var maxSection = furthestForwardSection + sectionHorizon;
        for (int i = 0; i < Agents.Length; i++)
        {
            // Generate times for reaching certain sections upto random one
            int earliestTime = -maxEpisodeSteps;
            for (int tp = furthestBackSection; tp < Agents[i].m_SectionIndex; tp++)
            {
                Agents[i].sectionTimes[tp] = UnityEngine.Random.Range(earliestTime, 0);
                earliestTime = Agents[i].sectionTimes[tp];
            }
            Agents[i].sectionTimes[Agents[i].m_SectionIndex] = 0;
        }

        ResetSectionHighlighting();
        for (int i = 0; i < Agents.Length; i++)
        {
            // Then, for each agent, Generate the initial plan
            Agents[i].prepareForReuse();
            Agents[i].initialPlan();
        }

        for (int i = 0; i < Agents.Length; i++)
        {
            Agents[i].Activate();
            Agents[i].OnEpisodeBegin();
        }
    }

    public bool sectionIsStraight(int section)
    {
        return Sections[section % Sections.Length].isStraight();
    }

    public bool sectionSpeedFeasible(int section, int velocity, int initLane, int finalLane, float tireAge, ArcadeKart m_Kart)
    {
        return Sections[section % Sections.Length].isVelFeasible(velocity, initLane, finalLane, tireAge, m_Kart.m_FinalStats.MaxGs, m_Kart.m_FinalStats.MinGs);
    }

    public float computeDistanceInSection(int section, int initLane, int finalLane)
    {
        // print("Going from lane " + initLane + " to lane " + finalLane + " in section " + Sections[section % Sections.Length].name + "is this long " + Sections[section % Sections.Length].distanceToTravel(initLane, finalLane));
        return Sections[section % Sections.Length].distanceToTravel(initLane, finalLane);
    }

    public float computeTireLoadInSection(int section, int max_velocity, int initLane, int finalLane)
    {
        // print("Section " + section + " tire load is " + Sections[section % Sections.Length].tireLoad(max_velocity, initLane, finalLane) + " with velocity" + max_velocity);
        return Sections[section % Sections.Length].tireLoad(max_velocity, initLane, finalLane);
    }

    public int getTeamID(KartAgent k)
    {
        for(int i = 0; i < Teams.Length; i++)
        {
            if (Teams[i].Racers.Contains(k))
            {
                return i;
            }
        }
        return -1;
    }
}

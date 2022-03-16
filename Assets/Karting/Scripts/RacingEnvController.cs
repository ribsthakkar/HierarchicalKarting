using KartGame.AI;
using KartGame.KartSystems;
using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Linq;
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

/**
* Class responsible for managing the racing environment
* Resets differently depending on whether it is set for Racing, Experiment or Training modes
* Outlines the teams in the game
* Stores the rewards that the RL algorithms use during training
**/
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
    [Tooltip("How much reward is given when the agent successfully passes the checkpoint with desired lane?")]
    public float PassCheckpointLaneReward = 4f;
    [Tooltip("How much reward is given when the agent successfully passes the checkpoint with desired velocity?")]
    public float PassCheckpointVelocityReward = 4f;
    [Tooltip("How much reward is given when the agent successfully passes the checkpoints?")]
    public float PassCheckpointBase = 20f;
    [Tooltip("How much reward is given when the agent successfully passes the checkpoints in faster time")]
    public float PassCheckpointTimeMultiplier = 5f;
    [Tooltip("How much reward is given when the agent successfully passes the checkpoints?")]
    public float TeamPassCheckpointBase = 20f;
    [Tooltip("How much reward is given when the agent successfully passes the checkpoints in faster time")]
    public float TeamPassCheckpointTimeMultiplier = 5f;
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
    [Tooltip("Penalty the agent for existing and not being at goal")]
    public float NotAtGoalPenalty = -0.001f;
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
    List<List<int>> allOrderings;
    [Header("Training Params")]
    public int episodeSteps;
    public int maxEpisodeSteps;
    public bool disableOnEnd;
    [Tooltip("How many sections ahead should the agents be looking ahead/driving to")]
    public int sectionHorizon;
    public bool highlightWaypoints;

    bool initialStarted = false;
    bool coroStarted = false;


    // Retreived from stack overflow here: https://stackoverflow.com/questions/756055/listing-all-permutations-of-a-string-integer
    public static IEnumerable<IEnumerable<T>>
    GetPermutations<T>(IEnumerable<T> list, int length)
    {
        if (length == 1) return list.Select(t => new T[] { t });

        return GetPermutations(list, length - 1)
            .SelectMany(t => list.Where(e => !t.Contains(e)),
                (t1, t2) => t1.Concat(new T[] { t2 })).ToList();
    }

    // Start is called before the first frame update
    void Start()
    {
        // Create Agent groups and calcualte the checkpoint index of the goal based on the number of laps.
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
        allOrderings = GetPermutations(Enumerable.Range(0, Agents.Length), Agents.Length).Select(perm => perm.ToList()).ToList();
        //ResetGame();
    }

    /**
    * Rewards added after the game has run out of time or all players have reached teh goal
    * The score is normalized between -1 and 1 and euqally applied to all team members depending on the overall performance of the teams
    **/
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
            // print(i + " individual reward is " + reward +" opp agent score " + finalOpponentScore + " curr agent team score " + finalCurrAgentScore);
            gtRewards.Add(reward);
        } 
        float[] groupRewards = new float[m_AgentGroups.Count];
        for (int i = 0; i < Agents.Length; i++)
        {
            if (Agents[i].Mode != AgentMode.Training) continue;
            if (maxReward != minReward)
            {
                float s = (ReachGoalCheckpointRewardBase + ReachGoalCheckpointRewardMultplier * ((gtRewards[i] - minReward) * 1.0f / (maxReward - minReward)));
                // print("Goal checkpoint Agent " + i + " score is " + s + " with cumulative reward " + Agents[i].GetCumulativeReward());
                groupRewards[getTeamID(Agents[i])] += s;
            }
            else
            {
                groupRewards[getTeamID(Agents[i])] += 0f;
            }
        }
        for (int i = 0; i < m_AgentGroups.Count; i++)
        {
            // print(name + " Group reward to add " + (groupRewards[i] / Teams[i].Racers.Count));
            m_AgentGroups[i].AddGroupReward(groupRewards[i]/Teams[i].Racers.Count);
            m_AgentGroups[i].EndGroupEpisode();
        }
        
    }

    void FixedUpdate()
    {
        if (inactiveAgents.Count == Agents.Length) // Everyone has finished or deactivated
        {
            foreach (KartAgent agent in Agents)
            {
                if (agent.is_active)
                    agent.Deactivate(disableOnEnd);
            }
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
            }

            AddGoalTimingRewards();
            if (initialStarted)
                experimentNum += 1;
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
            if (episodeSteps >= maxEpisodeSteps) // Game has run out of time
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
                }
                AddGoalTimingRewards();
                //print("from here 2");
                experimentNum += 1;
                ResetGame();
            }
        }

        if (!coroStarted && mode == EnvironmentMode.Experiment)
        {
            StartCoroutine(checkAllExperimentsDone());
        }
        //for (int i =0; i < Agents.Length; i ++)
        //{
        //    print(episodeSteps+ ":" + Agents[i].name + " " + Agents[i].m_Kart.Rigidbody.velocity);
        //}
    }

    /**
    * Coroutine to check all experiments done when multiple environments are runnign experiments simulatenously
    * Once they all finish, the game ends.
    **/
    IEnumerator checkAllExperimentsDone()
    {
        while (experimentNum == TotalExperiments)
        {
            coroStarted = true;
            bool quit = true;
            foreach (RacingEnvController controller in FindObjectsOfType<RacingEnvController>())
            {
                if (controller.experimentNum < controller.TotalExperiments)
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

    /**
    * For an agent, apply penalties and rewards based on the time at which they reached the section
    * Also include group rewards/penalties for a team's overall performance
    **/
    public void ApplySectionRewardsAndPenalties(KartAgent agent)
    {
        agent.ApplySectionReward();
        //agent.AddReward(PassCheckpointBase + PassCheckpointTimeMultiplier * (maxEpisodeSteps - episodeSteps) / (1f * maxEpisodeSteps));

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
                    agent.AddReward(BeingBehindOpponentCheckpointPenalty * (episodeSteps - minSectionTimes[i][agent.m_SectionIndex]) * agentsPastSection[i][agent.m_SectionIndex]/(1f * (Agents.Length - Teams[i].Racers.Count)));
                    totalAgentsPastSection += agentsPastSection[i][agent.m_SectionIndex];
                }
            }
            totalAgentsPastSection += 1;
        }
        else
        {
            //print(agent.name + "reached " + agent.m_SectionIndex + " late by " + (episodeSteps - minSectionTimes[agent.m_SectionIndex]));
            for (int i = 0; i < Teams.Length; i++)
            {
                if (i == agentTeam)
                {
                    agent.AddReward(BeingBehindTeammateCheckpointPenalty * (episodeSteps - minSectionTimes[i][agent.m_SectionIndex]) * agentsPastSection[i][agent.m_SectionIndex] / (1f * (Teams[i].Racers.Count)));
                }
                else if (minSectionTimes[i].ContainsKey(agent.m_SectionIndex))
                {
                    agent.AddReward(BeingBehindOpponentCheckpointPenalty * (episodeSteps - minSectionTimes[i][agent.m_SectionIndex]) * agentsPastSection[i][agent.m_SectionIndex] / (1f * (Agents.Length - Teams[i].Racers.Count)));
                    totalAgentsPastSection += agentsPastSection[i][agent.m_SectionIndex];
                } 
                
            }
            agentsPastSection[agentTeam][agent.m_SectionIndex] += 1;
            totalAgentsPastSection += 1;
        }
        //float[] groupRewardMultipliers = {0.1f, 0.075f, 0.05f, 0.05f};
        //float groupRewardMult = groupRewardMultipliers[Math.Min(totalAgentsPastSection - 1, 3)];
        //for (int i = 0; i < Teams.Length; i++)
        //{
        //    float timeRemainingProportion = (1f - (episodeSteps * 1f) / (maxEpisodeSteps));
        //    if (i == agentTeam)
        //    {
        //       m_AgentGroups[i].AddGroupReward(groupRewardMult*timeRemainingProportion/ Teams[i].Racers.Count);
        //    }
        //    else
        //    {
        //        m_AgentGroups[i].AddGroupReward(-groupRewardMult*timeRemainingProportion/(Agents.Length - Teams[i].Racers.Count));
        //    }
        //}

        float[] agentRewardMultipliers = { PassCheckpointTimeMultiplier, PassCheckpointTimeMultiplier * 0.75f, PassCheckpointTimeMultiplier * 0.6f, PassCheckpointTimeMultiplier * 0.4f };
        float[] agentRewardBase = {PassCheckpointBase, PassCheckpointBase * 0.75f, PassCheckpointBase * 0.6f, PassCheckpointBase * 0.4f };
        float agentRewardMult = agentRewardMultipliers[Math.Min(totalAgentsPastSection - 1, 3)];
        float agentRewardB = agentRewardBase[Math.Min(totalAgentsPastSection - 1, 3)];

        agent.AddReward(agentRewardB + agentRewardMult * (maxEpisodeSteps - episodeSteps) / (1f * maxEpisodeSteps));

        float[] groupRewardMultipliers = { TeamPassCheckpointTimeMultiplier, TeamPassCheckpointTimeMultiplier * 0.75f, TeamPassCheckpointTimeMultiplier * 0.6f, TeamPassCheckpointTimeMultiplier * 0.4f };
        float[] groupRewardBase = { TeamPassCheckpointBase, TeamPassCheckpointBase * 0.75f, TeamPassCheckpointBase * 0.6f, TeamPassCheckpointBase * 0.4f };
        float groupRewardMult = groupRewardMultipliers[Math.Min(totalAgentsPastSection - 1, 3)];
        float groupRewardB = groupRewardBase[Math.Min(totalAgentsPastSection - 1, 3)];
        m_AgentGroups[agentTeam].AddGroupReward(groupRewardB + groupRewardMult  * (maxEpisodeSteps - episodeSteps) / (1f * maxEpisodeSteps));


        //float[] groupRewardMultipliers = { 0.3f, 0.15f, -0.15f, -0.3f };
        //float groupRewardMult = groupRewardMultipliers[Math.Min(totalAgentsPastSection - 1, 3)];
        //m_AgentGroups[agentTeam].AddGroupReward(groupRewardMult * (groupRewardMult > 0 ? (1 - (episodeSteps * 1f) / maxEpisodeSteps) : (episodeSteps * 1f) / maxEpisodeSteps));

    }

    /**
    * Switch to handle various events that occur in the environment and the penalties or rewards that need to be enacted based on the involving/triggering agents
    **/
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
                    // Extra penalties for crashing into teammate
                    if (getTeamID(triggeringAgent) == getTeamID(agent))
                    {
                        triggeringAgent.ApplyHitOpponentPenalty(2.5f);
                        agent.ApplyHitByOpponentPenalty(1.5f);
                    }
                    else
                    {
                        agent.ApplyHitByOpponentPenalty();
                    }
                }
                otherInvolvedAgents.Clear();
                break;
            case Event.ReachNonGoalSection:
                ApplySectionRewardsAndPenalties(triggeringAgent);
                break;
            case Event.ReachGoalSection:
                triggeringAgent.m_timeSteps = episodeSteps;
                ApplySectionRewardsAndPenalties(triggeringAgent);
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

    /**
    * Routine to reset the game.
    **/
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
            foreach (KartAgent k in Teams[i].Racers)
            {
                if (!m_AgentGroups[i].GetRegisteredAgents().Contains(k))
                    m_AgentGroups[i].RegisterAgent(k);
            }
        }
        // For each agent
        var furthestForwardSection = -1;
        var furthestBackSection = 100000;
        HashSet< Collider > addedColliders = new HashSet<Collider>();
        bool headToHead = mode == EnvironmentMode.Training ? UnityEngine.Random.Range(0, 9) != 1 : true;
        var initialSection = -1;
        var minSectionIndex = 0;
        var maxSectionIndex = mode == EnvironmentMode.Training? goalSection : 0;
        int laneDirection = 1;
        var expLaneChoices = new int[] { 2, 3, 1, 4, 1, 4, 2, 3 };
        int lastPickedLaneIdx = 0;
        for (int j = 0; j < allOrderings[experimentNum % allOrderings.Count()].Count(); j++)
        {
            int i = allOrderings[experimentNum % allOrderings.Count()][j];
            if (!headToHead)
            {
                // Randomly Set Iniital Track Position for all agents (only used in Training mode)
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
                // Set the first agent so we can then place the other agents near them in the headtohead mode
                if (addedColliders.Count == 0)
                {
                    Agents[i].m_SectionIndex = UnityEngine.Random.Range(minSectionIndex, maxSectionIndex);
                    initialSection = Agents[i].m_SectionIndex;
                    Agents[i].InitCheckpointIndex = Agents[i].m_SectionIndex;
                    Agents[i].m_Kart.m_AccumulatedAngularV = UnityEngine.Random.Range(Agents[i].m_Kart.TireWearRate * minTirewearProportion, Agents[i].m_Kart.TireWearRate * maxTirewearProportion);
                    furthestForwardSection = Math.Max(Agents[i].m_SectionIndex, furthestForwardSection);
                    furthestBackSection = Math.Min(Agents[i].m_SectionIndex, furthestBackSection);
                    if (mode == EnvironmentMode.Experiment || mode == EnvironmentMode.Race)
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
                else // Place remaining agents nearby. +/- one checkpoint
                {
                    while (true)
                    {
                        // print("Trying to find a spot for " + Agents[i].name + " near section " + initialSection);
                        if (mode == EnvironmentMode.Training)
                            Agents[i].m_SectionIndex = UnityEngine.Random.Range(Math.Max(initialSection - 1, 0), Math.Min(initialSection + 2, maxSectionIndex));
                        else
                            Agents[i].m_SectionIndex = UnityEngine.Random.Range(initialSection, initialSection);
                        Agents[i].InitCheckpointIndex = Agents[i].m_SectionIndex;
                        if (mode == EnvironmentMode.Experiment || mode == EnvironmentMode.Race)
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
        // Also update number of agents past the section
        var maxSection = furthestForwardSection + sectionHorizon;
        for (int i = 0; i < Agents.Length; i++)
        {
            int team = getTeamID(Agents[i]);
            // Generate times for reaching certain sections upto random one
            int earliestTime = -maxEpisodeSteps;
            for (int tp = furthestBackSection; tp < Agents[i].m_SectionIndex; tp++)
            {
                Agents[i].sectionTimes[tp] = UnityEngine.Random.Range(earliestTime, 0);
                earliestTime = Agents[i].sectionTimes[tp];
                if (!agentsPastSection[team].ContainsKey(tp))
                    agentsPastSection[team][tp] = 1;
                else
                    agentsPastSection[team][tp] += 1;
            }
            Agents[i].sectionTimes[Agents[i].m_SectionIndex] = 0;
            if (!agentsPastSection[team].ContainsKey(Agents[i].m_SectionIndex))
                agentsPastSection[team][Agents[i].m_SectionIndex] = 1;
            else
                agentsPastSection[team][Agents[i].m_SectionIndex] += 1;
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
            Agents[i].m_Kart.Rigidbody.transform.position = new Vector3(Agents[i].transform.position.x, 0.28f, Agents[i].transform.position.z);
            Agents[i].transform.position = Agents[i].m_Kart.Rigidbody.transform.position;
        }
        StartCoroutine(StartRaceAfterDelay());
    }

    IEnumerator StartRaceAfterDelay()
    {
        if (mode != EnvironmentMode.Training)
            yield return new WaitForSeconds(1.5f);
        for (int i = 0; i < Agents.Length; i++)
        {
            Agents[i].m_Kart.Rigidbody.constraints = RigidbodyConstraints.None;
            Agents[i].m_Kart.Rigidbody.velocity = Vector3.zero;
            Agents[i].m_Kart.SetCanMove(true);
            Agents[i].OnEpisodeBegin();
        }
        // resetted = true;
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

    public float computeAvgSectionRadius(int section, int initLane, int finalLane)
    {
        // print("Going from lane " + initLane + " to lane " + finalLane + " in section " + Sections[section % Sections.Length].name + "is this long " + Sections[section % Sections.Length].distanceToTravel(initLane, finalLane));
        return Sections[section % Sections.Length].radiusOfLane(initLane, finalLane);
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

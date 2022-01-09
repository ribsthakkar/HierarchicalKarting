using KartGame.AI;
using KartGame.KartSystems;
using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Threading;
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


public class RacingEnvController : MonoBehaviour
{
    [Header("Racers"), Tooltip("What are the racing agents in the game?")]
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
    public float BeingBehindCheckpoingPenalty = -0.1f;
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
    [HideInInspector] public Dictionary<int, float> minSectionTimes = new Dictionary<int, float>();

    [Header("Training Params")]
    public int episodeSteps;
    public int maxEpisodeSteps;
    [Tooltip("How many sections ahead should the agents be looking ahead/driving to")]
    public int sectionHorizon;

    bool initialStarted = false;

    // Start is called before the first frame update
    void Start()
    {
        for (int i = 0; i < Agents.Length; i++)
        {
            inactiveAgents.Add(Agents[i]);
            AgentBodies[Agents[i].GetComponent<Rigidbody>()] = Agents[i];
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
        for (int i = 0; i < Agents.Length; i++)
        {
            int currAgentScore = (Agents.Length - 1)*Agents[i].m_timeSteps;
            int oppAgentScore = 0;
            for (int j = 0; j < Agents.Length; j++)
            {
                if (i != j)
                    oppAgentScore += Agents[j].m_timeSteps;

            }
            Agents[i].AddReward(-ReachGoalCheckpointRewardMultplier * (currAgentScore - oppAgentScore)/(1f*(Agents.Length-1)*maxEpisodeSteps));
            if (currAgentScore - oppAgentScore < 0)
            {
                Agents[i].AddReward(1);
            } else if (currAgentScore - oppAgentScore > 0)
            {
                Agents[i].AddReward(-1);
            } else
            {
                Agents[i].AddReward(0);
            }
        }
    }

    void FixedUpdate()
    {
        if (inactiveAgents.Count == Agents.Length)
        {

            // print("from here 1");
            if (initialStarted && mode == EnvironmentMode.Experiment && experimentNum < TotalExperiments)
            {
                var tm = FindObjectOfType<TelemetryViewer>();
                print(tm.uiText.text);
                StreamWriter writer = new StreamWriter("ExperimentLogs/" + ExperimentName + ".txt", true);
                writer.WriteLine("Experiment " + experimentNum);
                writer.WriteLine(tm.uiText.text);
                writer.Close();
                experimentNum += 1;
            }

            AddGoalTimingRewards();
            for (int i = 0; i < Agents.Length; i++)
            {
                if (initialStarted)
                {
                    Agents[i].EndEpisode();
                }
            }
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
                if (initialStarted && mode == EnvironmentMode.Experiment && experimentNum < TotalExperiments)
                {
                    var tm = FindObjectOfType<TelemetryViewer>();
                    print(tm.uiText.text);
                    StreamWriter writer = new StreamWriter("ExperimentLogs/" + ExperimentName + ".txt", true);
                    writer.WriteLine("Experiment " + experimentNum);
                    writer.WriteLine(tm.uiText.text);
                    writer.Close();
                    experimentNum += 1;
                }
                AddGoalTimingRewards();
                for (int i = 0; i < Agents.Length; i++)
                {
                    Agents[i].EndEpisode();
                }
                //print("from here 2");
                ResetGame();
            }
        }
        if (experimentNum == TotalExperiments && mode == EnvironmentMode.Experiment)
        {
#if UNITY_EDITOR
            // Application.Quit() does not work in the editor so
            // UnityEditor.EditorApplication.isPlaying need to be set to false to end the game
            UnityEditor.EditorApplication.isPlaying = false;
#else
            Application.Quit();
#endif
        }
    }

    public void timeDifferenceAtSectionPenalty(KartAgent agent)
    {
        if (!minSectionTimes.ContainsKey(agent.m_SectionIndex))
        {
            //print(agent.name + "reached " + agent.m_SectionIndex +  " soonest");
            minSectionTimes[agent.m_SectionIndex] = episodeSteps;
        } else
        {
            //print(agent.name + "reached " + agent.m_SectionIndex + " late by " + (episodeSteps - minSectionTimes[agent.m_SectionIndex]));
            agent.AddReward(BeingBehindCheckpoingPenalty * (episodeSteps - minSectionTimes[agent.m_SectionIndex]));
        }
    }

    public void ResolveEvent(Event triggeringEvent, KartAgent triggeringAgent, HashSet<KartAgent> otherInvolvedAgents)
    {
        //print("Resolving event");
        //print(triggeringEvent);
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
                triggeringAgent.Deactivate();
                inactiveAgents.Add(triggeringAgent);
                break;
            case Event.DroveReverseLimit:
                triggeringAgent.m_timeSteps = maxEpisodeSteps*2;
                triggeringAgent.Deactivate();
                inactiveAgents.Add(triggeringAgent);
                break;
            case Event.FellOffWorld:
                triggeringAgent.m_timeSteps = maxEpisodeSteps*2;
                triggeringAgent.Deactivate();
                inactiveAgents.Add(triggeringAgent);
                break;
        }
    }


    void ResetGame()
    {
        //print("resetting game");
        //if (mode == EnvironmentMode.Training)
        //{
        //    laps = UnityEngine.Random.Range(1, 5);
        //    maxEpisodeSteps = laps * 1500;
        //    goalSection = laps * Sections.Length + 1;
        //}
        float minTirewearProportion = mode == EnvironmentMode.Training ? 0.0f : 0.25f;
        float maxTirewearProportion = mode == EnvironmentMode.Training ? 1.0f : 0.25f;
        float minDistFromSpawn = mode == EnvironmentMode.Training ? 1.0f : 3.0f;
        float maxDistFromSpawn = mode == EnvironmentMode.Training ? 4.0f : 3.0f;
        foreach (DiscretePositionTracker section in Sections)
        {
            section.resetColors();
        }
        episodeSteps = 0;
        inactiveAgents.Clear();
        // For each agent
        var furthestForwardSection = -1;
        var furthestBackSection = 100000;
        HashSet< Collider > addedColliders = new HashSet<Collider>();
        bool headToHead = mode == EnvironmentMode.Training ? UnityEngine.Random.Range(0, 9) != 1 : true;
        var initialSection = -1;
        var minSectionIndex = 0;
        var maxSectionIndex = mode == EnvironmentMode.Training? goalSection : 0;
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
                    Agents[i].m_Lane = UnityEngine.Random.Range(1, 4);
                    Agents[i].m_LaneChanges = 0;
                    var collider = Sections[Agents[i].m_SectionIndex % Sections.Length].getBoxColliderForLane(Agents[i].m_Lane);
                    if (!addedColliders.Contains(collider))
                    {
                        Agents[i].m_Kart.m_AccumulatedAngularV = UnityEngine.Random.Range(Agents[i].m_Kart.TireWearRate*minTirewearProportion, Agents[i].m_Kart.TireWearRate * maxTirewearProportion);
                        furthestForwardSection = Math.Max(Agents[i].m_SectionIndex, furthestForwardSection);
                        furthestBackSection = Math.Min(Agents[i].m_SectionIndex, furthestBackSection);
                        Agents[i].transform.localRotation = collider.transform.rotation;
                        Agents[i].transform.position = collider.transform.position + (collider.transform.rotation * Vector3.forward).normalized * UnityEngine.Random.Range(minDistFromSpawn, maxDistFromSpawn);
                        Agents[i].GetComponent<Rigidbody>().transform.localRotation = collider.transform.rotation;
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
                        Agents[i].m_Lane = UnityEngine.Random.Range(1, 4);
                    }
                    Agents[i].m_LaneChanges = 0;
                    var collider = Sections[Agents[i].m_SectionIndex % Sections.Length].getBoxColliderForLane(Agents[i].m_Lane);
                    Agents[i].transform.localRotation = collider.transform.rotation;
                    Agents[i].transform.position = collider.transform.position + (collider.transform.rotation * Vector3.forward).normalized * UnityEngine.Random.Range(minDistFromSpawn, maxDistFromSpawn);
                    Agents[i].GetComponent<Rigidbody>().transform.localRotation = collider.transform.rotation;
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
                            Agents[i].m_Lane = UnityEngine.Random.Range(1, 4);
                        }
                        Agents[i].m_LaneChanges = 0;
                        var collider = Sections[Agents[i].m_SectionIndex % Sections.Length].getBoxColliderForLane(Agents[i].m_Lane);
                        if (!addedColliders.Contains(collider))
                        {
                            Agents[i].m_Kart.m_AccumulatedAngularV = UnityEngine.Random.Range(Agents[i].m_Kart.TireWearRate * minTirewearProportion, Agents[i].m_Kart.TireWearRate * maxTirewearProportion);
                            furthestForwardSection = Math.Max(Agents[i].m_SectionIndex, furthestForwardSection);
                            furthestBackSection = Math.Min(Agents[i].m_SectionIndex, furthestBackSection);
                            Agents[i].transform.localRotation = collider.transform.rotation;
                            Agents[i].transform.position = collider.transform.position + (collider.transform.rotation * Vector3.forward).normalized * UnityEngine.Random.Range(minDistFromSpawn, maxDistFromSpawn);
                            Agents[i].GetComponent<Rigidbody>().transform.localRotation = collider.transform.rotation;
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
            Agents[i].m_Kart.UpdateStats();
        }

        for (int i = 0; i < Agents.Length; i++)
        {
            // Then, for each agent, Generate the initial plan
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

}

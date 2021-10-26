using KartGame.AI;
using System;
using System.Collections.Generic;
using UnityEngine;



public enum Event
{
    HitSomething = 1,
    ReachNonGoalSection = 2,
    ReachGoalSection = 3
}



public class RacingEnvController : MonoBehaviour
{
    [Header("Racers"), Tooltip("What are the racing agents in the game?")]
    public KartAgent[] Agents;
    [Header("Checkpoints"), Tooltip("What are the series of checkpoints for the agent to seek and pass through?")]
    public DiscretePositionTracker[] Sections;
    [Tooltip("How many sections ahead should the agents be looking ahead/driving to")]
    public int sectionHorizon;

    [Tooltip("How many laps is considered reaching the goal")]
    public int laps=2;

    #region Rewards
    [Header("Rewards"), Tooltip("What penatly is given when the agent crashes with a wall?")]
    public float WallHitPenalty = -0.01f;
    [Tooltip("What penatly is given when the agent crashes into another agent?")]
    public float OpponentHitPenalty = -2f;
    [Tooltip("What penatly is given when the agent is crashed by another agent?")]
    public float HitByOpponentPenalty = -1f;
    [Tooltip("How much reward is given when the agent successfully passes the checkpoints?")]
    public float PassCheckpointReward = 0.1f;
    [Tooltip("How much the normalized remaining time is multiplied by to provide as a reward for the agent reaching the goal checkpoint?")]
    public float ReachGoalCheckpointRewardMultplier = 5.0f;
    [Tooltip("How much reward base reward is given to the agent for reaching the goal checkpoint?")]
    public float ReachGoalCheckpointRewardBase = 3.0f;
    [Tooltip("Should typically be a small value, but we reward the agent for moving in the right direction.")]
    public float TowardsCheckpointReward = 0.003f;
    [Tooltip("Typically if the agent moves faster, we want to reward it for finishing the track quickly.")]
    public float SpeedReward = 0.02f;
    [Tooltip("Reward the agent when it keeps accelerating")]
    public float AccelerationReward = 0.002f;
    #endregion

    public Dictionary<Rigidbody, KartAgent> AgentBodies = new Dictionary<Rigidbody, KartAgent>();
    public HashSet<KartAgent> inactiveAgents = new HashSet<KartAgent>();
    [HideInInspector] public int goalSection;

    public int episodeSteps;
    public int maxEpisodeSteps;

    // Start is called before the first frame update
    void Start()
    {
        for (int i = 0; i < Agents.Length; i++)
        {
            inactiveAgents.Add(Agents[i]);
            AgentBodies[Agents[i].GetComponent<Rigidbody>()] = Agents[i];
        }
        goalSection = laps * Sections.Length + 1;
        ResetGame();
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
        int maxGoalTiming = 0;
        for (int i = 0; i < Agents.Length; i++)
        {
            if (Agents[i].m_timeSteps == 0)
            {
                maxGoalTiming = maxEpisodeSteps;
                Agents[i].m_timeSteps = maxEpisodeSteps;
            } else
            {
                maxGoalTiming = Math.Max(Agents[i].m_timeSteps, maxGoalTiming);
            }
        }
        for (int i = 0; i < Agents.Length; i++)
        {
            if (Agents[i].m_timeSteps != 0)
            {
                Agents[i].AddReward(ReachGoalCheckpointRewardMultplier * (maxGoalTiming - Agents[i].m_timeSteps) * 1.0f / maxEpisodeSteps + ReachGoalCheckpointRewardBase);
            }
        }
    }


    void FixedUpdate()
    {
        if (inactiveAgents.Count == Agents.Length)
        {

            // print("from here 1");
            AddGoalTimingRewards();
            for (int i = 0; i < Agents.Length; i++)
            {
                Agents[i].EndEpisode();
            }
            ResetGame();
        }
        else
        {
            episodeSteps += 1;
            if (episodeSteps >= maxEpisodeSteps)
            {
                AddGoalTimingRewards();
                for (int i = 0; i < Agents.Length; i++)
                {
                    Agents[i].EpisodeInterrupted();
                }
                //print("from here 2");
                ResetGame();
            }
        }

    }

    public void ResolveEvent(Event triggeringEvent, KartAgent triggeringAgent, HashSet<KartAgent> otherInvolvedAgents)
    {
        //print("Resolving event");
        //print(triggeringEvent);
        switch (triggeringEvent)
        {
            case Event.HitSomething:
                triggeringAgent.ApplyHitPenalty();
                //if (!inactiveAgents.Contains(triggeringAgent))
                //{
                //    triggeringAgent.Deactivate();
                //    inactiveAgents.Add(triggeringAgent);
                //}
                foreach (KartAgent agent in otherInvolvedAgents)
                {
                    agent.ApplyHitPenalty();
                    //if (!inactiveAgents.Contains(triggeringAgent))
                    //{
                    //    agent.Deactivate();
                   //     inactiveAgents.Add(agent);
                   // }
                }
                otherInvolvedAgents.Clear();
                break;
            case Event.ReachNonGoalSection:
                triggeringAgent.ApplySectionReward();
                break;
            case Event.ReachGoalSection:
                triggeringAgent.m_timeSteps = episodeSteps;
                triggeringAgent.Deactivate();
                inactiveAgents.Add(triggeringAgent);
                break;
        }
    }

    void ResetGame()
    {
        //print("resetting game");
        episodeSteps = 0;
        inactiveAgents.Clear();
        // For each agent
        var furthestForwardSection = -1;
        HashSet< Collider > addedColliders = new HashSet<Collider>();
        bool headToHead = UnityEngine.Random.Range(0, 7) != 1;
        var initialSection = -1;
        for (int i = 0; i < Agents.Length; i++)
        {
            if (!headToHead)
            {
                // Randomly Set Iniital Track Position
                while (true)
                {
                    Agents[i].m_SectionIndex = UnityEngine.Random.Range(0, Sections.Length - 1);
                    Agents[i].InitCheckpointIndex = Agents[i].m_SectionIndex;
                    var collider = Sections[Agents[i].m_SectionIndex % Sections.Length].getBoxColliderForLane(UnityEngine.Random.Range(1, 4));
                    if (!addedColliders.Contains(collider))
                    {
                        furthestForwardSection = Math.Max(Agents[i].m_SectionIndex, furthestForwardSection);
                        Agents[i].transform.localRotation = collider.transform.rotation;
                        Agents[i].transform.position = collider.transform.position;
                        Agents[i].m_UpcomingLanes.Clear();
                        addedColliders.Add(collider);
                        break;
                    }
                }
            }
            else
            {
                if (addedColliders.Count == 0)
                {
                    Agents[i].m_SectionIndex = UnityEngine.Random.Range(0, Sections.Length - 1);
                    initialSection = Agents[i].m_SectionIndex;
                    Agents[i].InitCheckpointIndex = Agents[i].m_SectionIndex;
                    furthestForwardSection = Math.Max(Agents[i].m_SectionIndex, furthestForwardSection);
                    var collider = Sections[Agents[i].m_SectionIndex % Sections.Length].getBoxColliderForLane(UnityEngine.Random.Range(1, 4));
                    Agents[i].transform.localRotation = collider.transform.rotation;
                    Agents[i].transform.position = collider.transform.position;
                    Agents[i].m_UpcomingLanes.Clear();
                    addedColliders.Add(collider);
                }
                else
                {
                    while (true)
                    {
                        Agents[i].m_SectionIndex = UnityEngine.Random.Range(Math.Min(initialSection - 1, 0), initialSection + 1);
                        Agents[i].InitCheckpointIndex = Agents[i].m_SectionIndex;
                        var collider = Sections[Agents[i].m_SectionIndex % Sections.Length].getBoxColliderForLane(UnityEngine.Random.Range(1, 4));
                        if (!addedColliders.Contains(collider))
                        {
                            furthestForwardSection = Math.Max(Agents[i].m_SectionIndex, furthestForwardSection);
                            Agents[i].transform.localRotation = collider.transform.rotation;
                            Agents[i].transform.position = collider.transform.position;
                            Agents[i].m_UpcomingLanes.Clear();
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
        // Then, for each agent, Generate a Sequence of Lanes to follow
            for(int tp = Agents[i].m_SectionIndex+1; tp < maxSection; tp++)
            {
                Agents[i].m_UpcomingLanes[tp % Sections.Length] = UnityEngine.Random.Range(1, 4);

            }
        }

        for (int i = 0; i < Agents.Length; i++)
        {
            Agents[i].Activate();
            Agents[i].OnEpisodeBegin();
        }
    }
}

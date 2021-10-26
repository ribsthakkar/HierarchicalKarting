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

    public Dictionary<Rigidbody, KartAgent> AgentBodies = new Dictionary<Rigidbody, KartAgent>();
    public HashSet<KartAgent> inactiveAgents = new HashSet<KartAgent>();

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
        ResetGame();
    }

    void AddGoalTimingRewards()
    {
        if(Agents.Length == 1)
        {
            if (Agents[0].m_timeSteps == 0)
            {
                Agents[0].AddReward(-maxEpisodeSteps);
            }
            else
            {
                Agents[0].AddReward(-Agents[0].m_timeSteps);
            }
            return;
        }
        int maxGoalTiming = 0;
        for (int i = 0; i < Agents.Length; i++)
        {
            if (Agents[i].m_timeSteps == 0)
            {
                maxGoalTiming = maxEpisodeSteps;
                break;
            } else
            {
                maxGoalTiming = Math.Max(Agents[i].m_timeSteps, maxGoalTiming);
            }
        }
        for (int i = 0; i < Agents.Length; i++)
        {
            Agents[i].AddReward(maxGoalTiming - Agents[i].m_timeSteps);
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
        print("Resolving event");
        print(triggeringEvent);
        switch (triggeringEvent)
        {
            case Event.HitSomething:
                triggeringAgent.ApplyHitPenalty();
                if (!inactiveAgents.Contains(triggeringAgent))
                {
                    triggeringAgent.Deactivate();
                    inactiveAgents.Add(triggeringAgent);
                }
                foreach (KartAgent agent in otherInvolvedAgents)
                {
                    agent.ApplyHitPenalty();
                    if (!inactiveAgents.Contains(triggeringAgent))
                    {
                        agent.Deactivate();
                        inactiveAgents.Add(agent);
                    }
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
        for (int i = 0; i < Agents.Length; i++)
        {
            // Randomly Set Iniital Track Position
            Agents[i].m_SectionIndex = UnityEngine.Random.Range(0, Sections.Length - 1);
            furthestForwardSection = Math.Max(Agents[i].m_SectionIndex, furthestForwardSection);
            var collider = Sections[Agents[i].m_SectionIndex % Sections.Length].getBoxColliderForLane(UnityEngine.Random.Range(1, 4));
            Agents[i].transform.localRotation = collider.transform.rotation;
            Agents[i].transform.position = collider.transform.position;
        }

        // Use the furthest forward agent to determine the final goal track section by adding a fixed amount of sections to it
        var maxSection = furthestForwardSection + 8;
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

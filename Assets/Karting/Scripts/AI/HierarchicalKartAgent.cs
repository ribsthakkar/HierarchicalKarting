﻿using KartGame.KartSystems;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using UnityEngine;
using Random = UnityEngine.Random;
using System.Collections.Generic;
using System;
using Unity.MLAgents.Actuators;
using System.Linq;
using Unity.MLAgents.Policies;
using System.Threading;
using CenterSpace.NMath.Core;
using KartGame.AI.MPC;
using KartGame.AI.LQR;
using KartGame.AI.MCTS;
using MathNet.Numerics.LinearAlgebra;

namespace KartGame.AI
{
    [System.Serializable]
    public enum LowLevelMode
    {
        RL,
        MPC,
        LQR
    }

    [System.Serializable]
    public enum HighLevelMode
    {
        MCTS,
        Fixed
    }

    [System.Serializable]
    public struct DiscreteGameParams
    {
        [Header("High-Level MCTS Planner Parameters")]
        [Tooltip("What is the size of the discrete velocity buckets")]
        public int velocityBucketSize;
        [Tooltip("What is the precision of time (1, 10, 100, etc. to use)?")]
        public int timePrecision;
        [Tooltip("What is the time gap to represent a collision in the discrete game?")]
        public float collisionWindow;
        [Tooltip("When do we consider agents to be in game-theoritetic mode?")]
        public int sectionWindow;
        [Tooltip("How far to search in the tree?")]
        public int treeSearchDepth;
    }

    /// <summary>
    /// The KartAgent will drive the inputs for the KartController.
    /// </summary>
    ///
    public class HierarchicalKartAgent : KartAgent, IInput
    {

        #region MCTS Params
        public DiscreteGameParams gameParams;
        #endregion
        [Tooltip("Are we using the RL brain for low level control or MPC?")]
        public LowLevelMode LowMode = LowLevelMode.MPC;
        [Tooltip("Which High-Level control mode are we using?")]
        public HighLevelMode HighMode = HighLevelMode.MCTS;

        [HideInInspector] KartMCTSNode currentRoot = null;
        [HideInInspector] int CyclesRootProcessed = 0;
        [HideInInspector] int abortAttempts = 0;
        [HideInInspector] bool HLReadyToUse = true;
        [HideInInspector] List<DiscreteGameState> bestStates = new List<DiscreteGameState>();
        [HideInInspector] Thread t = null;
        [HideInInspector] List<Vector2> finerWaypoints;
        [HideInInspector] int currentFinerIndex;
        [HideInInspector] List<DoubleVector> resultVector;
        [HideInInspector] int mpcSteps = 3;
        [HideInInspector] Dictionary<String, Dictionary<int, int>> opponentUpcomingLanes = new Dictionary<string, Dictionary<int, int>>();
        [HideInInspector] Dictionary<String, Dictionary<int, float>> opponentUpcomingVelocities = new Dictionary<string, Dictionary<int, float>>();


        /**
         * Generate initial plan depending on state set in inspector or if in training mode 
        **/
        public override void initialPlan()
        {

            currentRoot = null;
            CyclesRootProcessed = 0;
            if (Mode == AgentMode.Inferencing)
            {
                if (HighMode == HighLevelMode.MCTS)
                {
                    planWithMCTS();
                }
                else
                {
                    planFixed();
                }
            }
            else
            {
                planRandomly();
            }
        }

        /**
         * Generate random plan for the lanes and highlight them. For the fixed agent, make  sure not to generate a velocity
         * Generate the target lanes with a bias for the pre-determined optimal choice of lane
        **/
        public override void planRandomly()
        {
            for (int i = m_SectionIndex + 1; i < Math.Min(m_SectionIndex + gameParams.treeSearchDepth, 1000) + 1; i++)
            {
                if (!m_UpcomingLanes.ContainsKey(i % m_envController.Sections.Length))
                {
                    int index = Mathf.RoundToInt(Mathf.Abs(KartMCTS.NextGaussian(0, 1f, -(float)4 + 1f, (float)4 - 1f)));
                    int optimalLaneSign = m_envController.Sections[(i - 1) % m_envController.Sections.Length].getOptimalLaneSign();
                    int lane = Enumerable.Range(1, 4).OrderBy((l) => optimalLaneSign * l).ToList()[index];
                    m_UpcomingLanes[i % m_envController.Sections.Length] = lane;
                    if (HighMode == HighLevelMode.Fixed)
                    {
                        m_UpcomingVelocities[i % m_envController.Sections.Length] = m_Kart.GetMaxSpeed();
                    }
                    else
                    {
                        m_UpcomingVelocities[i % m_envController.Sections.Length] = m_Kart.GetMaxSpeed() - Mathf.Abs(KartMCTS.NextGaussian(0, 1.5f, -8f, 8f));
                    }
                    if (name.Equals(m_envController.Agents[0].name) && m_envController.highlightWaypoints)
                    {
                        m_envController.Sections[i % m_envController.Sections.Length].getBoxColliderForLane(lane).GetComponent<Renderer>().material.color = Color.green;
                        //print(kartState.name + " Will reach Section " + kartState.section + " at time " + kartState.timeAtSection + " in lane " + kartState.lane + " with velocity " + kartState.getAverageVelocity());
                    }
                }
            }
        }

        /**
         * Generate Plan for Fixed hierarchical agents on predetermined optimal lane choices
        **/
        public void planFixed()
        {
            for (int i = m_SectionIndex + 1; i < Math.Min(m_SectionIndex + gameParams.treeSearchDepth, 1000) + 1; i++)
            {
                if (!m_UpcomingLanes.ContainsKey(i % m_envController.Sections.Length))
                {
                    int lane = m_envController.Sections[(i - 1) % m_envController.Sections.Length].getOptimalNextLane();
                    m_UpcomingLanes[i % m_envController.Sections.Length] = lane;
                    m_UpcomingVelocities[i % m_envController.Sections.Length] = m_Kart.GetMaxSpeed();
                    if (name.Equals(m_envController.Agents[0].name) && m_envController.highlightWaypoints)
                    {
                        m_envController.Sections[i % m_envController.Sections.Length].getBoxColliderForLane(lane).GetComponent<Renderer>().material.color = Color.green;
                        //print(kartState.name + " Will reach Section " + kartState.section + " at time " + kartState.timeAtSection + " in lane " + kartState.lane + " with velocity " + kartState.getAverageVelocity());
                    }
                }
            }
        }

        /**
         * Generate plan for the MCTS agents
         * FIrst construct the discrete game and start a background thread to run the MCTS algorithm.
        **/
        public void planWithMCTS()
        {
            // If no existing root of tree or thread that is alive to computing plan
            if (currentRoot == null && t == null)
            {
                // Find nearby agents and construct initial game states
                List<KartAgent> nearbyAgents = new List<KartAgent>();
                List<DiscreteKartState> nearbyAgentStates = new List<DiscreteKartState>();
                int initialSection = m_SectionIndex;
                KartAgent furthestForwardAgent = this;
                foreach (KartAgent agent in m_envController.Agents)
                {
                    if (Math.Abs(agent.m_SectionIndex - m_SectionIndex) < gameParams.sectionWindow)
                    {
                        nearbyAgents.Add(agent);
                        initialSection = Math.Max(initialSection, agent.m_SectionIndex);
                        if (initialSection == agent.m_SectionIndex)
                        {
                            furthestForwardAgent = agent;
                        }
                    }
                }
                int finalSection = initialSection + gameParams.treeSearchDepth;
                int count = 0;
                foreach (KartAgent agent in nearbyAgents)
                {
                    int min_velocity = 0;
                    int max_velocity = (int)agent.m_Kart.GetMaxSpeed();
                    for (int i = 0; i < (int)agent.m_Kart.GetMaxSpeed(); i += gameParams.velocityBucketSize)
                    {
                        if (agent.m_Kart.Rigidbody.velocity.magnitude >= i && agent.m_Kart.Rigidbody.velocity.magnitude >= i)
                        {
                            min_velocity = i;
                            max_velocity = Math.Min(i + gameParams.velocityBucketSize, (int)agent.m_Kart.GetMaxSpeed());
                            break;
                        }

                    }
                    int timeAtSection = 0;
                    if (agent.m_SectionIndex != initialSection)
                    {
                        timeAtSection = (int)((agent.sectionTimes[agent.m_SectionIndex] - furthestForwardAgent.sectionTimes[agent.m_SectionIndex]) * Time.fixedDeltaTime * gameParams.timePrecision);
                    }
                    if (m_Lane <= 0)
                    {
                        Debug.Log("ISSUE with current lane LESS THAN 0 for " + agent.name);
                    }
                    nearbyAgentStates.Add(new DiscreteKartState
                    {
                        player = count,
                        team = m_envController.getTeamID(agent),
                        section = initialSection,
                        timeAtSection = timeAtSection,
                        min_velocity = min_velocity,
                        max_velocity = max_velocity,
                        lane = agent.m_Lane,
                        tireAge = (int)((agent.m_Kart.baseStats.MaxSteer - agent.m_Kart.m_FinalStats.Steer) / (agent.m_Kart.baseStats.MaxSteer - agent.m_Kart.baseStats.MinSteer) * 10000),
                        laneChanges = agent.m_LaneChanges,
                        infeasible = false,
                        name = agent.name
                    });
                }

                DiscreteGameState initialGameState = new DiscreteGameState
                {
                    envController = m_envController,
                    kartStates = nearbyAgentStates,
                    kartAgents = nearbyAgents,
                    initialSection = initialSection,
                    finalSection = finalSection,
                    lastCompletedSection = initialSection,
                    gameParams = gameParams

                };
                t = new Thread(() =>
                {
                    try
                    {
                        currentRoot = KartMCTS.constructSearchTree(initialGameState, T: 0.9);
                        // print("Here cr calculated");
                        bestStates = KartMCTS.getBestStatesSequence(currentRoot);
                        CyclesRootProcessed = 1;
                    }
                    catch (ThreadAbortException e) {
                        print("Here: " + e.ToString());
                        return;
                    }

                });
                t.IsBackground = true;
                t.Start();

            }
            else if (t == null && CyclesRootProcessed < 3) // If we have an existing root that hasn't been reused 3 times and no existing thread that is currently planning
            {
                t = new Thread(() =>
                {
                    try
                    {
                        currentRoot = KartMCTS.constructSearchTree(currentRoot, T: 0.9);
                        bestStates = KartMCTS.getBestStatesSequence(currentRoot);
                        CyclesRootProcessed += 1;
                    }
                    catch (ThreadAbortException e) {
                        print("Here: " + e.ToString());
                        return;
                    }

                });
                t.IsBackground = true;
                t.Start();
            }
        }

        public override void Start()
        {
            base.Start();
            finerWaypoints = m_envController.Sections.SelectMany(s => s.finePoints).ToList();
            updateFinerIdxGuess();
            NMathConfiguration.LicenseKey = "2DB6FFC3C9D7B60";
        }

        /**
         * Create a function to be used in a coroutine to kill a thread if it missed its deadline 
        **/
        System.Collections.IEnumerator ResetHLEnvironment()
        {
            while (t.IsAlive){
                t.Abort();
                abortAttempts++;
                print(name + " Thread did not finish in 5 ms fixed Update " + m_SectionIndex + " abort attempts missed " + abortAttempts);
                yield return new WaitForSeconds(2f);
            } 
            t.Join();
            t = null;
            currentRoot = null;
            CyclesRootProcessed = 0;
            HLReadyToUse = true;
            yield return null;
        }

        protected override void FixedUpdate()
        {
            base.FixedUpdate();
            updateFinerIdxGuess();
            if (m_envController.episodeSteps % 1 == 0) // 50Hz Run LQR
            {
                if (LowMode == LowLevelMode.LQR && !m_envController.inactiveAgents.Contains(this))
                    SolveLQR(numSteps: mpcSteps);
                
            }
            if (m_envController.episodeSteps % 10 == 0 && !m_envController.inactiveAgents.Contains(this)) // 10 Hz Run MPC
            {
                if (LowMode == LowLevelMode.MPC)
                    SolveMPC(numSteps: mpcSteps*1);
            }
            if (m_envController.episodeSteps % 100 == 0 && m_envController.episodeSteps < m_envController.maxEpisodeSteps && m_envController.episodeSteps > 0 && !m_envController.inactiveAgents.Contains(this)) // 0.5 Hz 
            {
                if (Mode == AgentMode.Inferencing)
                {
                    if (HighMode == HighLevelMode.MCTS)
                    {
                        if (t != null)
                        {
                            if (HLReadyToUse && !t.Join(5))
                            {
                                HLReadyToUse = false;
                                abortAttempts = 0;
                                StartCoroutine(ResetHLEnvironment());
                            } else if (HLReadyToUse)
                            {
                                t = null;
                            }
                        }
                        if (HLReadyToUse)
                            planWithMCTS();
                    }
                    else
                    {
                        planFixed();
                    }
                }
                else
                {
                    planRandomly();
                }

            }
            // print(this.name + " " + initialGameState.kartAgents.Count);
            // print(bestStates.Count);
            // Update Target Lanes and Velocities based on the high-level mode
            foreach (DiscreteGameState gameState in bestStates)
            {
                foreach (DiscreteKartState kartState in gameState.kartStates)
                {

                    if (kartState.name.Equals(this.name) && kartState.section > m_SectionIndex + 1)
                    {
                        if (m_UpcomingLanes.ContainsKey(kartState.section % m_envController.Sections.Length))
                        {
                            if (name.Equals(m_envController.Agents[0].name))
                                m_envController.Sections[kartState.section % m_envController.Sections.Length].resetColors();

                        }
                        m_UpcomingLanes[kartState.section % m_envController.Sections.Length] = kartState.lane;
                        m_UpcomingVelocities[kartState.section % m_envController.Sections.Length] = kartState.max_velocity;

                        if (name.Equals(m_envController.Agents[0].name) && m_envController.highlightWaypoints)
                        {
                            m_envController.Sections[kartState.section % m_envController.Sections.Length].getBoxColliderForLane(kartState.lane).GetComponent<Renderer>().material.color = Color.green;
                        }
                    }
                    else if (!kartState.name.Equals(this.name) && opponentUpcomingLanes.ContainsKey(kartState.name))
                    {

                        opponentUpcomingLanes[kartState.name][kartState.section % m_envController.Sections.Length] = kartState.lane;
                        opponentUpcomingVelocities[kartState.name][kartState.section % m_envController.Sections.Length] = kartState.max_velocity;
                    }
                }
            }
            if (Mode == AgentMode.Inferencing)
            {
                var lines1 = m_UpcomingLanes.Select(kvp => kvp.Key + ": " + kvp.Value.ToString());
                //print("Upcoming Lanes " + this.name + " " + string.Join(",", lines1));
                var lines = m_UpcomingVelocities.Select(kvp => kvp.Key + ": " + kvp.Value.ToString());
                //print("Upcoming Velocities " + this.name + " " + string.Join(",", lines));
                //print(m_SectionIndex);
            }
        }

        protected override void Awake()
        {
            base.Awake();
            var behaviorParameters = GetComponent<BehaviorParameters>();
            var brainParameters = behaviorParameters.BrainParameters;
            /**
             * Sensors.Lenght -> RayCasts
             * SectionHorizon*5 -> Upcoming Lanes and Velocities and lane types
             * 8 -> Current Player's state
             * 12 -> Other player states
             **/
            brainParameters.VectorObservationSize = Sensors.Length + (m_envController.sectionHorizon * 5) + (name.Equals("MCTS-RL") || name.Equals("Fixed-RL") ?7:8) + (12 * (otherAgents.Length + teamAgents.Length));
            prepareForReuse();
        }

        public override void prepareForReuse()
        {
            base.prepareForReuse();
            foreach (Agent other in otherAgents)
            {
                opponentUpcomingLanes[other.name] = new Dictionary<int, int>();
                opponentUpcomingVelocities[other.name] = new Dictionary<int, float>();
            }
            foreach (Agent other in teamAgents)
            {
                opponentUpcomingLanes[other.name] = new Dictionary<int, int>();
                opponentUpcomingVelocities[other.name] = new Dictionary<int, float>();
            }
            if (t != null)
            {
                t.Abort();
                while (t.IsAlive);
                t = null;

            }
            currentRoot = null;
            CyclesRootProcessed = 0;
            bestStates = new List<DiscreteGameState>();
            HLReadyToUse = true;
        }

        /**
         * Calculate the divider for lane difference
        **/
        protected override void setLaneDifferenceDivider(int sectionIndex, int lane)
        {
            LaneDifferenceRewardDivider = 1.0f;
            if (lane != -1)
            {
                int target_lane = m_UpcomingLanes[sectionIndex % m_envController.Sections.Length];
                if ((m_envController.Sections[sectionIndex % m_envController.Sections.Length].getBoxColliderForLane(target_lane).transform.position - m_Kart.Rigidbody.position).magnitude > 1.3)
                {
                    LaneDifferenceRewardDivider = Mathf.Pow(1.3f, 1.0f * ((m_envController.Sections[sectionIndex % m_envController.Sections.Length].getBoxColliderForLane(target_lane).transform.position - m_Kart.Rigidbody.position).magnitude));
                }
            }
        }

        /**
         * Provide Heuristic function for ML agents to test that the environment is setup correctly using human input 
        **/
        protected override void setVelocityDifferenceDivider(int sectionIndex, float velocity)
        {
            VelocityDifferenceRewardDivider = 1.0f;
            if (HighMode == HighLevelMode.Fixed)
                return;
            if (Mathf.Abs(velocity - m_UpcomingVelocities[sectionIndex % m_envController.Sections.Length]) > gameParams.velocityBucketSize / 2.0f)
                VelocityDifferenceRewardDivider = Mathf.Pow(1.1f, 1.0f * (Mathf.Abs(velocity - m_UpcomingVelocities[sectionIndex % m_envController.Sections.Length])));
        }

        /**
         * Provide observations for the RL-based agents
        **/
        public override void CollectObservations(VectorSensor sensor)
        {
            //print("collecting observations");
            // Add observation for agent state (Speed, acceleration, lane, recent lane changes, tire age, section type)
            sensor.AddObservation(m_Kart.LocalSpeed());
            sensor.AddObservation(m_Acceleration);
            sensor.AddObservation(m_Lane);
            sensor.AddObservation(m_LaneChanges * 1f / m_envController.MaxLaneChanges);
            if (!name.Equals("MCTS-RL") && !name.Equals("Fixed-RL")) sensor.AddObservation(is_active);
            sensor.AddObservation(m_SectionIndex * 1f / m_envController.goalSection);
            sensor.AddObservation(m_envController.sectionIsStraight(m_SectionIndex));
            sensor.AddObservation(m_Kart.TireWearProportion());


            // Add observations for team agent states (Speed, acceleration, lane, recent lane chagnes, section type, tire age, distance, relative position)
            foreach (KartAgent agent in teamAgents)
            {
                sensor.AddObservation(agent.m_Kart.LocalSpeed());
                sensor.AddObservation(agent.m_Acceleration);
                sensor.AddObservation(agent.m_Lane);
                sensor.AddObservation(agent.m_LaneChanges * 1f / m_envController.MaxLaneChanges);
                sensor.AddObservation(agent.is_active);
                sensor.AddObservation(m_envController.Sections[agent.m_SectionIndex % m_envController.Sections.Length].isStraight());
                sensor.AddObservation(agent.m_Kart.TireWearProportion());
                sensor.AddObservation(agent.m_SectionIndex * 1f / m_envController.goalSection);
                sensor.AddObservation((agent.m_Kart.transform.position - m_Kart.transform.position).magnitude);
                sensor.AddObservation(m_Kart.transform.InverseTransformPoint(agent.m_Kart.transform.position));
            }

            // Add observation for opponent agent states (Speed, acceleration, lane, recent lane chagnes, section type, tire age, distance, relative position)
            foreach (KartAgent agent in otherAgents)
            {
                sensor.AddObservation(agent.m_Kart.LocalSpeed());
                sensor.AddObservation(agent.m_Acceleration);
                sensor.AddObservation(agent.m_Lane);
                sensor.AddObservation(agent.m_LaneChanges * 1f / m_envController.MaxLaneChanges);
                sensor.AddObservation(agent.is_active);
                sensor.AddObservation(m_envController.Sections[agent.m_SectionIndex % m_envController.Sections.Length].isStraight());
                sensor.AddObservation(agent.m_Kart.TireWearProportion());
                sensor.AddObservation(agent.m_SectionIndex * 1f / m_envController.goalSection);
                sensor.AddObservation((agent.m_Kart.transform.position - m_Kart.transform.position).magnitude);
                sensor.AddObservation(m_Kart.transform.InverseTransformPoint(agent.m_Kart.transform.position));
            }

            // Add an observation for direction of the agent to the next checkpoint and lane and the velocity at that lane.
            for (int i = m_SectionIndex + 1; i < m_SectionIndex + 1 + m_envController.sectionHorizon; i++)
            {
                var next = (i) % m_envController.Sections.Length;
                var nextSection = m_envController.Sections[next];
                if (nextSection == null)
                    return;
                if (m_UpcomingLanes.ContainsKey(next))
                {
                    BoxCollider targetLaneInSection = nextSection.getBoxColliderForLane(m_UpcomingLanes[next]);
                    sensor.AddObservation(m_Kart.transform.InverseTransformPoint(targetLaneInSection.transform.position));
                    sensor.AddObservation(m_UpcomingVelocities[next] / m_Kart.GetMaxSpeed());
                    sensor.AddObservation(m_envController.sectionIsStraight(next));
                    if (ShowRaycasts)
                        Debug.DrawLine(AgentSensorTransform.position, targetLaneInSection.transform.position, Color.magenta);
                }
                else
                {
                    Collider target = nextSection.Trigger;
                    sensor.AddObservation(m_Kart.transform.InverseTransformPoint(target.transform.position));
                    sensor.AddObservation(1.0f);
                    sensor.AddObservation(m_envController.sectionIsStraight(next));
                }
            }
            for (var i = 0; i < Sensors.Length; i++)
            {
                var current = Sensors[i];
                var xform = current.Transform;
                var hitTrack = Physics.Raycast(AgentSensorTransform.position, xform.forward, out var hitTrackInfo,
                    current.RayDistance, TrackMask, QueryTriggerInteraction.Ignore);

                var hitAgent = Physics.Raycast(AgentSensorTransform.position, xform.forward, out var hitAgentInfo,
                    current.RayDistance, AgentMask, QueryTriggerInteraction.Ignore);

                if (ShowRaycasts)
                {
                    //Debug.DrawRay(AgentSensorTransform.position, xform.forward * current.HitValidationDistance, Color.red);
                    if (hitTrack && hitTrackInfo.distance < current.RayDistance && (!hitAgent || hitTrackInfo.distance < hitAgentInfo.distance))
                    {
                        Debug.DrawLine(xform.position, hitTrackInfo.point, Color.blue);
                    }
                    else if (hitAgent && hitAgentInfo.distance < current.RayDistance)
                    {
                        Debug.DrawLine(xform.position, hitAgentInfo.point, Color.blue);
                    }
                    else
                    {
                        Debug.DrawRay(AgentSensorTransform.position, xform.forward * current.RayDistance, Color.green);
                    }
                }

                if (hitTrack && (!hitAgent || hitTrackInfo.distance < hitAgentInfo.distance))
                {
                    if (hitTrackInfo.distance < current.WallHitValidationDistance)
                    {
                        //print("hitting wall " + hitTrackInfo.distance);
                        m_envController.ResolveEvent(Event.HitWall, this, null);
                    }
                    sensor.AddObservation(hitTrackInfo.distance);
                }
                else if (hitAgent)
                {
                    if (hitAgentInfo.distance < current.AgentHitValidationDistance)
                    {
                        m_LastAccumulatedReward += m_envController.OpponentHitPenalty;
                        hitAgents.Add(m_envController.AgentBodies[hitAgentInfo.collider.attachedRigidbody]);
                        m_envController.ResolveEvent(Event.HitOpponent, this, hitAgents);
                    }
                    sensor.AddObservation(hitAgentInfo.distance);
                }
                else
                {
                    sensor.AddObservation(current.RayDistance);
                }
            }
        }

        /**
         * Executed when agent passes through the major checkpoints. 
         * Updates the section index (aka Checkpoint) and computes illegal lane change information.
         * Also detects if driving in reverse.
        **/
        void OnTriggerEnter(Collider other)
        {
            if (!is_active) return;
            var maskedValue = 1 << other.gameObject.layer;
            var triggered = maskedValue & CheckpointMask;

            FindSectionIndex(other, out var index, out var lane);
            LaneDifferenceRewardDivider = 1.0f;
            VelocityDifferenceRewardDivider = 1.0f;
            // Ensure that the agent touched the checkpoint and the new index is greater than the m_SectionIndex.
            if ((triggered > 0 && index != -1) && ((index > m_SectionIndex) || (index % m_envController.Sections.Length == 0 && m_SectionIndex % m_envController.Sections.Length == m_envController.Sections.Length - 1)))
            {
                if (m_UpcomingLanes.ContainsKey(index % m_envController.Sections.Length))
                {
                    setLaneDifferenceDivider(index, lane);
                    setVelocityDifferenceDivider(index, m_Kart.Rigidbody.velocity.magnitude);
                    UpdateLaneDifferenceCalculation(index, lane);
                    UpdateVelocityDifferenceCalculation(index, m_Kart.Rigidbody.velocity.magnitude);
                    m_UpcomingLanes.Remove(index % m_envController.Sections.Length);
                    m_UpcomingVelocities.Remove(index % m_envController.Sections.Length);
                }
                if (name.Equals(m_envController.Agents[0].name))
                    m_envController.Sections[index % m_envController.Sections.Length].resetColors();
                if (m_LaneChanges + Math.Abs(m_Lane - lane) > m_envController.MaxLaneChanges && m_envController.sectionIsStraight(m_SectionIndex))
                {
                    AddReward(m_envController.SwervingPenalty);
                    m_IllegalLaneChanges += 1;
                }
                if (m_envController.sectionIsStraight(m_SectionIndex) != m_envController.sectionIsStraight(index))
                {
                    m_LaneChanges = 0;
                }
                else if (m_Lane != lane)
                {
                    m_LaneChanges += Math.Abs(m_Lane - lane);
                }
                m_SectionIndex = index;
                m_Lane = lane;
                sectionTimes[m_SectionIndex] = m_envController.episodeSteps;
                if (m_SectionIndex == m_envController.goalSection)
                {
                    m_envController.ResolveEvent(Event.ReachGoalSection, this, null);
                }
                else
                {
                    m_envController.ResolveEvent(Event.ReachNonGoalSection, this, null);
                }
                currentRoot = null;
                CyclesRootProcessed = 0;
            }
            else if ((triggered > 0 && index != -1) && ((index < m_SectionIndex) || (m_SectionIndex % m_envController.Sections.Length == 0 && index % m_envController.Sections.Length == m_envController.Sections.Length - 1)))
            {
                // print("going backwards");
                AddReward(m_envController.ReversePenalty * (m_SectionIndex - index + 1));
                m_SectionIndex = index;
                currentRoot = null;
                CyclesRootProcessed = 0;
            }
            else if ((triggered > 0 && index == -1))
            {
                m_envController.ResolveEvent(Event.DroveReverseLimit, this, null);
            }
        }

        /**
         * Find out the "finer" index of the checkpoint. This calculates the checkpoint that further discretized between the main checkpoints on the track.
         * It is used for the MPC solver.
        **/
        void updateFinerIdxGuess()
        {
            currentFinerIndex = m_SectionIndex * 10;
            double minDistance = 1000;
            for (int initialFinerIdxGuess = m_SectionIndex * 10; initialFinerIdxGuess < m_SectionIndex * 10 + 10; initialFinerIdxGuess++)
            {
                Vector2 pt = new Vector2(m_Kart.Rigidbody.transform.position.x, m_Kart.Rigidbody.transform.position.z);
                if ((finerWaypoints[initialFinerIdxGuess % finerWaypoints.Count] - pt).sqrMagnitude < minDistance)
                {
                    minDistance = (finerWaypoints[initialFinerIdxGuess % finerWaypoints.Count] - pt).sqrMagnitude;
                    currentFinerIndex = initialFinerIdxGuess;
                }
            }
        }

        /**
         * Calculate control inputs by solving an LQ Nash Game
        **/
        public InputData SolveLQR(int numSteps = 200)
        {
            List<KartAgent> allPlayers = new[] { this }.Concat(teamAgents).Concat(otherAgents).ToList();
            List<KartLQRDynamics> dynamics = new List<KartLQRDynamics>();
            List<Vector<double>> initialStates = new List<Vector<double>>();
            List<KartLQRCosts> costs = new List<KartLQRCosts>();
            double dt = Time.fixedDeltaTime;
            for (int i = 0; i < allPlayers.Count; i++)
            {
                KartAgent k = allPlayers[i];
                // Create Initial Vector
                var initial = CreateVector.Dense<double>(4);
                initial[KartMPC.xIndex] = k.m_Kart.transform.position.x;
                initial[KartMPC.zIndex] = k.m_Kart.transform.position.z;
                initial[KartMPC.vIndex] = k.m_Kart.Rigidbody.velocity.magnitude;
                var heading = Mathf.Atan2(k.m_Kart.transform.forward.z, k.m_Kart.transform.forward.x);
                if (heading < 0) heading += 2 * Mathf.PI;
                initial[KartMPC.hIndex] = heading;

                // Create Dynamics
                dynamics.Add(new LinearizedBicycle(dt, initial));

                // print(initial.ToString());
                initialStates.Add(initial);

                // Add Cost Matrix Details
                var targetState = CreateVector.Dense<double>(4);
                int s = k.m_SectionIndex + 1;
                int idx = s % m_envController.Sections.Length;

                // Set Target Lane and Velocity
                BoxCollider lane;
                double vel;
                if (k == this)
                {
                    if (m_UpcomingLanes.ContainsKey(idx))
                    {
                        lane = m_envController.Sections[idx].getBoxColliderForLane(m_UpcomingLanes[idx]);
                        vel = Math.Min(m_Kart.GetMaxSpeed(), m_UpcomingVelocities[idx] + (HighMode == HighLevelMode.MCTS ? gameParams.velocityBucketSize*2 : 0));
                    }
                    else
                    {
                        lane = m_envController.Sections[idx].Trigger;
                        vel = m_Kart.GetMaxSpeed();
                    }
                }
                else
                {
                    if (opponentUpcomingLanes[k.name].ContainsKey(idx))
                    {
                        lane = m_envController.Sections[idx].getBoxColliderForLane(opponentUpcomingLanes[k.name][idx]);
                        vel = Math.Min(k.m_Kart.GetMaxSpeed(), opponentUpcomingVelocities[k.name][idx] + (HighMode == HighLevelMode.MCTS ? gameParams.velocityBucketSize * 2 : 0));
                    }
                    else
                    {
                        lane = m_envController.Sections[idx].Trigger;
                        vel = k.m_Kart.GetMaxSpeed();
                    }
                }
                BoxCollider centerLine = m_envController.Sections[idx].Trigger;
                BoxCollider nextLane;
                idx = (s + 1) % m_envController.Sections.Length;
                if (k == this)
                {
                    if (m_UpcomingLanes.ContainsKey(idx))
                    {
                        nextLane = m_envController.Sections[idx].getBoxColliderForLane(m_UpcomingLanes[idx]);
                    }
                    else
                    {
                        nextLane = m_envController.Sections[idx].Trigger;
                    }
                }
                else
                {
                    if (opponentUpcomingLanes[k.name].ContainsKey(idx))
                    {
                        nextLane = m_envController.Sections[idx].getBoxColliderForLane(opponentUpcomingLanes[k.name][idx]);
                    }
                    else
                    {
                        nextLane = m_envController.Sections[idx].Trigger;
                    }
                }
                targetState[KartMPC.xIndex] = lane.transform.position.x;
                targetState[KartMPC.zIndex] = lane.transform.position.z;
                if (k.GetComponent<Rigidbody>().velocity.magnitude <= 5f)
                {
                    targetState[KartMPC.vIndex] = 0.0f;
                }
                else
                {
                    targetState[KartMPC.vIndex] = vel;
                }
                // print(k.name + " vel: " + vel);
                // Heuristic to calculate target heading depending on the walls nearby and curvature of the next and following checkpoints
                double finalTargetHeading;
                var targetHeading = Mathf.Atan2(lane.transform.position.z - k.m_Kart.transform.position.z, lane.transform.position.x - k.m_Kart.transform.position.x);
                if (targetHeading < 0) targetHeading += 2 * Mathf.PI;
                if ((lane.transform.position - k.m_Kart.transform.position).magnitude <= (k.m_envController.sectionIsStraight(k.m_SectionIndex) ? 10.5f : 7.5f)) // If we're approaching the target checkpoint
                {

                    float finalTargetHeading1 = Mathf.Atan2(lane.transform.position.z - k.m_Kart.transform.position.z, lane.transform.position.x - k.m_Kart.transform.position.x);
                    float finalTargetHeading2 = Mathf.Atan2(nextLane.transform.position.z - lane.transform.position.z, nextLane.transform.position.x - lane.transform.position.x); ;
                    float finalTargetHeading3 = Mathf.Atan2(nextLane.transform.forward.z, nextLane.transform.forward.x);
                    float finalTargetHeading4 = Mathf.Atan2(lane.transform.forward.z, lane.transform.forward.x);
                    float finalTargetHeading5 = Mathf.Atan2(centerLine.transform.position.z - k.m_Kart.transform.position.z, centerLine.transform.position.x - k.m_Kart.transform.position.x);
                    float finalTargetHeading6 = Mathf.Atan2(nextLane.transform.position.z - k.m_Kart.transform.position.z, nextLane.transform.position.x - k.m_Kart.transform.position.x);
                    var cutTrack = Physics.Raycast(lane.transform.position, nextLane.transform.position - lane.transform.position, out var hitTrackInfo,
                                         (lane.transform.position - nextLane.transform.position).magnitude, TrackMask, QueryTriggerInteraction.Ignore);
                    var hitTrack0 = Physics.Raycast(k.Sensors[0].Transform.position, k.Sensors[0].Transform.forward, out var hitTrackInfo0,
                       k.m_Kart.Rigidbody.velocity.magnitude * 0.5f, TrackMask, QueryTriggerInteraction.Ignore);
                    //var hitTrack0 = false;
                    var hitTrack1 = Physics.Raycast(k.Sensors[2].Transform.position, k.Sensors[2].Transform.forward, out var hitTrackInfo1,
                        2.0f, TrackMask, QueryTriggerInteraction.Ignore);
                    var hitTrack2 = Physics.Raycast(k.Sensors[4].Transform.position, k.Sensors[4].Transform.forward, out var hitTrackInfo2,
                        1.5f, TrackMask, QueryTriggerInteraction.Ignore);
                    var hitTrack3 = Physics.Raycast(k.Sensors[8].Transform.position, k.Sensors[8].Transform.forward, out var hitTrackInfo3,
                        1.5f, TrackMask, QueryTriggerInteraction.Ignore);
                    var hitTrack4 = Physics.Raycast(k.Sensors[6].Transform.position, k.Sensors[6].Transform.forward, out var hitTrackInfo4,
                        2.0f, TrackMask, QueryTriggerInteraction.Ignore);

                    if (cutTrack) // If target lane cuts across off the track
                    {
                        // print(k.name + "here1");
                        if (finalTargetHeading5 < 0) finalTargetHeading5 += 2 * Mathf.PI;
                        finalTargetHeading = finalTargetHeading5;
                        if (finalTargetHeading < 0) finalTargetHeading += 2 * Mathf.PI;
                        finalTargetHeading = initial[KartMPC.hIndex] - AngleDifference(initial[KartMPC.hIndex], finalTargetHeading);
                        // print(k.name + "here1 Current Heading " + initial[KartMPC.hIndex] + " Target Heading " + finalTargetHeading);

                    }
                    // Check if side of car would be near a track in the opposite direction of the heading
                    else if ((hitTrack1 || hitTrack2 || hitTrack3 || hitTrack4) && (Mathf.Sign(finalTargetHeading1) == Mathf.Sign(finalTargetHeading5)) || hitTrack0)
                    {
                        if (finalTargetHeading5 < 0) finalTargetHeading5 += 2 * Mathf.PI;
                        finalTargetHeading = finalTargetHeading5 - AngleDifference(finalTargetHeading1, finalTargetHeading5) * 0.7f;
                        if (finalTargetHeading < 0) finalTargetHeading += 2 * Mathf.PI;
                        finalTargetHeading = initial[KartMPC.hIndex] - AngleDifference(initial[KartMPC.hIndex], finalTargetHeading);
                        //print(k.name + "here2 Current Heading " + initial[KartMPC.hIndex] + " Target Heading " + finalTargetHeading + "\n" +
                        //  " ht0: " + hitTrack0 + "," + hitTrackInfo0.distance + " ht1: " + hitTrack1 + "," + hitTrackInfo1.distance + " ht2: " + hitTrack2 + "," + hitTrackInfo2.distance + " ht3: "  + hitTrack3 + "," + hitTrackInfo3.distance + " ht4: " + hitTrack4 + "," + hitTrackInfo4.distance);
                    } // Check if side of car would be near track in the same direction as the heading
                    else if ((hitTrack1 || hitTrack2 || hitTrack3 || hitTrack4) && (Mathf.Sign(finalTargetHeading1) != Mathf.Sign(finalTargetHeading5)))
                    {
                        if (finalTargetHeading5 < 0) finalTargetHeading5 += 2 * Mathf.PI;
                        finalTargetHeading = finalTargetHeading5;
                        if (finalTargetHeading < 0) finalTargetHeading += 2 * Mathf.PI;
                        finalTargetHeading = initial[KartMPC.hIndex] - AngleDifference(initial[KartMPC.hIndex], finalTargetHeading);
                        // print(k.name + "here3 Current Heading " + initial[KartMPC.hIndex] + " Target Heading " + finalTargetHeading);
                    } // Check if we're near the checkpoint, so we can target the following checkpoint
                    else if ((centerLine.ClosestPoint(k.m_Kart.transform.position) - k.m_Kart.transform.position).magnitude <= 4f)
                    {
                        // print(k.name + "here2");

                        targetState[KartMPC.xIndex] = nextLane.transform.position.x;
                        targetState[KartMPC.zIndex] = nextLane.transform.position.z;
                        if (finalTargetHeading6 < 0) finalTargetHeading6 += 2 * Mathf.PI;
                        finalTargetHeading = finalTargetHeading6;
                        if (finalTargetHeading < 0) finalTargetHeading += 2 * Mathf.PI;
                        finalTargetHeading = initial[KartMPC.hIndex] - AngleDifference(initial[KartMPC.hIndex], finalTargetHeading);
                        // print(k.name + "here4 Current Heading " + initial[KartMPC.hIndex] + " Target Heading " + finalTargetHeading);

                    }
                    else // normal case
                    {
                        // print(k.name + "here3");

                        if (finalTargetHeading1 < 0) finalTargetHeading1 += 2 * Mathf.PI;
                        if (finalTargetHeading2 < 0) finalTargetHeading2 += 2 * Mathf.PI;

                        finalTargetHeading = finalTargetHeading1 - AngleDifference(finalTargetHeading2, finalTargetHeading1) * 0.4f;
                        if (finalTargetHeading < 0) finalTargetHeading += 2 * Mathf.PI;
                        finalTargetHeading = initial[KartMPC.hIndex] - AngleDifference(initial[KartMPC.hIndex], finalTargetHeading);
                        // print(k.name + "here5 Current Heading " + initial[KartMPC.hIndex] + " Target Heading " + finalTargetHeading);

                    }
                }
                else
                {
                    var hitTrack1 = Physics.Raycast(k.m_Kart.transform.position, k.m_Kart.transform.forward, out var hitTrackInfo1,
                                        k.m_envController.sectionIsStraight(k.m_SectionIndex)? 8: 5, TrackMask, QueryTriggerInteraction.Ignore);
                    if (hitTrack1) // Check if we're wuld cut the track
                    {
                        // print(k.name + "here4");
                        float finalTargetHeading1 = Mathf.Atan2(centerLine.transform.position.z - k.m_Kart.transform.position.z, centerLine.transform.position.x - k.m_Kart.transform.position.x);
                        if (finalTargetHeading1 < 0) finalTargetHeading1 += 2 * Mathf.PI;

                        finalTargetHeading = initial[KartMPC.hIndex] - AngleDifference(initial[KartMPC.hIndex], finalTargetHeading1)*0.85f;
                        // print(k.name + "here6 Current Heading " + initial[KartMPC.hIndex] + " Target Heading " + finalTargetHeading);
                    
                    }
                    else // normal case to just set the target heading to the true lane
                    {
                        finalTargetHeading = initial[KartMPC.hIndex] - AngleDifference(initial[KartMPC.hIndex], targetHeading);
                        // print(k.name + "here7 Current Heading " + initial[KartMPC.hIndex] + " Target Heading " + finalTargetHeading);
                    }
                }
                // print(k.name + " Current Heading " + initial[KartMPC.hIndex] + " Target Heading " + finalTargetHeading);
                targetState[KartMPC.hIndex] = finalTargetHeading;

                var targetWeights = new Dictionary<int, double>();
                targetWeights[KartMPC.hIndex] = (HighMode == HighLevelMode.Fixed ? 2.1: 3.5) ;
                if (k.GetComponent<Rigidbody>().velocity.magnitude <= 5f)
                {
                    targetWeights[KartMPC.xIndex] = 0.3 * (HighMode == HighLevelMode.Fixed ? 3.1 : 3.1);
                    targetWeights[KartMPC.zIndex] = 0.3 * (HighMode == HighLevelMode.Fixed ? 3.1 : 3.1);
                    targetWeights[KartMPC.vIndex] = -2;
                }
                else
                {
                    targetWeights[KartMPC.xIndex] = 0.3 * (HighMode == HighLevelMode.Fixed ? 3.1 : 3.1) / Math.Max(1, initial[KartMPC.vIndex]);
                    targetWeights[KartMPC.zIndex] = 0.3 * (HighMode == HighLevelMode.Fixed ? 3.1 : 3.1) / Math.Max(1, initial[KartMPC.vIndex]);
                    targetWeights[KartMPC.vIndex] = (HighMode == HighLevelMode.Fixed ? 5e-4 : 5e-4) ;
                }

                var avoidWeights = new Dictionary<int, List<double>>();
                avoidWeights[KartMPC.xIndex] = new List<double>();
                avoidWeights[KartMPC.zIndex] = new List<double>();
                var avoidIndices = new Dictionary<int, List<int>>();
                avoidIndices[KartMPC.xIndex] = new List<int>();
                avoidIndices[KartMPC.zIndex] = new List<int>();

                var opponentTargetStates = new List<Vector<double>>();
                var opponentTargetWeights = new List<Dictionary<int, double>>();

                // Process opponents
                var avoidDynamics = new List<KartLQRDynamics>();
                for (int j = 0; j < k.otherAgents.Length; j++)
                {
                    var o = otherAgents[j];
                    // Avoidance Weights
                    if (o.name == k.name || (o.m_Kart.transform.position - k.m_Kart.transform.position).magnitude > 8 || !o.is_active)
                    {
                        avoidWeights[KartMPC.xIndex].Add(0.00);
                        avoidIndices[KartMPC.xIndex].Add(KartMPC.xIndex);
                        avoidWeights[KartMPC.zIndex].Add(0.00);
                        avoidIndices[KartMPC.zIndex].Add(KartMPC.zIndex);
                    }
                    else
                    {
                        float multiplier = HighMode == HighLevelMode.Fixed? 1f : 1.2f;
                        avoidWeights[KartMPC.xIndex].Add(1f / (Mathf.Pow((o.m_Kart.transform.position - k.m_Kart.transform.position).magnitude, 1.5f) * multiplier));
                        avoidIndices[KartMPC.xIndex].Add(KartMPC.xIndex);
                        avoidWeights[KartMPC.zIndex].Add(1f / (Mathf.Pow((o.m_Kart.transform.position - k.m_Kart.transform.position).magnitude, 1.5f) * multiplier));
                        avoidIndices[KartMPC.zIndex].Add(KartMPC.zIndex);
                    }

                    // Create Other Dynamics
                    var otherInitial = CreateVector.Dense<double>(4);
                    otherInitial[KartMPC.xIndex] = o.m_Kart.transform.position.x;
                    otherInitial[KartMPC.zIndex] = o.m_Kart.transform.position.z;
                    otherInitial[KartMPC.vIndex] = o.m_Kart.Rigidbody.velocity.magnitude;
                    heading = Mathf.Atan2(o.m_Kart.transform.forward.z, o.m_Kart.transform.forward.x);
                    otherInitial[KartMPC.hIndex] = heading;
                    avoidDynamics.Add(new LinearizedBicycle(dt, otherInitial));

                    // Create Opponent Target State
                    var otherTarget = CreateVector.Dense<double>(avoidDynamics.Last().getXDim());
                    s = o.m_SectionIndex + 1;
                    idx = s % m_envController.Sections.Length;
                    if (o == this)
                    {
                        if (m_UpcomingLanes.ContainsKey(idx))
                        {
                            lane = m_envController.Sections[idx].getBoxColliderForLane(m_UpcomingLanes[idx]);
                            vel = Math.Min(m_Kart.GetMaxSpeed(), m_UpcomingVelocities[idx] + (HighMode == HighLevelMode.MCTS ? gameParams.velocityBucketSize * 2 : 0));
                        }
                        else
                        {
                            lane = m_envController.Sections[idx].Trigger;
                            vel = m_Kart.GetMaxSpeed();
                        }
                    }
                    else
                    {
                        if (opponentUpcomingLanes[o.name].ContainsKey(idx))
                        {
                            lane = m_envController.Sections[idx].getBoxColliderForLane(opponentUpcomingLanes[o.name][idx]);
                            vel = Math.Min(o.m_Kart.GetMaxSpeed(), opponentUpcomingVelocities[o.name][idx] + (HighMode == HighLevelMode.MCTS ? gameParams.velocityBucketSize * 2 : 0));
                        }
                        else
                        {
                            lane = m_envController.Sections[idx].Trigger;
                            vel = o.m_Kart.GetMaxSpeed();
                        }
                    }
                    otherTarget[KartMPC.xIndex] = lane.transform.position.x;
                    otherTarget[KartMPC.zIndex] = lane.transform.position.z;
                    otherTarget[KartMPC.vIndex] = vel;
                    opponentTargetStates.Add(otherTarget);

                    // Add weights for preventing or helping opponent's target state
                    var otherTargetWeights = new Dictionary<int, double>();

                    otherTargetWeights[KartMPC.xIndex] = 0.1 / Math.Max(1, initial[KartMPC.vIndex]);
                    otherTargetWeights[KartMPC.zIndex] = 0.1 / Math.Max(1, initial[KartMPC.vIndex]);
                    otherTargetWeights[KartMPC.vIndex] = 0.08;
                    opponentTargetWeights.Add(otherTargetWeights);
                }

                // Process teammates
                for (int j = 0; j < k.teamAgents.Length; j++)
                {
                    var o = teamAgents[j];
                    // Avoidance Weights
                    if (o.name == k.name || (o.m_Kart.transform.position - k.m_Kart.transform.position).magnitude > 8 || !o.is_active)
                    {
                        avoidWeights[KartMPC.xIndex].Add(0.0);
                        avoidIndices[KartMPC.xIndex].Add(KartMPC.xIndex);
                        avoidWeights[KartMPC.zIndex].Add(0.0);
                        avoidIndices[KartMPC.zIndex].Add(KartMPC.zIndex);
                    }
                    else
                    {
                        float multiplier = HighMode == HighLevelMode.Fixed ? 0.022f : 0.048f;
                        avoidWeights[KartMPC.xIndex].Add(1f / (Mathf.Pow((o.m_Kart.transform.position - k.m_Kart.transform.position).magnitude, 1.5f) * multiplier));
                        avoidIndices[KartMPC.xIndex].Add(KartMPC.xIndex);
                        avoidWeights[KartMPC.zIndex].Add(1f / (Mathf.Pow((o.m_Kart.transform.position - k.m_Kart.transform.position).magnitude, 1.5f) * multiplier));
                        avoidIndices[KartMPC.zIndex].Add(KartMPC.zIndex);
                    }



                    // Create Other Dynamics
                    var otherInitial = CreateVector.Dense<double>(4);
                    otherInitial[KartMPC.xIndex] = o.m_Kart.transform.position.x;
                    otherInitial[KartMPC.zIndex] = o.m_Kart.transform.position.z;
                    otherInitial[KartMPC.vIndex] = o.m_Kart.Rigidbody.velocity.magnitude;
                    heading = Mathf.Atan2(o.m_Kart.transform.forward.z, o.m_Kart.transform.forward.x);
                    otherInitial[KartMPC.hIndex] = heading;
                    avoidDynamics.Add(new LinearizedBicycle(dt, otherInitial));

                    // Create Opponent Target State
                    var otherTarget = CreateVector.Dense<double>(avoidDynamics.Last().getXDim());
                    s = o.m_SectionIndex + 1;
                    idx = s % m_envController.Sections.Length;
                    if (o == this)
                    {
                        if (m_UpcomingLanes.ContainsKey(idx))
                        {
                            lane = m_envController.Sections[idx].getBoxColliderForLane(m_UpcomingLanes[idx]);
                            vel = m_Kart.getMaxSpeedForState();
                        }
                        else
                        {
                            lane = m_envController.Sections[idx].Trigger;
                            vel = m_Kart.getMaxSpeedForState();
                        }
                    }
                    else
                    {
                        if (opponentUpcomingLanes[o.name].ContainsKey(idx))
                        {
                            lane = m_envController.Sections[idx].getBoxColliderForLane(opponentUpcomingLanes[o.name][idx]);
                            vel = o.m_Kart.getMaxSpeedForState();
                        }
                        else
                        {
                            lane = m_envController.Sections[idx].Trigger;
                            vel = o.m_Kart.getMaxSpeedForState();
                        }
                    }
                    otherTarget[KartMPC.xIndex] = lane.transform.position.x;
                    otherTarget[KartMPC.zIndex] = lane.transform.position.z;
                    otherTarget[KartMPC.vIndex] = vel;
                    opponentTargetStates.Add(otherTarget);

                    // Add weights for preventing or helping opponent's target state
                    var otherTargetWeights = new Dictionary<int, double>();
                    otherTargetWeights[KartMPC.xIndex] = HighMode == HighLevelMode.Fixed ? -0.033 : -0.033f;
                    otherTargetWeights[KartMPC.zIndex] = HighMode == HighLevelMode.Fixed ? -0.033 : -0.033f;
                    otherTargetWeights[KartMPC.vIndex] = HighMode == HighLevelMode.Fixed ? -0.08 : -0.08f;
                    opponentTargetWeights.Add(otherTargetWeights);
                }


                costs.Add(new LQRCheckpointReachAvoidCost(targetState, targetWeights, HighMode == HighLevelMode.Fixed ? 0.115 : 0.115, dynamics.Last(), opponentTargetStates, opponentTargetWeights, avoidWeights, avoidIndices, avoidDynamics));
            }

            // Sovle LQR
            var resultVector = KartLQR.solveFeedbackLQR(dynamics, costs, initialStates, HighMode == HighLevelMode.Fixed ? numSteps : 3);
            // print(resultVector.ToString());
            // Parse Results
            var angVel = Mathf.Clamp((float)resultVector[1], -m_Kart.getMaxAngularVelocity(), m_Kart.getMaxAngularVelocity());
            if (resultVector[0] < 0)
            {
                m_Acceleration = false;
                m_Brake = true;
            }
            else if (resultVector[0] > 0)
            {
                m_Acceleration = true;
                m_Brake = false;
            }
            else
            {
                m_Acceleration = false;
                m_Brake = false;
                angVel = 0.0f;
            }

            m_Steering = angVel / (0.4f * m_Kart.m_FinalStats.Steer);
            // print("Initial vector " + initialStates[0] + " result output " + resultVector);
            // print("Result Control input Accelerate " + m_Acceleration + " Brake: " + m_Brake + " turning input: " + m_Steering);
            return new InputData
            {
                Accelerate = m_Acceleration,
                Brake = m_Brake,
                TurnInput = m_Steering
            };
        }

        /**
         * Calculate control inputs by using Iterated-Best Response and Model Predictive Control
         * This implementation relies is not used because the underlying SQP Solver is not fast enough to meet the control deadlines
        **/
        public InputData SolveMPC(int numSteps = 200)
        {
            List<KartAgent> allPlayers = m_envController.Agents.ToList();
            List<KartMPCDynamics> dynamics = new List<KartMPCDynamics>();
            List<DoubleVector> initialStates = new List<DoubleVector>();
            List<List<KartMPCConstraints>> individualConstraints = new List<List<KartMPCConstraints>>();
            List<List<KartMPCCosts>> individualCosts = new List<List<KartMPCCosts>>();
            List<List<CoupledKartMPCConstraints>> coupledConstraints = new List<List<CoupledKartMPCConstraints>>();
            List<List<CoupledKartMPCCosts>> coupledCosts = new List<List<CoupledKartMPCCosts>>();
            double dt = Time.fixedDeltaTime * 10;
            int currentPlayer = 0;
            int T = numSteps;
            for (int i = 0; i < allPlayers.Count; i++)
            {
                KartAgent k = allPlayers[i];
                if (k.Equals(this))
                {
                    currentPlayer = i;
                }
                // Create Dynamics
                dynamics.Add(new Bicycle(dt, k.m_Kart.m_FinalStats.Acceleration, -k.m_Kart.m_FinalStats.Braking, k.m_Kart.getMaxAngularVelocity(), -k.m_Kart.getMaxAngularVelocity(), k.m_Kart.GetMaxSpeed(), k.m_Kart.getMaxLateralGs()));
                // Create Initial Vector
                DoubleVector initial = new DoubleVector(numSteps * (KartMPC.xDim + KartMPC.uDim));
                initial[KartMPC.xIndex * numSteps] = k.m_Kart.transform.position.x;
                initial[KartMPC.zIndex * numSteps] = k.m_Kart.transform.position.z;
                initial[KartMPC.vIndex * numSteps] = k.m_Kart.Rigidbody.velocity.magnitude;
                var heading = Mathf.Atan2(k.m_Kart.transform.forward.z, k.m_Kart.transform.forward.x);
                initial[KartMPC.hIndex * numSteps] = heading;
                initial[KartMPC.aIndex * numSteps] = k.m_Kart.acc.magnitude;
                initial[KartMPC.sIndex * numSteps] = k.m_Kart.Rigidbody.angularVelocity.y;
                for (int t = 1; t < T; t++)
                {
                    // Piecewise Dynamics
                    initial[KartMPC.xIndex * T + (t)] = initial[KartMPC.xIndex * T + (t - 1)] + dt * initial[KartMPC.vIndex * T + (t - 1)] * Math.Cos(initial[KartMPC.hIndex * T + (t - 1)]);
                    initial[KartMPC.zIndex * T + (t)] = initial[KartMPC.zIndex * T + (t - 1)] + dt * initial[KartMPC.vIndex * T + (t - 1)] * Math.Sin(initial[KartMPC.hIndex * T + (t - 1)]);
                    initial[KartMPC.hIndex * T + (t)] = initial[KartMPC.hIndex * T + (t - 1)] + dt * initial[KartMPC.sIndex * T + (t - 1)];
                    initial[KartMPC.vIndex * T + (t)] = initial[KartMPC.vIndex * T + (t - 1)] + dt * initial[KartMPC.aIndex * T + (t - 1)];

                    // initial[KartMPC.aIndex * T + (t)] = 0;
                    // initial[KartMPC.sIndex * T + (t)] = 0;
                    initial[KartMPC.aIndex * T + (t)] = initial[KartMPC.aIndex*T + (t-1)];
                    initial[KartMPC.sIndex * T + (t)] = initial[KartMPC.sIndex*T + (t-1)];
                }

                // print(initial.ToString());
                initialStates.Add(initial);
                // Create Individual Constraints
                List<KartMPCConstraints> constraints = new List<KartMPCConstraints>();
                constraints.Add(new OnTrackConstraint(finerWaypoints, Math.Max(0, currentFinerIndex-15), currentFinerIndex + 15, 5));
                individualConstraints.Add(constraints);

                // Create Individual Costs
                List<KartMPCCosts> costs = new List<KartMPCCosts>();
                // print(currentFinerIndex + " " + m_SectionIndex*10);
                for (int s = m_SectionIndex + 1; s < m_SectionIndex + 1 + 1; s++)
                {
                    int idx = s % m_envController.Sections.Length;
                    if (m_UpcomingLanes.ContainsKey(idx))
                    {
                        var lane = m_envController.Sections[idx].getBoxColliderForLane(m_UpcomingLanes[idx]);
                        costs.Add(new WaypointCost(30, 4, lane.transform.position.x, lane.transform.position.z, m_UpcomingVelocities[idx]));
                    }
                }
                // costs.Add(new ForwardProgressReward(finerWaypoints, currentFinerIndex, currentFinerIndex +  10, 5000));
                individualCosts.Add(costs);

                List<CoupledKartMPCConstraints> cConstraints = new List<CoupledKartMPCConstraints>();
                List<CoupledKartMPCCosts> cCosts = new List<CoupledKartMPCCosts>();
                for (int j = 0; j < allPlayers.Count; j++)
                {
                    if (i == j || Mathf.Abs(allPlayers[j].m_SectionIndex - m_SectionIndex) > 1) continue;
                    KartAgent k2 = allPlayers[j];
                    // Create Coupled Constraints
                    cConstraints.Add(new CoupledDistanceConstraint(0.8, j));
                    // Create Coupled Costs
                    cCosts.Add(new CoupledProgressReward(finerWaypoints, Math.Max(0, currentFinerIndex-15), currentFinerIndex + 15, 20, j));
                }
                coupledConstraints.Add(cConstraints);
                coupledCosts.Add(cCosts);
            }

            // Sovle MPC
            resultVector = KartMPC.solveGame(dynamics, individualCosts, individualConstraints, coupledCosts, coupledConstraints, initialStates, numSteps);
            // Parse Results
            int finalT = T - 1;
            m_Acceleration = resultVector[currentPlayer][KartMPC.aIndex * numSteps+1] > 0;
            m_Brake = resultVector[currentPlayer][KartMPC.aIndex * numSteps+1] < 0;
            m_Steering = Mathf.Clamp((float)resultVector[currentPlayer][KartMPC.sIndex * numSteps+1], -m_Kart.getMaxAngularVelocity(), m_Kart.getMaxAngularVelocity()) / (0.4f * m_Kart.m_FinalStats.Steer);
            print(m_Acceleration + " " + m_Brake + " " + m_Steering);
            return new InputData
            {
                Accelerate = m_Acceleration,
                Brake = m_Brake,
                TurnInput = m_Steering
            };
        }


        // From StackOverflow: https://stackoverflow.com/questions/1878907/how-can-i-find-the-difference-between-two-angles
        public static double AngleDifference(double angle1, double angle2)
        {
            return Math.Atan2(Math.Sin((angle2 - angle1)), Math.Cos((angle2 - angle1)));
        }

        /**
         * Generate the InputData struct that is used by the Kart script
        **/
        public override InputData GenerateInput()
        {
            if (!is_active)
                return new InputData
                {
                    Accelerate = false,
                    Brake = false,
                    TurnInput = 0
                };
            //if (name.Equals(m_envController.Agents[0].name))
            //    print("Accelerate " + m_Acceleration + " brake " + m_Brake + "  steering " + m_Steering);
            return new InputData
            {
                Accelerate = m_Acceleration,
                Brake = m_Brake,
                TurnInput = m_Steering,
            };
        }

        /**
         * Provide Heuristic function for ML agents to test that the environment is setup correctly using human input 
        **/
        public override void InterpretDiscreteActions(ActionBuffers actions)
        {
            if (LowMode == LowLevelMode.RL)
            {
                m_Steering = actions.ContinuousActions[0];
                m_Acceleration = actions.DiscreteActions[0] > 1;
                m_Brake = actions.DiscreteActions[0] < 1;
            }
        }

        void OnDrawGizmos()
        {
            if (finerWaypoints != null)
            {
                Vector3 lastpt = new Vector3(finerWaypoints[0].x, 0, finerWaypoints[0].y);
                for (int j = 1; j < finerWaypoints.Count; j++)
                {
                    Vector3 wayPoint = new Vector3(finerWaypoints[j].x, 0, finerWaypoints[j].y);
                    Gizmos.color = Color.black;
                    Gizmos.DrawLine(lastpt, wayPoint);

                    lastpt = wayPoint;
                }
            }
            if (resultVector != null)
            {
                Color[] colors = { Color.red, Color.green, Color.yellow, Color.cyan };
                for (int i = 0; i < 2; i++)
                {
                    // print(resultVector[i].ToString());
                    Vector3 lastpt = new Vector3((float)resultVector[i][KartMPC.xIndex * mpcSteps], 0, (float)resultVector[i][KartMPC.zIndex * mpcSteps]);
                    for (int j = 1; j < mpcSteps; j++)
                    {
                        Vector3 wayPoint = new Vector3((float)resultVector[i][KartMPC.xIndex * mpcSteps + j], 0, (float)resultVector[i][KartMPC.zIndex * mpcSteps + j]);
                        Gizmos.color = colors[i];
                        Gizmos.DrawLine(lastpt, wayPoint);
                        lastpt = wayPoint;
                    }
                }
            }
        }
    }


}

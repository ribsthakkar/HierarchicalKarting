using KartGame.KartSystems;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using UnityEngine;
using Random = UnityEngine.Random;
using System.Collections.Generic;
using System;
using Unity.MLAgents.Actuators;
using System.Linq;
using Unity.MLAgents.Policies;
using KartGame.AI.MCTS;
using System.Threading;

namespace KartGame.AI
{
    public class EndToEndKartAgent : KartAgent
    {
        [HideInInspector] public const int velocityBucketSize = 1;
        [HideInInspector] public const int timePrecision = 100;
        [HideInInspector] public const float collisionWindow = 0.1f;
        [HideInInspector] public const int sectionWindow = 2;
        [HideInInspector] public const int treeSearchDepth = 8;
        [HideInInspector] DiscreteGameParams gameParams;
        [HideInInspector] KartMCTSNode currentRoot = null;
        [HideInInspector] int CyclesRootProcessed = 0;
        [HideInInspector] int abortAttempts = 0;
        [HideInInspector] bool HLReadyToUse = true;
        [HideInInspector] List<DiscreteGameState> bestStates = new List<DiscreteGameState>();
        [HideInInspector] Thread t = null;

        public bool runQuasiMCTS;

        protected override void FixedUpdate()
        {
            base.FixedUpdate();
            if (m_envController.episodeSteps % 100 == 0 && m_envController.episodeSteps < m_envController.maxEpisodeSteps && m_envController.episodeSteps > 0 && !m_envController.inactiveAgents.Contains(this))
            {
                if (Mode == AgentMode.Inferencing && runQuasiMCTS)
                {
                    if (t != null)
                    {
                        if (HLReadyToUse && !t.Join(5))
                        {
                            HLReadyToUse = false;
                            abortAttempts = 0;
                            StartCoroutine(ResetHLEnvironment());
                        }
                        else if (HLReadyToUse)
                        {
                            t = null;
                        }
                    }
                    if (HLReadyToUse)
                        planWithMCTS();

                }

            }
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
                            //print(kartState.name + " Will reach Section " + kartState.section + " at time " + kartState.timeAtSection + " in lane " + kartState.lane + " with velocity " + kartState.getAverageVelocity());
                        }
                    }
                }
            }
        }

        #region QuasiMCTS Code
        public override void prepareForReuse()
        {
            base.prepareForReuse();
            if (t != null)
            {
                t.Abort();
                while (t.IsAlive) ;
                t = null;

            }
            currentRoot = null;
            CyclesRootProcessed = 0;
            bestStates = new List<DiscreteGameState>();
            HLReadyToUse = true;
        }

        System.Collections.IEnumerator ResetHLEnvironment()
        {
            while (t.IsAlive)
            {
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

        /**
         * Although this is an End-to-End RL Kart, we have and MCTS planner here that allows us to collect metrics of how closely the E2E planner follows the MCTS plan
        **/
        public void planWithMCTS()
        {
            if (currentRoot == null && t == null)
            {
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
                        Debug.Log("LANE LESS THAN 0 for TEAM " + agent.name);
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
                    catch (ThreadAbortException e)
                    {
                        print("Here: " + e.ToString());
                        return;
                    }

                });
                t.IsBackground = true;
                t.Start();

            }
            else if (t == null && CyclesRootProcessed < 3)
            {
                t = new Thread(() =>
                {
                    try
                    {
                        currentRoot = KartMCTS.constructSearchTree(currentRoot, T: 0.9);
                        bestStates = KartMCTS.getBestStatesSequence(currentRoot);
                        CyclesRootProcessed += 1;
                    }
                    catch (ThreadAbortException e)
                    {
                        print("Here: " + e.ToString());
                        return;
                    }

                });
                t.IsBackground = true;
                t.Start();
            }
        }
        #endregion

        protected override void Awake()
        {
            base.Awake();
            var behaviorParameters = GetComponent<BehaviorParameters>();
            var brainParameters = behaviorParameters.BrainParameters;
            /**
             * Sensors.Lenght -> RayCasts
             * SectionHorizon * 5 -> Upcoming Checkpoints (Vector3 location, target speed, isStaright)
             * 8 -> Current Player's state
             * 12 -> Other player states
             **/
            brainParameters.VectorObservationSize = Sensors.Length + (sectionHorizon * 5) + (name.Equals("E2E")? 7: 8) + (12 * (teamAgents.Length + otherAgents.Length));

            // construct game params struct for quasi MCTS running
            gameParams = new DiscreteGameParams
            {
                collisionWindow = collisionWindow,
                sectionWindow = sectionWindow,
                timePrecision = timePrecision,
                treeSearchDepth = treeSearchDepth,
                velocityBucketSize = velocityBucketSize
            };
        }

        /**
         * Set Divider to just default because there is no target lane in E2E
        **/
        protected override void setLaneDifferenceDivider(int sectionIndex, int lane)
        {
            LaneDifferenceRewardDivider = 1.0f;
        }

        /**
         * Set divier to default because there is no target velocity in E2E
        **/
        protected override void setVelocityDifferenceDivider(int sectionIndex, float velocity)
        {
            VelocityDifferenceRewardDivider = 1.0f;
        }

        /**
         * Collect Observations for E2E RL brain
        **/
        public override void CollectObservations(VectorSensor sensor)
        {
            sensor.AddObservation(m_Kart.LocalSpeed());
            sensor.AddObservation(m_Acceleration);
            sensor.AddObservation(m_Lane);
            sensor.AddObservation(m_LaneChanges * 1f / m_envController.MaxLaneChanges);
            if (!name.Equals("E2E")) sensor.AddObservation(is_active);
            sensor.AddObservation(m_envController.sectionIsStraight(m_SectionIndex));
            sensor.AddObservation(m_Kart.TireWearProportion());
            sensor.AddObservation(m_SectionIndex * 1f / m_envController.goalSection);

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

            // Add an observation for direction of the agent to the next checkpoint and lane.
            for (int i = m_SectionIndex + 1; i < m_SectionIndex + 1 + sectionHorizon; i++)
            {
                var next = (i) % m_envController.Sections.Length;
                var nextSection = m_envController.Sections[next];
                Collider target = nextSection.Trigger;
                sensor.AddObservation(m_Kart.transform.InverseTransformPoint(target.transform.position));
                sensor.AddObservation(1.0f);
                sensor.AddObservation(m_envController.sectionIsStraight(next));
                if (ShowRaycasts)
                    Debug.DrawLine(AgentSensorTransform.position, target.transform.position, Color.magenta);
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
         * Interpret RL brain actions
        **/
        public override void OnActionReceived(ActionBuffers actions)
        {
            base.OnActionReceived(actions);
            if (!is_active)
            {
                SetZeroInputs();
                return;
            }
            InterpretDiscreteActions(actions);

            // Find the next checkpoint when registering the current checkpoint that the agent has passed.
            var next = (m_SectionIndex + 1) % m_envController.Sections.Length;
            var nextCollider = m_envController.Sections[next].Trigger;
            var direction = (nextCollider.transform.position - m_Kart.transform.position).normalized;
            var reward = Vector3.Dot(m_Kart.Rigidbody.velocity.normalized, direction);

            // if (ShowRaycasts) Debug.DrawRay(AgentSensorTransform.position, m_Kart.Rigidbody.velocity, Color.blue);

            // Add rewards if the agent is heading in the right direction
            var speedProportion = 0.00f;
            AddReward(reward * m_envController.TowardsCheckpointReward);
            AddReward((m_Acceleration && !m_Brake ? 1.0f : 0.0f) * m_envController.AccelerationReward);
            if (speedProportion > 0.01f && m_Kart.LocalSpeed() < speedProportion)
            {
                AddReward(m_envController.SlowMovingPenalty + (-m_envController.SlowMovingPenalty) * (m_Kart.LocalSpeed() / speedProportion));
            }
            else
            {
                AddReward((m_Kart.LocalSpeed() - speedProportion) / (1 - speedProportion) * m_envController.SpeedReward);
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
    }


}
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

namespace KartGame.AI
{
    public struct DiscreteKartAction
    {
        public int min_velocity;
        public int max_velocity;
        public int lane;
    }

    public struct DiscreteKartState
    {
        public int player;
        public int section;
        public int timeAtSection;
        public int min_velocity;
        public int max_velocity;
        public int lane;
        public int tireAge;
        public int laneChanges;
        public bool infeasible;
        
        public float getAverageVelocity()
        {
            return (1.0f * (min_velocity + max_velocity)) / 2.0f;
        }

        private float computeTOC(ArcadeKart kart, float distance, float initV, float finalV)
        {
            // Full acceleration or full braking does not have enough distance
            if (finalV > initV && (finalV*finalV - initV*initV)/(2*kart.m_FinalStats.Acceleration) > distance)
            {
                return -1.0f;
            }
            if (initV > finalV && (initV * initV - finalV * finalV) / (2 * kart.m_FinalStats.Braking) > distance)
            {
                return -1.0f;
            }

            float t1 = (kart.GetMaxSpeed() - initV) / kart.m_FinalStats.Acceleration;
            float x1 = 0.5f * (initV + kart.GetMaxSpeed()) * t1;
            float x3 = (kart.GetMaxSpeed() * kart.GetMaxSpeed() - finalV*finalV) / (2 * kart.m_FinalStats.Braking);
            float t3 = (kart.GetMaxSpeed() - finalV) / kart.m_FinalStats.Braking;
            float x2 = distance - x1 - x3;
            float t2 = x2 / kart.GetMaxSpeed();
            if (t2 > 0.001)
            {
                return t1 + t2 + t3;
            }
            else
            {
                float maxSpeed = Mathf.Sqrt((2*distance + initV*initV*kart.m_FinalStats.Braking + finalV*finalV*kart.m_FinalStats.Acceleration) / (kart.m_FinalStats.Acceleration + kart.m_FinalStats.Braking));
                t1 = (maxSpeed - initV) / kart.m_FinalStats.Acceleration;
                t3 = (maxSpeed - finalV) / kart.m_FinalStats.Braking;
                return t1 + t3;
            }
        }

        public DiscreteKartState applyAction(DiscreteKartAction action, RacingEnvController environment, DiscreteGameParams gameParams)
        {
            ArcadeKart kart = environment.Agents[player].m_Kart;
            DiscreteKartState newState = new DiscreteKartState();
            newState.player = player;
            newState.section = section + 1;
            newState.min_velocity = action.min_velocity;
            newState.max_velocity = action.max_velocity;
            newState.lane = action.lane;
            if (newState.lane != lane && environment.sectionIsStraight(section))
            {
                newState.laneChanges = laneChanges + 1;
            }
            else
            {
                newState.laneChanges = 0;
            }

            // Update timeAtSection estimate using 1D TOC
            float distanceInSection = environment.computeDistanceInSection(section, lane, action.lane);
            int timeUpdate = (int) computeTOC(kart, distanceInSection, getAverageVelocity(), newState.getAverageVelocity()) * gameParams.timePrecision;
            if (timeUpdate < 0)
            {
                newState.infeasible = true;
            }
            newState.timeAtSection = timeAtSection + timeUpdate;

            // Update TireAge estimate
            float tireLoad = environment.computeTireLoadInSection(section, action.max_velocity, lane, action.lane);
            newState.tireAge = tireAge + (int) (tireLoad * kart.m_FinalStats.TireWearFactor);

            return newState;
        }
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
    }

    public class DiscreteGameState
    {
        public RacingEnvController envController;
        public List<DiscreteKartState> kartStates;
        public List<KartAgent> kartAgents;
        public DiscreteGameParams gameParams;
        public int initialSection;
        public int lastCompletedSection;
        public int finalSection;


        public int upNext()
        {
            var ordered = kartStates.Where(s => true).ToList();
            ordered.Sort((a,b) => {
                if (a.section < b.section)
                {
                    return -1;
                }
                else if (a.section > b.section)
                {
                    return 1;
                }
                else
                {
                    if (a.timeAtSection < b.timeAtSection)
                    {
                        return -1;
                    }
                    else if (a.timeAtSection == b.timeAtSection)
                    {
                        if (a.getAverageVelocity() > b.getAverageVelocity())
                        {
                            return -1;
                        }
                        else if (a.getAverageVelocity() == b.getAverageVelocity())
                        {
                            return 0;
                        }
                        else
                        {
                            return 1;
                        }
                    }
                    else
                    {
                        return 1;
                    }
                }
            });
            for (int i = 0; i < ordered.Count; ++i)
            {
                if (ordered[i].section != lastCompletedSection + 1)
                {
                    for (int j = 0; j < kartStates.Count; j++)
                    {
                        if (ordered[i].Equals(kartStates[j]))
                        {
                            return j;
                        }
                    }
                    return -1;
                }
            }
            return -1;
        }
        public Tuple<bool, List<int>> isOver()
        {
            if (lastCompletedSection != finalSection)
            {
                return Tuple.Create(false, new List<int>());
            }
            else
            {
                List<int> scores = new List<int>();
                foreach(DiscreteKartState s in kartStates)
                {
                    int score = 0;
                    foreach(DiscreteKartState o in kartStates)
                    {
                        if (s.Equals(o))
                        {
                            score -= o.timeAtSection;
                        }
                        else
                        {
                            score += o.timeAtSection;
                        }
                    }
                    scores.Add(score);
                }
                return Tuple.Create(true, scores);
            }
        }
        public List<DiscreteKartAction> nextMoves()
        {
            int nextPlayer = upNext();
            KartAgent agent = kartAgents[nextPlayer];
            DiscreteKartState currentState = kartStates[nextPlayer];
            List<DiscreteKartAction> possibleActions = new List<DiscreteKartAction>();
            for(int i = 0; i < (int) agent.m_Kart.GetMaxSpeed(); i += gameParams.velocityBucketSize)
            {
                for (int j = 1; j < 5; j++)
                {
                    possibleActions.Add(new DiscreteKartAction
                    {
                        min_velocity = i,
                        max_velocity = Math.Min(i + gameParams.velocityBucketSize, (int) agent.m_Kart.GetMaxSpeed()),
                        lane = j
                    });
                }
            }
            return possibleActions.FindAll((action) =>
            {
                //UnityEngine.Debug.Log(currentState.lane + " " + action.lane + ", " + currentState.player);
                // Is Lane changing not allowed?
                if (envController.sectionIsStraight(lastCompletedSection) && currentState.laneChanges == 1 && action.lane != currentState.lane)
                {
                    return false;
                }                
                
                // Is lateral gs infeasible?
                if (!envController.sectionSpeedFeasible(lastCompletedSection, action.max_velocity, currentState.lane, action.lane, agent.m_Kart))
                {
                    return false;
                }

                // Is changing speed infeasible?
                DiscreteKartState appliedAction = currentState.applyAction(action, envController, gameParams);
                if(appliedAction.infeasible)
                {
                    return false;
                }


                // Will there be a collision?
                foreach (DiscreteKartState other in kartStates)
                {
                    if (other.player != appliedAction.player && other.section == lastCompletedSection + 1 && appliedAction.lane == other.lane && Math.Abs(appliedAction.timeAtSection - other.timeAtSection)*1.0f/gameParams.timePrecision < gameParams.collisionWindow)
                    {
                        Debug.Log("Our Player" + appliedAction.player);
                        Debug.Log("Other Player " + other.player);
                        Debug.Log("Our tas " + appliedAction.timeAtSection);
                        Debug.Log("Other tas " + other.timeAtSection);
                        Debug.Log("Collision Fear");
                        return false;
                    }
                }

                return true;
            }
            ).ToList();

        }
        public DiscreteGameState makeMove(DiscreteKartAction action)
        {
            DiscreteGameState newGameState = new DiscreteGameState
            {
                envController = envController,
                kartStates = kartStates.ConvertAll(state => state),
                kartAgents = kartAgents,
                initialSection = initialSection,
                lastCompletedSection = lastCompletedSection,
                finalSection = finalSection,
                gameParams = gameParams
            };
            int nextPlayer = upNext();
            DiscreteKartState currentState = newGameState.kartStates[nextPlayer];
            DiscreteKartState newState = currentState.applyAction(action, envController, gameParams);
            newGameState.kartStates[nextPlayer] = newState;
            bool allAhead = true;
            foreach (var state in newGameState.kartStates)
            {
                allAhead &= state.section > lastCompletedSection;
            }
            if (allAhead)
            {
                newGameState.lastCompletedSection += 1;
            }
            return newGameState;
        }
    } 

    /// <summary>
    /// The KartAgent will drive the inputs for the KartController.
    /// </summary>
    ///
    public class HierarchicalKartAgent : KartAgent
    {

        #region MCTS Params
        public DiscreteGameParams gameParams;
        #endregion

        public override void initialPlan()
        {
            planWithMCTS();
        }

        public void planWithMCTS()
        {
            List<KartAgent> nearbyAgents = new List<KartAgent>();
            List<DiscreteKartState> nearbyAgentStates = new List<DiscreteKartState>();
            int initialSection = m_SectionIndex;
            KartAgent furthestForwardAgent = this;
            foreach(KartAgent agent in m_envController.Agents)
            {
                if(Math.Abs(agent.m_SectionIndex - m_SectionIndex) < gameParams.sectionWindow)
                {
                    nearbyAgents.Add(agent);
                    initialSection = Math.Max(initialSection, agent.m_SectionIndex);
                    if (initialSection == agent.m_SectionIndex)
                    {
                        furthestForwardAgent = agent;
                    }
                }
            }
            int finalSection = initialSection + sectionHorizon;
            int count = 0;
            foreach(KartAgent agent in nearbyAgents)
            {
                int min_velocity = 0;
                int max_velocity = (int) agent.m_Kart.GetMaxSpeed();
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
                    timeAtSection = (int) ((agent.sectionTimes[agent.m_SectionIndex] - furthestForwardAgent.sectionTimes[agent.m_SectionIndex]) * Time.fixedDeltaTime * gameParams.timePrecision);
                }
                if (m_Lane <= 0)
                {
                    Debug.Log("LANE LESS THAN 0 for TEAM " + agent.name);
                }
                nearbyAgentStates.Add(new DiscreteKartState
                {
                    player = count,
                    section = initialSection,
                    timeAtSection = timeAtSection,
                    min_velocity = min_velocity,
                    max_velocity = max_velocity,
                    lane = agent.m_Lane,
                    tireAge = (int) (m_Kart.m_FinalStats.Steer/m_Kart.baseStats.Steer * 100),
                    laneChanges = agent.m_LaneChanges,
                    infeasible = false
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
            KartMCTSNode root = KartMCTS.constructSearchTree(initialGameState);
            List<DiscreteGameState> bestStates = KartMCTS.getBestStatesSequence(root);

            foreach(DiscreteGameState gameState in bestStates)
            {
                foreach(DiscreteKartState kartState in gameState.kartStates)
                {
                    if (gameState.kartAgents[kartState.player] == this && kartState.section > m_SectionIndex)
                    {
                        m_UpcomingLanes[kartState.section % m_envController.Sections.Length] = kartState.lane;
                        m_UpcomingVelocities[kartState.section % m_envController.Sections.Length] = kartState.getAverageVelocity();
                    }
                }
            }
        }

        protected override void FixedUpdate()
        {
            base.FixedUpdate();
            episodeSteps += 1;
            if (episodeSteps % 100 == 0 && !m_envController.inactiveAgents.Contains(this))
            {
                //try
                //{
                    planWithMCTS();
                //}
                //catch (Exception e)
                //{
                //    print(e.ToString());
                //    planRandomly();
                //}
            }
        }

        protected override void Awake()
        {
            base.Awake();
            var behaviorParameters = GetComponent<BehaviorParameters>();
            var brainParameters = behaviorParameters.BrainParameters;
            /**
             * Sensors.Lenght -> RayCasts
             * SectionHorizon*2 -> Upcoming Lanes and Velocities
             * 6 -> Current Player's state
             * 9 -> Other player states
             **/
            brainParameters.VectorObservationSize = Sensors.Length + sectionHorizon*2 + 6 + 9 * otherAgents.Length;
        }

        protected override void setLaneDifferenceDivider(int sectionIndex, int lane)
        {
            LaneDifferenceRewardDivider = 1.0f;
            if (lane != -1)
            {
                //var lines = m_UpcomingLanes.Select(kvp => kvp.Key + ": " + kvp.Value.ToString());
                //print(string.Join(",", lines));
                //print(sectionIndex);
                LaneDifferenceRewardDivider = (float) Math.Pow(2.0, 1.0*Math.Abs(lane - m_UpcomingLanes[sectionIndex % m_envController.Sections.Length]));
            }
        }

        protected override void setVelocityDifferenceDivider(int sectionIndex, float velocity)
        {
            VelocityDifferenceRewardDivider = 1.0f;
            //var lines1 = m_UpcomingLanes.Select(kvp => kvp.Key + ": " + kvp.Value.ToString());
            //print("Upcoming Lanes " + this.name + " " + string.Join(",", lines1));
            //var lines = m_UpcomingVelocities.Select(kvp => kvp.Key + ": " + kvp.Value.ToString());
            //print("Upcoming Velocities " + this.name + " " + string.Join(",", lines));
            //print(sectionIndex);
            if (Mathf.Abs(velocity - m_UpcomingVelocities[sectionIndex % m_envController.Sections.Length]) > gameParams.velocityBucketSize/2.0f)
                VelocityDifferenceRewardDivider = (float)Math.Pow(2.0, 1.0 * (Mathf.Abs(velocity - m_UpcomingVelocities[sectionIndex % m_envController.Sections.Length]) - gameParams.velocityBucketSize/2.0));
        }

        public override void CollectObservations(VectorSensor sensor)
        {
            //print("collecting observations");
            // Add observation for agent state (Speed, acceleration, lane, recent lane changes, tire age, section type)
            sensor.AddObservation(m_Kart.LocalSpeed());
            sensor.AddObservation(m_Acceleration);
            sensor.AddObservation(m_Lane);
            sensor.AddObservation(m_LaneChanges);
            //print("Section Index" + m_SectionIndex);
            sensor.AddObservation(m_envController.sectionIsStraight(m_SectionIndex));
            sensor.AddObservation(m_Kart.m_FinalStats.Steer / m_Kart.baseStats.Steer);

            // Add observation for opponent agent states (Speed, acceleration, lane, recent lane chagnes, section type, tire age, distance, direction)
            foreach (KartAgent agent in otherAgents)
            {
                sensor.AddObservation(agent.m_Kart.LocalSpeed());
                sensor.AddObservation(agent.m_Acceleration);
                sensor.AddObservation(agent.m_Lane);
                sensor.AddObservation(agent.m_LaneChanges);
                sensor.AddObservation(agent.gameObject.activeSelf);
                sensor.AddObservation(m_envController.Sections[agent.m_SectionIndex % m_envController.Sections.Length].isStraight());
                sensor.AddObservation(m_Kart.m_FinalStats.Steer / m_Kart.baseStats.Steer);
                sensor.AddObservation((agent.m_Kart.transform.position - m_Kart.transform.position).magnitude);
                sensor.AddObservation(Vector3.SignedAngle(m_Kart.transform.forward, agent.m_Kart.transform.position, Vector3.up));

            }

            // Add an observation for direction of the agent to the next checkpoint and lane and the velocity at that lane.
            for (int i = m_SectionIndex + 1; i < m_SectionIndex + 1 + sectionHorizon; i++)
            {
                var next = (i) % m_envController.Sections.Length;
                var nextSection = m_envController.Sections[next];
                if (nextSection == null)
                    return;
                if (m_UpcomingLanes.ContainsKey(next))
                {
                    BoxCollider targetLaneInSection = nextSection.getBoxColliderForLane(m_UpcomingLanes[next]);
                    var direction = (targetLaneInSection.transform.position - m_Kart.transform.position).normalized;
                    sensor.AddObservation(Vector3.Dot(m_Kart.Rigidbody.velocity.normalized, direction));
                    sensor.AddObservation(m_UpcomingVelocities[next] / m_Kart.GetMaxSpeed());
                    if (ShowRaycasts)
                        Debug.DrawLine(AgentSensorTransform.position, targetLaneInSection.transform.position, Color.magenta);
                }
                else
                {
                    sensor.AddObservation(3.5f);
                    sensor.AddObservation(0.5f);
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
                    Debug.DrawRay(AgentSensorTransform.position, xform.forward * current.RayDistance, Color.green);
                    Debug.DrawRay(AgentSensorTransform.position, xform.forward * current.HitValidationDistance,
                        Color.red);

                    if (hitTrack && hitTrackInfo.distance < current.HitValidationDistance && hitTrackInfo.distance < hitAgentInfo.distance)
                    {
                        Debug.DrawRay(hitTrackInfo.point, Vector3.up * 3.0f, Color.blue);
                    }
                    else if (hitAgent && hitAgentInfo.distance < current.HitValidationDistance)
                    {
                        Debug.DrawRay(hitAgentInfo.point, Vector3.up * 3.0f, Color.blue);
                    }
                }
                //print("TrackDist");
                //print(hitTrackInfo.distance);
                //print(hitTrack);
                //print("AgentDist");
                //print(hitAgentInfo.distance);
                //print(hitAgent);
                //print(current.HitValidationDistance);
                if (hitTrack)
                {
                    if (hitTrackInfo.distance < current.HitValidationDistance && (!hitAgent || hitTrackInfo.distance < hitAgentInfo.distance))
                    {
                        //print("hit wall");
                        m_HitOccured = true;
                        m_envController.ResolveEvent(Event.HitWall, this, null);
                    }
                    sensor.AddObservation(hitTrackInfo.distance);
                }
                else if (hitAgent)
                {
                    if (hitAgentInfo.distance < current.HitValidationDistance)
                    {
                        //print("hit agent");
                        m_HitOccured = true;
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

    }


}

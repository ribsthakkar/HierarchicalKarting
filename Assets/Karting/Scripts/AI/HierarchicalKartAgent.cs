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
using System.Threading;
using CenterSpace.NMath.Core;
using KartGame.AI.MPC;
using KartGame.AI.LQR;

namespace KartGame.AI
{
    [System.Serializable]
    public enum LowLevelMode
    {
        RL,
        MPC
    }

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
        public string name;
        

        public static DiscreteKartState Copy(DiscreteKartState state)
        {
            return new DiscreteKartState
            {
                player = state.player,
                section = state.section,
                timeAtSection = state.timeAtSection,
                min_velocity = state.min_velocity,
                max_velocity = state.max_velocity,
                lane = state.lane,
                tireAge = state.tireAge,
                laneChanges = state.laneChanges,
                infeasible = state.infeasible,
                name = state.name
            };
        }

        public float getAverageVelocity()
        {
            return (1.0f * (min_velocity + max_velocity)) / 2.0f;
        }

        private float computeTOC(ArcadeKart kart, float distance, float initV, float finalV)
        {
            // Full acceleration or full braking does not have enough distance
            if (finalV > initV && (finalV*finalV - initV*initV)/(2*kart.m_FinalStats.Acceleration) > distance)
            {
                //Debug.Log(finalV);
                //Debug.Log(initV);
                //Debug.Log((finalV * finalV - initV * initV) / (2 * kart.m_FinalStats.Acceleration));
                //Debug.Log(distance);
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
            newState.name = name;
            newState.player = player;
            newState.section = section + 1;
            newState.min_velocity = action.min_velocity;
            newState.max_velocity = action.max_velocity;
            newState.lane = action.lane;
            if (environment.sectionIsStraight(section) != environment.sectionIsStraight(section + 1))
            {
                newState.laneChanges = 0;
            }
            else if (newState.lane != lane)
            {
                newState.laneChanges = laneChanges + Math.Abs(newState.lane - lane);
            }
            else
            {
                newState.laneChanges = laneChanges;
            }

            // Update timeAtSection estimate using 1D TOC
            float distanceInSection = environment.computeDistanceInSection(section, lane, action.lane);
            int timeUpdate = (int) (computeTOC(kart, distanceInSection, getAverageVelocity(), newState.getAverageVelocity()) * gameParams.timePrecision);
            if (timeUpdate < 0)
            {
                // Debug.Log(timeUpdate);
                newState.infeasible = true;
            }
            else
            {
                // Debug.Log("To travel " + distanceInSection + " it takes " + timeUpdate);
            }
            newState.timeAtSection = timeAtSection + timeUpdate;

            // Update TireAge estimate
            float tireLoad = environment.computeTireLoadInSection(section, action.max_velocity, lane, action.lane);
            newState.tireAge =  (int) ((tireAge/10000f + tireLoad * kart.m_FinalStats.TireWearFactor) * 10000);

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
        [Tooltip("How far to search in the tree?")]
        public int treeSearchDepth;
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
        public Tuple<bool, List<float>> isOver()
        {
            if (nextMoves().Count == 0)
            {
                List<float> scores = new List<float>();
                int noMovePlayer = upNext();
                for (int i = 0; i < kartStates.Count; i++)
                {
                    if (i == noMovePlayer)
                    {
                        scores.Add(0.0f);
                    }
                    scores.Add(0.5f);
                }
                return Tuple.Create(true, scores);
            }
            else if (lastCompletedSection != finalSection)
            {
                return Tuple.Create(false, new List<float>());
            }
            else if (kartStates.Count > 1)
            {
                int maxScore = gameParams.timePrecision * -1000;
                int minScore = gameParams.timePrecision * 1000;
                List<float> scores = new List<float>();
                List<float> normalizedScores = new List<float>();
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
                    maxScore = Math.Max(maxScore, score);
                    minScore = Math.Min(minScore, score);
                }
                foreach(int score in scores)
                {
                    normalizedScores.Add((score - minScore) * 1.0f / (maxScore - minScore));
                }
                return Tuple.Create(true, normalizedScores);
            } else
            {
                List<float> scores = new List<float>();
                scores.Add(envController.maxEpisodeSteps - kartStates[0].timeAtSection / envController.maxEpisodeSteps);
                return Tuple.Create(true, scores);
            }
        }
        public List<DiscreteKartAction> nextMoves()
        {
            int nextPlayer = upNext();
            KartAgent agent = kartAgents[nextPlayer];
            DiscreteKartState currentState = kartStates[nextPlayer];
            List<DiscreteKartAction> possibleActions = new List<DiscreteKartAction>();
            for(int i = 6; i < (int) agent.m_Kart.GetMaxSpeed(); i += gameParams.velocityBucketSize)
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
            List<DiscreteKartAction> preCollisionFilteredActions =  possibleActions.FindAll((action) =>
            {
                //UnityEngine.Debug.Log(currentState.lane + " " + action.lane + ", " + currentState.player);
                // Is Lane changing not allowed?
                if (envController.sectionIsStraight(currentState.section) && currentState.laneChanges + (action.lane - currentState.lane) > envController.MaxLaneChanges)
                {
                    // Debug.Log("Current Lane changes " + currentState.laneChanges);
                    // Debug.Log("Delta change " + (action.lane - currentState.lane));
                    // Debug.Log("Max change " + envController.MaxLaneChanges);
                    return false;
                }                
                
                // Is lateral gs infeasible?
                if (!envController.sectionSpeedFeasible(currentState.section, action.max_velocity, currentState.lane, action.lane, currentState.tireAge /10000f, agent.m_Kart))
                {
                    // Debug.Log("Tire age " + currentState.tireAge);
                    if (! envController.sectionIsStraight(currentState.section))
                    {
                        Debug.Log("Going on " + currentState.section + " from lane " + currentState.lane + " to " + action.lane + " at max velocity " + action.max_velocity + " with tire age " + currentState.tireAge/10000f + " is infeasible");
                    }
                    return false;
                }

                // Is changing speed infeasible?
                DiscreteKartState appliedAction = currentState.applyAction(action, envController, gameParams);
                if(appliedAction.infeasible)
                {
                    // Debug.Log("Going from speed " + currentState.getAverageVelocity() + " to " + appliedAction.getAverageVelocity() + " infeasible in section " + appliedAction.section);
                    return false;
                }

                return true;
            }
            ).ToList();
            if(preCollisionFilteredActions.Count == 0)
            {
                return preCollisionFilteredActions;
            }
            // If can't find collision free action set, allow collision action-set hoping that Low-level planner will prevent the crash
            List<DiscreteKartAction> filteredActions = new List<DiscreteKartAction>();
            float windowDivider = 1.0f;
            float windowGrowthRate = 2f;
            do
            {
                filteredActions = preCollisionFilteredActions.FindAll((action) =>
                {

                    DiscreteKartState appliedAction = currentState.applyAction(action, envController, gameParams);
                    // Will there be a collision?
                    foreach (DiscreteKartState other in kartStates)
                    {
                        //Debug.Log(Math.Abs(appliedAction.timeAtSection - other.timeAtSection) * 1.0f / gameParams.timePrecision);
                        if (false && !other.name.Equals(appliedAction.name) && other.section == appliedAction.section && appliedAction.lane == other.lane && Math.Abs(appliedAction.timeAtSection - other.timeAtSection) * 1.0f / gameParams.timePrecision < 0)
                        {
                            //Debug.Log("Our Player" + appliedAction.name);
                            //Debug.Log("Other Player " + other.name);
                            //Debug.Log("Our tas " + appliedAction.timeAtSection);
                            //Debug.Log("Other tas " + other.timeAtSection);
                            //Debug.Log("Collision Fear");
                            return false;
                        }
                    }
                    return true;
                }
                ).ToList();
                windowDivider *= windowGrowthRate;
            } while (filteredActions.Count == 0);
           
            return preCollisionFilteredActions;
        }
        public DiscreteGameState makeMove(DiscreteKartAction action)
        {
            DiscreteGameState newGameState = new DiscreteGameState
            {
                envController = envController,
                kartStates = kartStates.ConvertAll(state => DiscreteKartState.Copy(state)),
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
        [Tooltip("Are we using the RL brain for low level control or MPC?")]
        public LowLevelMode LowMode = LowLevelMode.MPC;

        [HideInInspector] KartMCTSNode currentRoot = null;
        [HideInInspector] List<DiscreteGameState> bestStates = new List<DiscreteGameState>();
        [HideInInspector] Thread t = null;
        [HideInInspector] List<Vector2> finerWaypoints;
        [HideInInspector] int currentFinerIndex;
        [HideInInspector] List<DoubleVector> resultVector;
        [HideInInspector] int mpcSteps = 5;

        public override void initialPlan()
        {

            currentRoot = null;
            if (Mode == AgentMode.Inferencing)
            {
                planWithMCTS();
            }
            else
            {
            planRandomly();
            }
        }

        public new void planRandomly()
        {
            for (int i = m_SectionIndex + 1; i < Math.Min(m_SectionIndex + gameParams.treeSearchDepth, 1000) + 1; i++)
            {
                if (!m_UpcomingLanes.ContainsKey(i % m_envController.Sections.Length))
                {
                    int lane = Random.Range(1, 4);
                    m_UpcomingLanes[i % m_envController.Sections.Length] = lane;
                    m_UpcomingVelocities[i % m_envController.Sections.Length] = Random.Range(5f, m_Kart.GetMaxSpeed());
                    if (name.Equals("KartClassic_HierarchicalMLAgent"))
                    {
                        m_envController.Sections[i % m_envController.Sections.Length].getBoxColliderForLane(lane).GetComponent<Renderer>().material.color = Color.green;
                        //print(kartState.name + " Will reach Section " + kartState.section + " at time " + kartState.timeAtSection + " in lane " + kartState.lane + " with velocity " + kartState.getAverageVelocity());
                    }
                }
            }
        }

        public void planWithMCTS()
        {
            if (currentRoot == null)
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
                        section = initialSection,
                        timeAtSection = timeAtSection,
                        min_velocity = min_velocity,
                        max_velocity = max_velocity,
                        lane = agent.m_Lane,
                        tireAge = (int)((m_Kart.baseStats.MaxSteer - m_Kart.m_FinalStats.Steer) / (m_Kart.baseStats.MaxSteer - m_Kart.baseStats.MinSteer) * 10000),
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
                currentRoot = KartMCTS.constructSearchTree(initialGameState);
                t = new Thread(() =>
                {
                    currentRoot = KartMCTS.constructSearchTree(initialGameState, T: 1.0);
                    bestStates = KartMCTS.getBestStatesSequence(currentRoot);
                });
                t.Start();
            }
            else
            {
                t = new Thread(() =>
                {
                    currentRoot = KartMCTS.constructSearchTree(currentRoot, T: 1.0);
                    bestStates = KartMCTS.getBestStatesSequence(currentRoot);
                });
                t.Start();
            }      
        }

        public new void Start()
        {
            base.Start();
            finerWaypoints = m_envController.Sections.SelectMany(s => s.finePoints).ToList();
            updateFinerIdxGuess();
        }

        protected override void FixedUpdate()
        {
            base.FixedUpdate();
            updateFinerIdxGuess();
            if (episodeSteps % 100 == 0 && !m_envController.inactiveAgents.Contains(this))
            {
                SolveMPC(numSteps: mpcSteps);
                if (t != null)
                    t.Join();

                if(Mode == AgentMode.Inferencing)
                {
                    planWithMCTS();
                }
                else
                {
                    planRandomly();
                }


            }
            // print(this.name + " " + initialGameState.kartAgents.Count);
            // print(bestStates.Count);

            foreach (DiscreteGameState gameState in bestStates)
            {
                foreach (DiscreteKartState kartState in gameState.kartStates)
                {
                    //if (name.Equals("KartClassic_HierarchicalMLAgent"))
                    //{
                    //    print(kartState.name);
                    //}

                    if (kartState.name.Equals(this.name) && kartState.section > m_SectionIndex)
                    {
                        if (m_UpcomingLanes.ContainsKey(kartState.section % m_envController.Sections.Length))
                        {
                            if (name.Equals("KartClassic_HierarchicalMLAgent"))
                                m_envController.Sections[kartState.section % m_envController.Sections.Length].resetColors();

                        }
                        m_UpcomingLanes[kartState.section % m_envController.Sections.Length] = kartState.lane;
                        m_UpcomingVelocities[kartState.section % m_envController.Sections.Length] = kartState.getAverageVelocity();
                        if (name.Equals("KartClassic_HierarchicalMLAgent"))
                        {
                            m_envController.Sections[kartState.section % m_envController.Sections.Length].getBoxColliderForLane(kartState.lane).GetComponent<Renderer>().material.color = Color.green;
                            //print(kartState.name + " Will reach Section " + kartState.section + " at time " + kartState.timeAtSection + " in lane " + kartState.lane + " with velocity " + kartState.getAverageVelocity());
                        }
                    }
                }
            }
            if (Mode == AgentMode.Inferencing)
            {
                var lines1 = m_UpcomingLanes.Select(kvp => kvp.Key + ": " + kvp.Value.ToString());
                print("Upcoming Lanes " + this.name + " " + string.Join(",", lines1));
                var lines = m_UpcomingVelocities.Select(kvp => kvp.Key + ": " + kvp.Value.ToString());
                print("Upcoming Velocities " + this.name + " " + string.Join(",", lines));
                print(m_SectionIndex);
            }
        }

        protected override void Awake()
        {
            base.Awake();
            var behaviorParameters = GetComponent<BehaviorParameters>();
            var brainParameters = behaviorParameters.BrainParameters;
            /**
             * Sensors.Lenght -> RayCasts
             * TreeDepth*5 -> Upcoming Lanes and Velocities and lane types
             * 6 -> Current Player's state
             * 11 -> Other player states
             **/
            brainParameters.VectorObservationSize = Sensors.Length + (gameParams.treeSearchDepth*5) + 6 + (11 *otherAgents.Length);
        }

        protected override void setLaneDifferenceDivider(int sectionIndex, int lane)
        {
            LaneDifferenceRewardDivider = 1.0f;
            if (lane != -1)
            {
                //var lines = m_UpcomingLanes.Select(kvp => kvp.Key + ": " + kvp.Value.ToString());
                //print(string.Join(",", lines));
                //print(sectionIndex);
                LaneDifferenceRewardDivider = -Mathf.Pow(1.5f, -1.0f*Math.Abs(lane - m_UpcomingLanes[sectionIndex % m_envController.Sections.Length]));
                //LaneDifferenceRewardDivider = Mathf.Pow(3f, 1.0f*Math.Abs(lane - m_UpcomingLanes[sectionIndex % m_envController.Sections.Length]) - 1);
            }
            else
            {
                //LaneDifferenceRewardDivider = -Mathf.Pow(2f, -1.0f *3);
                //LaneDifferenceRewardDivider = Mathf.Pow(2f, 1.0f *3);
            }
        }

        protected override void setVelocityDifferenceDivider(int sectionIndex, float velocity)
        {
            VelocityDifferenceRewardDivider = 1.0f;
            //var lines1 = m_UpcomingLanes.Select(kvp => kvp.Key + ": " + kvp.Value.ToString());
            //print("Upcoming Lanes " + this.name + " " + string.Join(",", lines1));
            //var lines = m_UpcomingVelocities.Select(kvp => kvp.Key + ": " + kvp.Value.ToString());
            // print("Upcoming Velocities " + this.name + " " + m_UpcomingVelocities[sectionIndex % m_envController.Sections.Length]);
            // print(sectionIndex);
            // print("Actual velocity " + velocity);
            if (Mathf.Abs(velocity - m_UpcomingVelocities[sectionIndex % m_envController.Sections.Length])/m_Kart.GetMaxSpeed() > gameParams.velocityBucketSize/2.0f)
                VelocityDifferenceRewardDivider = (float)Math.Pow(1.5, 1.0 * (Mathf.Abs(velocity - m_UpcomingVelocities[sectionIndex % m_envController.Sections.Length]) - gameParams.velocityBucketSize/2.0));
        }

        public override void CollectObservations(VectorSensor sensor)
        {
            //print("collecting observations");
            // Add observation for agent state (Speed, acceleration, lane, recent lane changes, tire age, section type)
            sensor.AddObservation(m_Kart.LocalSpeed());
            sensor.AddObservation(m_Acceleration);
            sensor.AddObservation(m_Lane);
            sensor.AddObservation(m_LaneChanges*1f/m_envController.MaxLaneChanges);
            //print("Section Index" + m_SectionIndex);
            sensor.AddObservation(m_envController.sectionIsStraight(m_SectionIndex));
            sensor.AddObservation(m_Kart.TireWearProportion());

            // Add observation for opponent agent states (Speed, acceleration, lane, recent lane chagnes, section type, tire age, distance, relative position)
            foreach (KartAgent agent in otherAgents)
            {
                sensor.AddObservation(agent.m_Kart.LocalSpeed());
                sensor.AddObservation(agent.m_Acceleration);
                sensor.AddObservation(agent.m_Lane);
                sensor.AddObservation(agent.m_LaneChanges * 1f / m_envController.MaxLaneChanges);
                sensor.AddObservation(agent.gameObject.activeSelf);
                sensor.AddObservation(m_envController.Sections[agent.m_SectionIndex % m_envController.Sections.Length].isStraight());
                sensor.AddObservation(agent.m_Kart.TireWearProportion());
                sensor.AddObservation((agent.m_Kart.transform.position - m_Kart.transform.position).magnitude);
                sensor.AddObservation(m_Kart.transform.InverseTransformPoint(agent.m_Kart.transform.position));

            }

            // Add an observation for direction of the agent to the next checkpoint and lane and the velocity at that lane.
            for (int i = m_SectionIndex + 1; i < m_SectionIndex + 1 + gameParams.treeSearchDepth; i++)
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
                    sensor.AddObservation(Vector3.zero);
                    sensor.AddObservation(0f);
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
                    Debug.DrawRay(AgentSensorTransform.position, xform.forward * current.RayDistance, Color.green);
                    //Debug.DrawRay(AgentSensorTransform.position, xform.forward * current.HitValidationDistance, Color.red);

                    if (hitTrack && hitTrackInfo.distance < current.WallHitValidationDistance && hitTrackInfo.distance < hitAgentInfo.distance)
                    {
                        Debug.DrawRay(hitTrackInfo.point, Vector3.up * 3.0f, Color.blue);
                    }
                    else if (hitAgent && hitAgentInfo.distance < current.AgentHitValidationDistance)
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
                    if (hitTrackInfo.distance < current.WallHitValidationDistance && (!hitAgent || hitTrackInfo.distance < hitAgentInfo.distance))
                    {
                        // print("hit wall");
                        // m_HitOccured = true;
                        m_envController.ResolveEvent(Event.HitWall, this, null);
                    }
                    sensor.AddObservation(hitTrackInfo.distance);
                }
                else if (hitAgent)
                {
                    if (hitAgentInfo.distance < current.AgentHitValidationDistance)
                    {
                        // print("hit agent");
                        // m_HitOccured = true;
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

        void OnTriggerEnter(Collider other)
        {
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
                    m_UpcomingLanes.Remove(index % m_envController.Sections.Length);
                    m_UpcomingVelocities.Remove(index % m_envController.Sections.Length);
                }
                if (name.Equals("KartClassic_HierarchicalMLAgent"))
                    for (int i = 1; i <= 4; i++)
                        m_envController.Sections[index % m_envController.Sections.Length].getBoxColliderForLane(i).GetComponent<Renderer>().material.color = Color.magenta;
                if (m_LaneChanges + Math.Abs(m_Lane - lane) > m_envController.MaxLaneChanges && m_envController.sectionIsStraight(m_SectionIndex))
                {
                    AddReward(m_envController.SwervingPenalty);
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
                if (false)
                {
                    m_envController.ResolveEvent(Event.ReachGoalSection, this, null);
                }
                else
                {
                    m_envController.ResolveEvent(Event.ReachNonGoalSection, this, null);
                }
                currentRoot = null;
            }
            else if ((triggered > 0 && index != -1) && ((index <= m_SectionIndex) || (m_SectionIndex % m_envController.Sections.Length == 0 && index % m_envController.Sections.Length == m_envController.Sections.Length - 1)))
            {
                // print("going backwards");
                AddReward(m_envController.ReversePenalty * (m_SectionIndex - index + 1));
            }
            else if ((triggered > 0 && index == -1))
            {
                m_envController.ResolveEvent(Event.DroveReverseLimit, this, null);
            }

        }

        void updateFinerIdxGuess()
        {
            currentFinerIndex = m_SectionIndex * 10;
            double minDistance = 1000;
            for(int initialFinerIdxGuess = m_SectionIndex * 10; initialFinerIdxGuess < m_SectionIndex*10 + 10; initialFinerIdxGuess++)
            {
                Vector2 pt = new Vector2(m_Kart.Rigidbody.transform.position.x, m_Kart.Rigidbody.transform.position.z);
                if((finerWaypoints[initialFinerIdxGuess] - pt).sqrMagnitude < minDistance)
                {
                    minDistance = (finerWaypoints[initialFinerIdxGuess] - pt).sqrMagnitude;
                    currentFinerIndex = initialFinerIdxGuess;
                }
            }
        }



        public InputData SolveMPC(int numSteps = 200)
        {
            List<KartAgent> allPlayers = new[] { this }.Concat(otherAgents).ToList();
            List<KartMPCDynamics> dynamics = new List<KartMPCDynamics>();
            List<DoubleVector> initialStates = new List<DoubleVector>();
            List<List<KartMPCConstraints>> individualConstraints = new List<List<KartMPCConstraints>>();
            List<List<KartMPCCosts>> individualCosts = new List<List<KartMPCCosts>>();
            List<List<CoupledKartMPCConstraints>> coupledConstraints = new List<List<CoupledKartMPCConstraints>>();
            List<List<CoupledKartMPCCosts>> coupledCosts = new List<List<CoupledKartMPCCosts>>();
            double dt = Time.fixedTimeAsDouble;
            for(int i = 0; i < allPlayers.Count; i ++)
            {
                KartAgent k = allPlayers[i];
                // Create Dynamics
                dynamics.Add(new Bicycle(dt, k.m_Kart.m_FinalStats.Acceleration, -k.m_Kart.m_FinalStats.Braking, k.m_Kart.getMaxAngularVelocity(), -k.m_Kart.getMaxAngularVelocity(), k.m_Kart.GetMaxSpeed(), k.m_Kart.getMaxLateralGs()));
                // Create Initial Vector
                DoubleVector initial = new DoubleVector(numSteps * (KartMPC.xDim + KartMPC.uDim));
                initial[KartMPC.xIndex * numSteps] = k.m_Kart.transform.position.x;
                initial[KartMPC.zIndex * numSteps] = k.m_Kart.transform.position.z;
                initial[KartMPC.vIndex * numSteps] = k.m_Kart.Rigidbody.velocity.magnitude;
                var heading = Mathf.Atan2(k.m_Kart.transform.forward.z, m_Kart.transform.forward.x);
                initial[KartMPC.hIndex * numSteps] =  heading;
                initial[KartMPC.aIndex * numSteps] = k.m_Kart.acc.magnitude;
                initial[KartMPC.sIndex * numSteps] = k.m_Kart.Rigidbody.angularVelocity.y;
                int T = numSteps;
                for (int t = 1; t < T; t++)
                {
                    // Piecewise Dynamics
                    initial[KartMPC.xIndex * T + (t)] = initial[KartMPC.xIndex * T + (t - 1)] + dt * initial[KartMPC.vIndex * T + (t - 1)] * Math.Cos(initial[KartMPC.hIndex * T + (t - 1)]);
                    // initial[KartMPC.xIndex * T + (t)] = finerWaypoints[(currentFinerIndex + t) % finerWaypoints.Count].x;
                    initial[KartMPC.zIndex * T + (t)] = initial[KartMPC.zIndex * T + (t - 1)] + dt * initial[KartMPC.vIndex * T + (t - 1)] * Math.Sin(initial[KartMPC.hIndex * T + (t - 1)]);
                    // initial[KartMPC.zIndex * T + (t)] = finerWaypoints[(currentFinerIndex + t) % finerWaypoints.Count].y;
                    initial[KartMPC.hIndex * T + (t)] = initial[KartMPC.hIndex * T + (t - 1)] + dt * initial[KartMPC.sIndex * T + (t - 1)];
                    initial[KartMPC.vIndex * T + (t)] = initial[KartMPC.vIndex * T + (t - 1)] + dt * initial[KartMPC.aIndex * T + (t - 1)];
                    initial[KartMPC.aIndex * T + (t)] = 0;
                    initial[KartMPC.sIndex * T + (t)] = 0;
                }

                print(initial.ToString());
                initialStates.Add(initial);
                // Create Individual Constraints
                List<KartMPCConstraints> constraints = new List<KartMPCConstraints>();
                //constraints.Add(new OnTrackConstraint(finerWaypoints, currentFinerIndex, currentFinerIndex + sectionHorizon / 2 * 10, 5));
                individualConstraints.Add(constraints);

                // Create Individual Costs
                List<KartMPCCosts> costs = new List<KartMPCCosts>();
                print(currentFinerIndex + " " + m_SectionIndex*10);
                for(int s = m_SectionIndex + 1; s < m_SectionIndex + 1 + 1; s++)
                {
                    int idx = s % m_envController.Sections.Length;
                    if (m_UpcomingLanes.ContainsKey(idx))
                    {
                        var lane = m_envController.Sections[idx].getBoxColliderForLane(m_UpcomingLanes[idx]);
                        // costs.Add(new WaypointCost(10, 4, lane.transform.position.x, lane.transform.position.z, m_UpcomingVelocities[idx]));
                    }
                }
                // costs.Add(new DistanceFromCenterCost(finerWaypoints, currentFinerIndex, currentFinerIndex + sectionHorizon / 2 * 10, 5f, 1000));
                //costs.Add(new ForwardProgressReward(finerWaypoints, currentFinerIndex, currentFinerIndex + sectionHorizon / 2 * 10, 50));
                //costs.Add(new DistanceTraveledReward(dt, 5));
                individualCosts.Add(costs);

                List<CoupledKartMPCConstraints> cConstraints = new List<CoupledKartMPCConstraints>();
                List<CoupledKartMPCCosts> cCosts = new List<CoupledKartMPCCosts>();
                for (int j = 0; j < allPlayers.Count; j++)
                {
                    if (i == j) continue;
                    KartAgent k2 = allPlayers[j];
                    // Create Coupled Constraints
                    // cConstraints.Add(new CoupledDistanceConstraint(0.8, j));
                    // Create Coupled Costs
                    // cCosts.Add(new CoupledProgressReward(finerWaypoints, currentFinerIndex, currentFinerIndex + sectionHorizon/2 * 10, 20, j));
                }
                coupledConstraints.Add(cConstraints);
                coupledCosts.Add(cCosts);
            }

            // Sovle MPC
            resultVector = KartMPC.solveGame(dynamics, individualCosts, individualConstraints, coupledCosts, coupledConstraints, initialStates, numSteps);
            // Parse Results

            return new InputData
            {
                Accelerate = m_Acceleration,
                Brake = m_Brake,
                TurnInput = m_Steering
            };
        }

        public InputData SolveLQR(int numSteps = 200)
        {
            List<KartAgent> allPlayers = new[] { this }.Concat(otherAgents).ToList();
            List<KartLQRDynamics> lqrDynamics = new List<KartLQRDynamics>();
            List<DoubleVector> initialStates = new List<DoubleVector>();
            List<DoubleMatrix> costs = new List<KartLQRCosts>();
            double dt = Time.fixedTimeAsDouble;
            for (int i = 0; i < allPlayers.Count; i++)
            {
                KartAgent k = allPlayers[i];
                // Create Dynamics
                lqrDynamics.Add(new LinearizedBicycle(dt, k.m_Kart.m_FinalStats.Acceleration, -k.m_Kart.m_FinalStats.Braking, k.m_Kart.getMaxAngularVelocity(), -k.m_Kart.getMaxAngularVelocity(), k.m_Kart.GetMaxSpeed(), k.m_Kart.getMaxLateralGs()));
                // Create Initial Vector
                DoubleVector initial = new DoubleVector(numSteps * (KartMPC.xDim + KartMPC.uDim));
                initial[KartMPC.xIndex] = k.m_Kart.transform.position.x;
                initial[KartMPC.zIndex] = k.m_Kart.transform.position.z;
                initial[KartMPC.vIndex] = k.m_Kart.Rigidbody.velocity.magnitude;
                var heading = Mathf.Atan2(k.m_Kart.transform.forward.z, m_Kart.transform.forward.x);
                initial[KartMPC.hIndex * numSteps] = heading;
                initial[KartMPC.aIndex * numSteps] = k.m_Kart.acc.magnitude;
                initial[KartMPC.sIndex * numSteps] = k.m_Kart.Rigidbody.angularVelocity.y;
                int T = numSteps;
                for (int t = 1; t < T; t++)
                {
                    // Piecewise Dynamics
                    initial[KartMPC.xIndex * T + (t)] = initial[KartMPC.xIndex * T + (t - 1)] + dt * initial[KartMPC.vIndex * T + (t - 1)] * Math.Cos(initial[KartMPC.hIndex * T + (t - 1)]);
                    // initial[KartMPC.xIndex * T + (t)] = finerWaypoints[(currentFinerIndex + t) % finerWaypoints.Count].x;
                    initial[KartMPC.zIndex * T + (t)] = initial[KartMPC.zIndex * T + (t - 1)] + dt * initial[KartMPC.vIndex * T + (t - 1)] * Math.Sin(initial[KartMPC.hIndex * T + (t - 1)]);
                    // initial[KartMPC.zIndex * T + (t)] = finerWaypoints[(currentFinerIndex + t) % finerWaypoints.Count].y;
                    initial[KartMPC.hIndex * T + (t)] = initial[KartMPC.hIndex * T + (t - 1)] + dt * initial[KartMPC.sIndex * T + (t - 1)];
                    initial[KartMPC.vIndex * T + (t)] = initial[KartMPC.vIndex * T + (t - 1)] + dt * initial[KartMPC.aIndex * T + (t - 1)];
                    initial[KartMPC.aIndex * T + (t)] = 0;
                    initial[KartMPC.sIndex * T + (t)] = 0;
                }

                print(initial.ToString());
                initialStates.Add(initial);
                // Create Individual Constraints
                List<KartMPCConstraints> constraints = new List<KartMPCConstraints>();
                //constraints.Add(new OnTrackConstraint(finerWaypoints, currentFinerIndex, currentFinerIndex + sectionHorizon / 2 * 10, 5));
                individualConstraints.Add(constraints);

                // Create Individual Costs
                List<KartMPCCosts> costs = new List<KartMPCCosts>();
                print(currentFinerIndex + " " + m_SectionIndex * 10);
                for (int s = m_SectionIndex + 1; s < m_SectionIndex + 1 + 1; s++)
                {
                    int idx = s % m_envController.Sections.Length;
                    if (m_UpcomingLanes.ContainsKey(idx))
                    {
                        var lane = m_envController.Sections[idx].getBoxColliderForLane(m_UpcomingLanes[idx]);
                        // costs.Add(new WaypointCost(10, 4, lane.transform.position.x, lane.transform.position.z, m_UpcomingVelocities[idx]));
                    }
                }
                // costs.Add(new DistanceFromCenterCost(finerWaypoints, currentFinerIndex, currentFinerIndex + sectionHorizon / 2 * 10, 5f, 1000));
                //costs.Add(new ForwardProgressReward(finerWaypoints, currentFinerIndex, currentFinerIndex + sectionHorizon / 2 * 10, 50));
                //costs.Add(new DistanceTraveledReward(dt, 5));
                individualCosts.Add(costs);

                List<CoupledKartMPCConstraints> cConstraints = new List<CoupledKartMPCConstraints>();
                List<CoupledKartMPCCosts> cCosts = new List<CoupledKartMPCCosts>();
                for (int j = 0; j < allPlayers.Count; j++)
                {
                    if (i == j) continue;
                    KartAgent k2 = allPlayers[j];
                    // Create Coupled Constraints
                    // cConstraints.Add(new CoupledDistanceConstraint(0.8, j));
                    // Create Coupled Costs
                    // cCosts.Add(new CoupledProgressReward(finerWaypoints, currentFinerIndex, currentFinerIndex + sectionHorizon/2 * 10, 20, j));
                }
                coupledConstraints.Add(cConstraints);
                coupledCosts.Add(cCosts);
            }

            // Sovle MPC
            resultVector = KartMPC.solveGame(dynamics, individualCosts, individualConstraints, coupledCosts, coupledConstraints, initialStates, numSteps);
            // Parse Results

            return new InputData
            {
                Accelerate = m_Acceleration,
                Brake = m_Brake,
                TurnInput = m_Steering
            };
        }


        public override InputData GenerateInput()
        {
            if (LowMode == LowLevelMode.RL)
            {
                return new InputData
                {
                    Accelerate = m_Acceleration,
                    Brake = m_Brake,
                    TurnInput = m_Steering
                };
            } else if (LowMode == LowLevelMode.MPC)
            {
                ;
            }
            return new InputData
            {
                Accelerate = false,
                Brake = false,
                TurnInput = 0f,
            };
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
                Color[] colors = {Color.red, Color.green, Color.yellow, Color.cyan };
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

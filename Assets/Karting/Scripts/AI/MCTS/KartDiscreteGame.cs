using KartGame.AI;
using KartGame.KartSystems;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace KartGame.AI.MCTS
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
        public int team;
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
                team = state.team,
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
            if (finalV > initV && (finalV * finalV - initV * initV) / (2 * kart.m_FinalStats.Acceleration) > distance)
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
            float x3 = (kart.GetMaxSpeed() * kart.GetMaxSpeed() - finalV * finalV) / (2 * kart.m_FinalStats.Braking);
            float t3 = (kart.GetMaxSpeed() - finalV) / kart.m_FinalStats.Braking;
            float x2 = distance - x1 - x3;
            float t2 = x2 / kart.GetMaxSpeed();
            if (t2 > 0.001)
            {
                return t1 + t2 + t3;
            }
            else
            {
                float maxSpeed = Mathf.Sqrt((2 * distance + initV * initV * kart.m_FinalStats.Braking + finalV * finalV * kart.m_FinalStats.Acceleration) / (kart.m_FinalStats.Acceleration + kart.m_FinalStats.Braking));
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
            newState.team = team;
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
            int timeUpdate = (int)(computeTOC(kart, distanceInSection, getAverageVelocity(), newState.getAverageVelocity()) * gameParams.timePrecision);
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
            newState.tireAge = (int)((tireAge / 10000f + tireLoad * kart.m_FinalStats.TireWearFactor) * 10000);

            return newState;
        }
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
            ordered.Sort((a, b) =>
            {
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
                    if (i == noMovePlayer || kartStates[i].team == kartStates[noMovePlayer].team)
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
                float maxScore = gameParams.timePrecision * -1000f;
                float minScore = gameParams.timePrecision * 1000f;
                List<float> scores = new List<float>();
                List<float> normalizedScores = new List<float>();
                float teamScore = 0f;
                float opponentScore = 0f;
                int teamCount = 0;
                int opponentCount = 0;
                foreach (DiscreteKartState s in kartStates)
                {
                    foreach (DiscreteKartState o in kartStates)
                    {
                        if (s.Equals(o))
                        {
                            teamScore += o.timeAtSection;
                        }
                        else if (s.team == o.team)
                        {
                            teamScore += o.timeAtSection * envController.TeamScoreRewardMultiplier;
                            teamCount += 1;
                        }
                        else
                        {
                            opponentScore += o.timeAtSection;
                            opponentCount += 1;
                        }
                    }
                    float score = opponentScore * ((teamCount * envController.TeamScoreRewardMultiplier + 1f) / (opponentCount * 1f)) - teamScore;
                    scores.Add(score);
                    maxScore = Math.Max(maxScore, score);
                    minScore = Math.Min(minScore, score);
                }
                foreach (int score in scores)
                {
                    normalizedScores.Add((score - minScore) * 1.0f / (maxScore - minScore));
                }
                return Tuple.Create(true, normalizedScores);
            }
            else
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
            for (int i = 6; i < (int)agent.m_Kart.GetMaxSpeed(); i += gameParams.velocityBucketSize)
            {
                for (int j = 1; j < 5; j++)
                {
                    possibleActions.Add(new DiscreteKartAction
                    {
                        min_velocity = i,
                        max_velocity = Math.Min(i + gameParams.velocityBucketSize, (int)agent.m_Kart.GetMaxSpeed()),
                        lane = j
                    });
                }
            }
            List<DiscreteKartAction> preCollisionFilteredActions = possibleActions.FindAll((action) =>
            {
            // UnityEngine.Debug.Log(currentState.lane + " " + action.lane + ", " + currentState.player);
            // Is Lane changing not allowed?
            if (envController.sectionIsStraight(currentState.section) && currentState.laneChanges + Math.Abs(action.lane - currentState.lane) > envController.MaxLaneChanges)
                {
                // Debug.Log("Current Lane changes " + currentState.laneChanges);
                // Debug.Log("Delta change " + (action.lane - currentState.lane));
                // Debug.Log("Max change " + envController.MaxLaneChanges);

                return false;
                }

            // Is lateral gs infeasible?
            if (!envController.sectionSpeedFeasible(currentState.section, action.max_velocity, currentState.lane, action.lane, currentState.tireAge / 10000f, agent.m_Kart))
                {
                // Debug.Log("Tire age " + currentState.tireAge);
                if (!envController.sectionIsStraight(currentState.section))
                    {
                    // Debug.Log("Going on " + currentState.section + " from lane " + currentState.lane + " to " + action.lane + " at max velocity " + action.max_velocity + " with tire age " + currentState.tireAge/10000f + " is infeasible");
                }
                    return false;
                }

            // Is changing speed infeasible?
            DiscreteKartState appliedAction = currentState.applyAction(action, envController, gameParams);
                if (appliedAction.infeasible)
                {
                // Debug.Log("Going from speed " + currentState.getAverageVelocity() + " to " + appliedAction.getAverageVelocity() + " infeasible in section " + appliedAction.section);
                return false;
                }

                return true;
            }
            ).ToList();
            if (preCollisionFilteredActions.Count == 0)
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
}
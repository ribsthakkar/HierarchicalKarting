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
    public class EndToEndKartAgent : KartAgent
    {
        protected override void FixedUpdate()
        {
            base.FixedUpdate();
        }

        protected override void Awake()
        {
            base.Awake();
            var behaviorParameters = GetComponent<BehaviorParameters>();
            var brainParameters = behaviorParameters.BrainParameters;
            /**
             * Sensors.Lenght -> RayCasts
             * SectionHorizon * 5 -> Upcoming Checkpoints (Vector3 location, target speed, isStaright)
             * 7 -> Current Player's state
             * 12 -> Other player states
             **/
            brainParameters.VectorObservationSize = Sensors.Length + (sectionHorizon * 5) + 7 + (12 * otherAgents.Length);
        }

        protected override void setLaneDifferenceDivider(int sectionIndex, int lane)
        {
            LaneDifferenceRewardDivider = 1.0f;
        }

        protected override void setVelocityDifferenceDivider(int sectionIndex, float velocity)
        {
            VelocityDifferenceRewardDivider = 1.0f;
        }

        public override void CollectObservations(VectorSensor sensor)
        {
            sensor.AddObservation(m_Kart.LocalSpeed());
            sensor.AddObservation(m_Acceleration);
            sensor.AddObservation(m_Lane);
            sensor.AddObservation(m_LaneChanges * 1f / m_envController.MaxLaneChanges);
            sensor.AddObservation(m_envController.sectionIsStraight(m_SectionIndex));
            sensor.AddObservation(m_Kart.TireWearProportion());
            sensor.AddObservation(m_SectionIndex * 1f / m_envController.goalSection);
            foreach (KartAgent agent in otherAgents)
            {
                sensor.AddObservation(agent.m_Kart.LocalSpeed());
                sensor.AddObservation(agent.m_Acceleration);
                sensor.AddObservation(agent.m_Lane);
                sensor.AddObservation(agent.m_LaneChanges * 1f / m_envController.MaxLaneChanges);
                sensor.AddObservation(agent.gameObject.activeSelf);
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
            bool hittingWall = false;
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
                    if (hitTrackInfo.distance < 0.85f)
                    {
                        m_envController.ResolveEvent(Event.HitWall, this, null);
                        hittingWall = true;
                    }
                    sensor.AddObservation(hitTrackInfo.distance);
                }
                else if (hitAgent)
                {
                    if (hitAgentInfo.distance < 1.5f)
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
            hitWall = hittingWall;
        }
        
    }


}
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

    /// <summary>
    /// The KartAgent will drive the inputs for the KartController.
    /// </summary>
    public class HierarchicalKartAgent : KartAgent
    {
        protected override void FixedUpdate()
        {
            base.FixedUpdate();
            episodeSteps += 1;
            if (episodeSteps % 100 == 0)
            {
                for (int i = m_SectionIndex + 1; i < Math.Min(m_SectionIndex + sectionHorizon, m_envController.goalSection) + 1; i++)
                {
                    m_UpcomingLanes[i % m_envController.Sections.Length] = Random.Range(1, 4);
                }
            }
        }

        protected override void Awake()
        {
            base.Awake();
            var behaviorParameters = GetComponent<BehaviorParameters>();
            var brainParameters = behaviorParameters.BrainParameters;
            brainParameters.VectorObservationSize = Sensors.Length + sectionHorizon + 5 + 8 * otherAgents.Length;
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

        public override void CollectObservations(VectorSensor sensor)
        {
            //print("collecting observations");
            // Add observation for agent state (Speed, acceleration, lane, recent lane changes, section type)
            sensor.AddObservation(m_Kart.LocalSpeed());
            sensor.AddObservation(m_Acceleration);
            sensor.AddObservation(m_Lane);
            sensor.AddObservation(m_LaneChanges);
            //print("Section Index" + m_SectionIndex);
            sensor.AddObservation(m_envController.Sections[m_SectionIndex % m_envController.Sections.Length].transform.parent.GetComponent<MeshCollider>().sharedMesh.name == "ModularTrackStraight");

            // Add observation for opponent agent states (Speed, acceleration, lane, recent lane chagnes, section type, distance, direction)
            foreach (KartAgent agent in otherAgents)
            {
                sensor.AddObservation(agent.m_Kart.LocalSpeed());
                sensor.AddObservation(agent.m_Acceleration);
                sensor.AddObservation(agent.m_Lane);
                sensor.AddObservation(agent.m_LaneChanges);
                sensor.AddObservation(agent.gameObject.activeSelf);
                sensor.AddObservation(m_envController.Sections[agent.m_SectionIndex % m_envController.Sections.Length].transform.parent.GetComponent<MeshCollider>().sharedMesh.name == "ModularTrackStraight");
                sensor.AddObservation((agent.m_Kart.transform.position - m_Kart.transform.position).magnitude);
                sensor.AddObservation(Vector3.SignedAngle(m_Kart.transform.forward, agent.m_Kart.transform.position, Vector3.up));

            }

            // Add an observation for direction of the agent to the next checkpoint and lane.
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

                    if (ShowRaycasts)
                        Debug.DrawLine(AgentSensorTransform.position, targetLaneInSection.transform.position, Color.magenta);
                }
                else
                {
                    sensor.AddObservation(3.5f);
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

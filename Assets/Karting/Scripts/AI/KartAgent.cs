﻿using KartGame.KartSystems;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using UnityEngine;
using Random = UnityEngine.Random;
using System.Collections.Generic;
using System;
using Unity.MLAgents.Actuators;
using System.Linq;

namespace KartGame.AI
{
    /// <summary>
    /// Sensors hold information such as the position of rotation of the origin of the raycast and its hit threshold
    /// to consider a "crash".
    /// </summary>
    [System.Serializable]
    public struct Sensor
    {
        public Transform Transform;
        public float RayDistance;
        public float HitValidationDistance;
    }


    /// <summary>
    /// We only want certain behaviours when the agent runs.
    /// Training would allow certain functions such as OnAgentReset() be called and execute, while Inferencing will
    /// assume that the agent will continuously run and not reset.
    /// </summary>
    public enum AgentMode
    {
        Training,
        Inferencing
    }

    /// <summary>
    /// The KartAgent will drive the inputs for the KartController.
    /// </summary>
    public class KartAgent : Agent, IInput
    {
#region Training Modes
        [Tooltip("Are we training the agent or is the agent production ready?")]
        public AgentMode Mode = AgentMode.Training;
        [Tooltip("What is the initial checkpoint the agent will go to? This value is only for inferencing.")]
        public ushort InitCheckpointIndex;


        #endregion

        #region Senses
        [Header("Observation Params")]
        [Tooltip("What objects should the raycasts hit and detect?")]
        public LayerMask Mask;
        [Tooltip("Sensors contain ray information to sense out the world, you can have as many sensors as you need.")]
        public Sensor[] Sensors;
        [Header("Opponents"), Tooltip("What are the other agents in the racing game?")]
        public KartAgent[] otherAgents;

        [Tooltip("What layer are the checkpoints on? This should be an exclusive layer for the agent to use.")]
        public LayerMask CheckpointMask;
        [Tooltip("What are the layers we want to detect for the track and the ground?")]
        public LayerMask TrackMask;
        [Tooltip("What are the layers we want to detect for other agents?")]
        public LayerMask AgentMask;


        [Space]
        [Tooltip("Would the agent need a custom transform to be able to raycast and hit the track? " +
            "If not assigned, then the root transform will be used.")]
        public Transform AgentSensorTransform;


        #endregion

        #region Rewards
        [Header("Rewards"), Tooltip("What penatly is given when the agent crashes with a wall?")]
        public float WallHitPenalty = -1f;
        [Tooltip("What penatly is given when the agent crashes into another agent?")]
        public float OpponentHitPenalty = -100f;
        [Tooltip("What penatly is given when the agent is crashed by another agent?")]
        public float HitByOpponentPenalty = -50f;
        [Tooltip("How much reward is given when the agent successfully passes the checkpoints?")]
        public float PassCheckpointReward;
        [Tooltip("Should typically be a small value, but we reward the agent for moving in the right direction.")]
        public float TowardsCheckpointReward;
        [Tooltip("Typically if the agent moves faster, we want to reward it for finishing the track quickly.")]
        public float SpeedReward;
        [Tooltip("Reward the agent when it keeps accelerating")]
        public float AccelerationReward;
        #endregion

        #region ResetParams
        [Header("Inference Reset Params")]
        [Tooltip("What is the unique mask that the agent should detect when it falls out of the track?")]
        public LayerMask OutOfBoundsMask;
        [Tooltip("How far should the ray be when casted? For larger karts - this value should be larger too.")]
        public float GroundCastDistance;
#endregion

#region Debugging
        [Header("Debug Option")] [Tooltip("Should we visualize the rays that the agent draws?")]
        public bool ShowRaycasts;
#endregion

        ArcadeKart m_Kart;
        bool m_Acceleration;
        bool m_Brake;
        float m_Steering;
        public int m_SectionIndex;
        int m_Lane;
        int m_LaneChanges;
        public bool anyHit = false;
        public int m_timeSteps = 0;
        float LaneDifferenceRewardDivider = 1.0f;
        public Dictionary<int, int> m_UpcomingLanes = new Dictionary<int, int>();
        HashSet<KartAgent> hitAgents = new HashSet<KartAgent>();

        bool m_HitOccured;
        float m_LastAccumulatedReward;
        public RacingEnvController m_envController;

        void Awake()
        {
            m_Kart = GetComponent<ArcadeKart>();
            if (AgentSensorTransform == null) AgentSensorTransform = transform;
        }
        void Start()
        {        
            if (Mode == AgentMode.Inferencing) m_SectionIndex = InitCheckpointIndex;
        }

        void FixedUpdate()
        {
            if (m_HitOccured)
            {
                m_envController.ResolveEvent(Event.HitSomething, this, hitAgents);
            }
        }

        void LateUpdate()
        {
            switch (Mode)
            {
                case AgentMode.Inferencing:
                    if (ShowRaycasts) 
                        Debug.DrawRay(transform.position, Vector3.down * GroundCastDistance, Color.cyan);

                    // We want to place the agent back on the track if the agent happens to launch itself outside of the track.
                    if (Physics.Raycast(transform.position + Vector3.up, Vector3.down, out var hit, GroundCastDistance, TrackMask)
                        && ((1 << hit.collider.gameObject.layer) & OutOfBoundsMask) > 0)
                    {
                        // Reset the agent back to its last known agent checkpoint
                        var checkpoint = m_envController.Sections[m_SectionIndex].transform;
                        transform.localRotation = checkpoint.rotation;
                        transform.position = checkpoint.position;
                        m_Kart.Rigidbody.velocity = default;
                        m_Steering = 0f;
						m_Acceleration = m_Brake = false; 
                    }

                    break;
            }
        }

        void OnTriggerEnter(Collider other)
        {
            var maskedValue = 1 << other.gameObject.layer;
            var triggered = maskedValue & CheckpointMask;

            FindSectionIndex(other, out var index, out var lane);

            // Ensure that the agent touched the checkpoint and the new index is greater than the m_CheckpointIndex.
            if (triggered > 0 && index > m_SectionIndex || index == 0 && m_SectionIndex == m_envController.Sections.Length - 1)
            {
                LaneDifferenceRewardDivider = 1.0f;
                if (lane != -1)
                {
                    //var lines = m_UpcomingLanes.Select(kvp => kvp.Key + ": " + kvp.Value.ToString());
                   // print(string.Join(",", lines));
                    //print(index);
                    LaneDifferenceRewardDivider = Math.Abs(lane - m_UpcomingLanes[index % m_envController.Sections.Length])+1.0f;
                }
                m_UpcomingLanes.Remove(index % m_envController.Sections.Length);
                if (m_UpcomingLanes.Count == 0)
                {
                    m_envController.ResolveEvent(Event.ReachGoalSection, this, null);
                } 
                else
                {
                    m_envController.ResolveEvent(Event.ReachNonGoalSection, this, null);
                }
                m_SectionIndex = index;
                m_Lane = lane;
            }
        }

        void FindSectionIndex(Collider checkPointTrigger, out int index, out int lane, int sectionHorizon = 6)
        {
            for (int i = m_SectionIndex; i < m_SectionIndex + sectionHorizon; i++)
            {
                if (m_envController.Sections[i % m_envController.Sections.Length].Trigger.GetInstanceID() == checkPointTrigger.GetInstanceID())
                {
                    index = i;
                    //print("Next Section" + index);
                    lane = m_envController.Sections[i % m_envController.Sections.Length].CalculateLane(m_Kart);
                    return;
                }
            }
            index = -1;
            lane = -1;
        }

        float Sign(float value)
        {
            if (value > 0)
            {
                return 1;
            } 
            if (value < 0)
            {
                return -1;
            }
            return 0;
        }

        public override void CollectObservations(VectorSensor sensor)
        {
            //print("collecting observations");
            // Add observation for agent state (Speed, acceleration, lane, recent lane changes)
            sensor.AddObservation(m_Kart.LocalSpeed());
            sensor.AddObservation(m_Acceleration);
            sensor.AddObservation(m_Lane);
            sensor.AddObservation(m_LaneChanges);

            // Add observation for opponent agent states (Speed, acceleration, lane, time diff to current section)


            // Add an observation for direction of the agent to the next checkpoint and lane.
            var next = (m_SectionIndex + 1) % m_envController.Sections.Length;
            var nextSection = m_envController.Sections[next];
            if (nextSection == null)
                return;

            BoxCollider targetLaneInSection = nextSection.getBoxColliderForLane(m_UpcomingLanes.ContainsKey(next) ? m_UpcomingLanes[next] : Random.Range(1, 4));
            var direction = (targetLaneInSection.transform.position - m_Kart.transform.position).normalized;
            sensor.AddObservation(Vector3.Dot(m_Kart.Rigidbody.velocity.normalized, direction));

            if (ShowRaycasts)
                Debug.DrawLine(AgentSensorTransform.position, targetLaneInSection.transform.position, Color.magenta);

            m_LastAccumulatedReward = 0.0f;
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
                        m_LastAccumulatedReward += WallHitPenalty;
                    }
                    sensor.AddObservation(hitTrackInfo.distance);
                }
                else if (hitAgent)
                {
                    if (hitAgentInfo.distance < current.HitValidationDistance)
                    {
                        m_HitOccured = true;
                        m_LastAccumulatedReward += OpponentHitPenalty;
                        hitAgents.Add(m_envController.AgentBodies[hitAgentInfo.collider.attachedRigidbody]);
                    }
                    sensor.AddObservation(hitAgentInfo.distance);
                }
                else
                {
                    sensor.AddObservation(current.RayDistance);
                }
            }
        }



        public void ApplyHitPenalty()
        {
            AddReward(WallHitPenalty);
            m_LastAccumulatedReward = 0.0f;
        }

        public void ApplySectionReward()
        {
            AddReward(PassCheckpointReward/LaneDifferenceRewardDivider);
        }

        public void Deactivate()
        {
            gameObject.SetActive(false);
        }

        public void Activate()
        {
            m_LastAccumulatedReward = 0.0f;
            m_timeSteps = 0;
            m_HitOccured = false;
            gameObject.SetActive(true);
        }

        public override void OnActionReceived(ActionBuffers actions)
        {
            base.OnActionReceived(actions);
            InterpretDiscreteActions(actions);

            // Find the next checkpoint when registering the current checkpoint that the agent has passed.
            var next = (m_SectionIndex + 1) % m_envController.Sections.Length;
            var nextCollider = m_envController.Sections[next].getBoxColliderForLane(m_UpcomingLanes.ContainsKey(next) ? m_UpcomingLanes[next] : Random.Range(1, 4));
            var direction = (nextCollider.transform.position - m_Kart.transform.position).normalized;
            var reward = Vector3.Dot(m_Kart.Rigidbody.velocity.normalized, direction);

            if (ShowRaycasts) Debug.DrawRay(AgentSensorTransform.position, m_Kart.Rigidbody.velocity, Color.blue);

            // Add rewards if the agent is heading in the right direction
            AddReward(reward * TowardsCheckpointReward);
            AddReward((m_Acceleration && !m_Brake ? 1.0f : 0.0f) * AccelerationReward);
            AddReward(m_Kart.LocalSpeed() * SpeedReward);
        }
        public override void Heuristic(in ActionBuffers actionsOut)
        {
            base.Heuristic(actionsOut);
            ActionSegment<float> continuousActions = actionsOut.ContinuousActions;
            ActionSegment<int> discreteActions = actionsOut.DiscreteActions;
            continuousActions[0] = Input.GetAxisRaw("Horizontal");
            if (Input.GetButton("Accelerate") && Input.GetButton("Brake"))
            {
                discreteActions[0] = 0;
            } else if (Input.GetButton("Accelerate"))
            {
                discreteActions[0] = 1;
            } else  if (Input.GetButton("Brake"))
            {
                discreteActions[0] = -1;
            }

        }
        public override void OnEpisodeBegin()
        {
            base.OnEpisodeBegin();
            switch (Mode)
            {
                case AgentMode.Training:
                    m_Kart.Rigidbody.velocity = default;
                    m_Acceleration = false;
                    m_Brake = false;
                    m_Steering = 0f;
                    break;
                default:
                    break;
            }
        }

        void InterpretDiscreteActions(ActionBuffers actions)
        {
            m_Steering = actions.ContinuousActions[0];
            m_Acceleration = actions.DiscreteActions[0] >= 1;
            m_Brake = actions.DiscreteActions[0] < 1;
        }

        public InputData GenerateInput()
        {
            return new InputData
            {
                Accelerate = m_Acceleration,
                Brake = m_Brake,
                TurnInput = m_Steering
            };
        }
    }
}

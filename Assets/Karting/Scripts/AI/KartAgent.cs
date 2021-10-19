using KartGame.KartSystems;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using UnityEngine;
using Random = UnityEngine.Random;
using System.Collections.Generic;
using System;
using Unity.MLAgents.Actuators;

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
        [Header("Checkpoints"), Tooltip("What are the series of checkpoints for the agent to seek and pass through?")]
        public DiscretePositionTracker[] Sections;
        [Tooltip("What layer are the checkpoints on? This should be an exclusive layer for the agent to use.")]
        public LayerMask CheckpointMask;

        [Space]
        [Tooltip("Would the agent need a custom transform to be able to raycast and hit the track? " +
            "If not assigned, then the root transform will be used.")]
        public Transform AgentSensorTransform;
#endregion

#region Rewards
        [Header("Rewards"), Tooltip("What penatly is given when the agent crashes?")]
        public float HitPenalty = -1f;
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
        [Tooltip("What are the layers we want to detect for the track and the ground?")]
        public LayerMask TrackMask;
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
        int m_SectionIndex;
        int m_Lane;
        Dictionary<int, int> m_UpcomingLanes;

        bool m_EndEpisode;
        float m_LastAccumulatedReward;

        void Awake()
        {
            m_Kart = GetComponent<ArcadeKart>();
            if (AgentSensorTransform == null) AgentSensorTransform = transform;
        }

        void Start()
        {
            // If the agent is training, then at the start of the simulation, pick a random checkpoint to train the agent.
            OnEpisodeBegin();

            if (Mode == AgentMode.Inferencing) m_SectionIndex = InitCheckpointIndex;
            m_UpcomingLanes = new Dictionary<int, int>();
            m_UpcomingLanes[0] = 2;
            m_UpcomingLanes[1] = 2;
            m_UpcomingLanes[2] = 2;
            m_UpcomingLanes[3] = 2;
            m_UpcomingLanes[4] = 2;
            m_UpcomingLanes[5] = 2;
            m_UpcomingLanes[6] = 2;
            m_UpcomingLanes[7] = 2;
            m_UpcomingLanes[8] = 2;
            m_UpcomingLanes[9] = 2;
            m_UpcomingLanes[10] = 2;
            m_UpcomingLanes[11] = 2;
            m_UpcomingLanes[12] = 2;
            m_UpcomingLanes[13] = 2;
            m_UpcomingLanes[14] = 2;
            m_UpcomingLanes[15] = 2;
            m_UpcomingLanes[16] = 2;
            m_UpcomingLanes[17] = 2;
            m_UpcomingLanes[18] = 2;
            m_UpcomingLanes[19] = 2;
            m_UpcomingLanes[20] = 2;
            m_UpcomingLanes[21] = 2;
            m_UpcomingLanes[22] = 2;
            m_UpcomingLanes[23] = 2;
        }

        void Update()
        {
            if (m_EndEpisode)
            {
                m_EndEpisode = false;
                AddReward(m_LastAccumulatedReward);
                EndEpisode();
                OnEpisodeBegin();
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
                        var checkpoint = Sections[m_SectionIndex].transform;
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
            if (triggered > 0 && index > m_SectionIndex || index == 0 && m_SectionIndex == Sections.Length - 1)
            {
                float LaneDifferenceDivider = 1.0f;
                if (lane != -1)
                {
                    LaneDifferenceDivider = Math.Abs(lane - m_UpcomingLanes[index % Sections.Length])*1.0f;
                }
                AddReward(PassCheckpointReward/LaneDifferenceDivider);
                m_SectionIndex = index;
                m_Lane = lane;
            }
        }

        void FindSectionIndex(Collider checkPointTrigger, out int index, out int lane, int sectionHorizon = 6)
        {
            for (int i = m_SectionIndex; i < m_SectionIndex + sectionHorizon; i++)
            {
                if (Sections[i % Sections.Length].Trigger.GetInstanceID() == checkPointTrigger.GetInstanceID())
                {
                    index = i;
                    print("Next Section" + index);
                    lane = Sections[i % Sections.Length].CalculateLane(m_Kart);
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
            sensor.AddObservation(m_Kart.LocalSpeed());

            // Add an observation for direction of the agent to the next checkpoint and lane.
            var next = (m_SectionIndex + 1) % Sections.Length;
            var nextSection = Sections[next];
            if (nextSection == null)
                return;
            //print(next);
            //print(nextSection);
            // print(m_UpcomingLanes[next]);
            BoxCollider targetLaneInSection = nextSection.getBoxColliderForLane(m_UpcomingLanes[next]);
            var direction = (targetLaneInSection.transform.position - m_Kart.transform.position).normalized;
            sensor.AddObservation(Vector3.Dot(m_Kart.Rigidbody.velocity.normalized, direction));

            if (ShowRaycasts)
                Debug.DrawLine(AgentSensorTransform.position, targetLaneInSection.transform.position, Color.magenta);

            m_LastAccumulatedReward = 0.0f;
            m_EndEpisode = false;
            for (var i = 0; i < Sensors.Length; i++)
            {
                var current = Sensors[i];
                var xform = current.Transform;
                var hit = Physics.Raycast(AgentSensorTransform.position, xform.forward, out var hitInfo,
                    current.RayDistance, Mask, QueryTriggerInteraction.Ignore);

                if (ShowRaycasts)
                {
                    Debug.DrawRay(AgentSensorTransform.position, xform.forward * current.RayDistance, Color.green);
                    Debug.DrawRay(AgentSensorTransform.position, xform.forward * current.HitValidationDistance, 
                        Color.red);

                    if (hit && hitInfo.distance < current.HitValidationDistance)
                    {
                        Debug.DrawRay(hitInfo.point, Vector3.up * 3.0f, Color.blue);
                    }
                }

                if (hit)
                {
                    if (hitInfo.distance < current.HitValidationDistance)
                    {
                        m_LastAccumulatedReward += HitPenalty;
                        m_EndEpisode = true;
                    }
                }

                sensor.AddObservation(hit ? hitInfo.distance : current.RayDistance);
            }

            sensor.AddObservation(m_Acceleration);
            sensor.AddObservation(m_Lane);
        }

        public override void OnActionReceived(ActionBuffers actions)
        {
            base.OnActionReceived(actions);
            InterpretDiscreteActions(actions);

            // Find the next checkpoint when registering the current checkpoint that the agent has passed.
            var next = (m_SectionIndex + 1) % Sections.Length;
            var nextCollider = Sections[next].getBoxColliderForLane(m_UpcomingLanes[next]);
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
            switch (Mode)
            {
                case AgentMode.Training:
                    m_SectionIndex = Random.Range(0, Sections.Length - 1);
                    var collider = Sections[m_SectionIndex].Trigger;
                    transform.localRotation = collider.transform.rotation;
                    transform.position = collider.transform.position;
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

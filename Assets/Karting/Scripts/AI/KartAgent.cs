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
        [Tooltip("How many sections/checkpoints ahead does the agent expect to reach between a pair of actions/observations?")]
        public int sectionHorizon;

        [Space]
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

        #region TrainingEnvironment
        [Header("Traning Environment"), Tooltip("What training enviornment controller to use?")]
        public RacingEnvController m_envController;
        #endregion

        #region Rewards
        [Header("Rewards"), Tooltip("What penatly is given when the agent crashes with a wall?")]
        public float WallHitPenalty = -1f;
        [Tooltip("What penatly is given when the agent crashes into another agent?")]
        public float OpponentHitPenalty = -10f;
        [Tooltip("What penatly is given when the agent is crashed by another agent?")]
        public float HitByOpponentPenalty = -5f;
        [Tooltip("How much reward is given when the agent successfully passes the checkpoints?")]
        public float PassCheckpointReward = 0.1f;
        [Tooltip("How much the normalized remaining time is multiplied by to provide as a reward for the agent reaching the goal checkpoint?")]
        public float ReachGoalCheckpointRewardMultplier = 5.0f;
        [Tooltip("How much reward base reward is given to the agent for reaching the goal checkpoint?")]
        public float ReachGoalCheckpointRewardBase = 1.0f;
        [Tooltip("Should typically be a small value, but we reward the agent for moving in the right direction.")]
        public float TowardsCheckpointReward = 0.003f;
        [Tooltip("Typically if the agent moves faster, we want to reward it for finishing the track quickly.")]
        public float SpeedReward = 0.002f;
        [Tooltip("Reward the agent when it keeps accelerating")]
        public float AccelerationReward = 0.0f;
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

        [HideInInspector]
        public ArcadeKart m_Kart;
        [HideInInspector]
        public bool m_Acceleration;
        [HideInInspector]
        public bool m_Brake;
        [HideInInspector]
        public float m_Steering;
        [HideInInspector] public int m_SectionIndex;
        [HideInInspector] public int m_Lane;
        [HideInInspector] public int m_LaneChanges;
        [HideInInspector] public bool anyHit = false;
        [HideInInspector] public int m_timeSteps = 0;
        [HideInInspector] public float LaneDifferenceRewardDivider = 1.0f;
        [HideInInspector] public Dictionary<int, int> m_UpcomingLanes = new Dictionary<int, int>();
        [HideInInspector] public HashSet<KartAgent> hitAgents = new HashSet<KartAgent>();
        [HideInInspector] public bool m_HitOccured;
        [HideInInspector] public float m_LastAccumulatedReward;



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

        protected virtual void Awake()
        {
            m_Kart = GetComponent<ArcadeKart>();
            if (AgentSensorTransform == null) AgentSensorTransform = transform;
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

        protected virtual void setLaneDifferenceDivider(int sectionIndex, int lane)
        {
            LaneDifferenceRewardDivider = 1.0f;
        }

        void OnTriggerEnter(Collider other)
        {
            var maskedValue = 1 << other.gameObject.layer;
            var triggered = maskedValue & CheckpointMask;

            FindSectionIndex(other, out var index, out var lane);

            // Ensure that the agent touched the checkpoint and the new index is greater than the m_CheckpointIndex.
            if (triggered > 0 && index > m_SectionIndex || index == 0 && m_SectionIndex == m_envController.Sections.Length - 1)
            {
                setLaneDifferenceDivider(index, lane);
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

        void FindSectionIndex(Collider checkPointTrigger, out int index, out int lane)
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

        public void ApplyHitPenalty()
        {
            AddReward(m_LastAccumulatedReward);
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

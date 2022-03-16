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
using CenterSpace.NMath.Core;

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
        public float WallHitValidationDistance;
        public float AgentHitValidationDistance;
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
        public int InitCheckpointIndex;


        #endregion

        #region Senses
        [Header("Observation Params")]
        [Tooltip("What objects should the raycasts hit and detect?")]
        public LayerMask Mask;
        [Tooltip("Sensors contain ray information to sense out the world, you can have as many sensors as you need.")]
        public Sensor[] Sensors;

        [Header("Racers")]
        [Tooltip("Who are the racers on your team in the game?")]
        public KartAgent[] teamAgents;
        [Tooltip("Who are the opposing agents in the racing game?")]
        public KartAgent[] otherAgents;

        [HideInInspector] protected int sectionHorizon;

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

        #region StateVariables
        [HideInInspector] public ArcadeKart m_Kart;
        [HideInInspector] public bool m_Acceleration;
        [HideInInspector] public bool m_Brake;
        [HideInInspector] public float m_Steering;
        [HideInInspector] public int m_SectionIndex;
        [HideInInspector] public int m_Lane;
        [HideInInspector] public int m_LaneChanges;
        [HideInInspector] public int m_IllegalLaneChanges;
        [HideInInspector] public bool anyHit = false;
        [HideInInspector] public int m_timeSteps = 0;
        [HideInInspector] public float LaneDifferenceRewardDivider = 1.0f;
        [HideInInspector] public float VelocityDifferenceRewardDivider = 1.0f;
        [HideInInspector] public float AverageVelDifference;
        [HideInInspector] public float AverageLaneDifference;
        [HideInInspector] public Dictionary<int, int> m_UpcomingLanes = new Dictionary<int, int>();
        [HideInInspector] public Dictionary<int, float> m_UpcomingVelocities = new Dictionary<int, float>();
        [HideInInspector] public HashSet<KartAgent> hitAgents = new HashSet<KartAgent>();
        [HideInInspector] public bool m_HitOccured;
        [HideInInspector] public float m_LastAccumulatedReward;
        [HideInInspector] protected int episodeSteps = 0;
        [HideInInspector] public bool is_active = true;
        [HideInInspector] public Dictionary<int, int> sectionTimes = new Dictionary<int, int>();
        [HideInInspector] public bool forwardCollision = false;
        [HideInInspector] public int forwardCollisions = 0;
        [HideInInspector] public int lastCollisionTime = 0;
        #endregion

        public virtual void Start()
        {        
            if (Mode == AgentMode.Inferencing) m_SectionIndex = InitCheckpointIndex;
        }

        protected virtual void FixedUpdate()
        {
            episodeSteps += 1;
            var hitAgent = Physics.Raycast(AgentSensorTransform.position, Sensors[0].Transform.forward, out var hitAgentInfo,
                0.8f, AgentMask, QueryTriggerInteraction.Ignore) |
                Physics.Raycast(AgentSensorTransform.position, Sensors[1].Transform.forward, out var hitAgentInfo2,
                0.8f, AgentMask, QueryTriggerInteraction.Ignore) |
                Physics.Raycast(AgentSensorTransform.position, Sensors[5].Transform.forward, out var hitAgentInfo3,
                0.8f, AgentMask, QueryTriggerInteraction.Ignore);

            // Discretely Count collisions by check if consecutive hits to other agents are greater than 1.5 seconds apart
            if (hitAgent && !forwardCollision && (lastCollisionTime == 0 || episodeSteps - lastCollisionTime > 75))
            {
                forwardCollision = true;
                forwardCollisions += 1;
                lastCollisionTime = episodeSteps;
            }
            else if (hitAgent)
            {
                forwardCollision = true;
                lastCollisionTime = episodeSteps;
            }
            else
            {
                forwardCollision = false;
            }
            if (is_active || m_SectionIndex != m_envController.goalSection)
                AddReward(m_envController.NotAtGoalPenalty);
        }

        protected virtual void Awake()
        {
            m_Kart = GetComponent<ArcadeKart>();
            if (AgentSensorTransform == null) AgentSensorTransform = transform;
            sectionHorizon = m_envController.sectionHorizon;
        }

        void LateUpdate()
        {
            if (ShowRaycasts) 
                Debug.DrawRay(transform.position, Vector3.down * GroundCastDistance, Color.cyan);
            var inAir = !Physics.Raycast(transform.position, Vector3.down, out var hit, 5, TrackMask);
            // We want to place the agent back on the track if the agent happens to launch itself outside of the track.
            var dist2Track = (m_Kart.transform.position - m_envController.Sections[m_SectionIndex % m_envController.Sections.Length].transform.position).magnitude;
            if (inAir && dist2Track > 25)
            {
                // print(agent.name + " is falling");
                switch(Mode)
                {
                    case AgentMode.Training:
                        m_envController.ResolveEvent(Event.FellOffWorld, this, null);
                        break;
                    case AgentMode.Inferencing:
                        // Reset the agent back to its last known agent checkpoint
                        var checkpoint = m_envController.Sections[m_SectionIndex % m_envController.Sections.Length].transform;
                        transform.localRotation = checkpoint.rotation;
                        transform.position = checkpoint.position;
                        m_Kart.Rigidbody.velocity = Vector3.zero;
                        m_Steering = 0f;
				        m_Acceleration = m_Brake = false;
                        break;
                }

            }

        }

        /**
         * Reset some parameters for re-using this game object for training purposes or resetting the environment
         **/
        public virtual void prepareForReuse()
        {
            // Reset some parameters when re-activating this game object
            sectionTimes[m_SectionIndex] = 0;
            m_Kart.UpdateStats();
            forwardCollisions = 0;
            forwardCollision = false;
            lastCollisionTime = 0;
            AverageVelDifference = 0f;
            AverageLaneDifference = 0f;
            m_UpcomingLanes = new Dictionary<int, int>();
            m_UpcomingVelocities = new Dictionary<int, float>();
        }

        /**
         * Update average lane difference state. Used for metrics to compare implemented hierarchical agents.
         **/
        public void UpdateLaneDifferenceCalculation(int sectionIndex, int lane)
        {
            if (m_UpcomingLanes.ContainsKey(sectionIndex % m_envController.Sections.Length))
                AverageLaneDifference = (Mathf.Max((m_Kart.transform.position - m_envController.Sections[sectionIndex % m_envController.Sections.Length].getBoxColliderForLane(m_UpcomingLanes[sectionIndex % m_envController.Sections.Length]).transform.position).magnitude-1.3f, 0f) + AverageLaneDifference * (sectionIndex - InitCheckpointIndex - 1)) / (sectionIndex - InitCheckpointIndex);
        }

        /**
         * Update average velocity difference state. Used for metrics to compare implemented hierarchical agents.
         **/
        public void UpdateVelocityDifferenceCalculation(int sectionIndex, float velocity)
        {
            if (m_UpcomingVelocities.ContainsKey(sectionIndex % m_envController.Sections.Length))
                AverageVelDifference = ((velocity - m_UpcomingVelocities[sectionIndex % m_envController.Sections.Length]) + AverageVelDifference * (sectionIndex - InitCheckpointIndex - 1)) / (sectionIndex - InitCheckpointIndex);
        }


        /**
         * Set the reward divider for RL agents based on lane difference
        **/
        protected virtual void setLaneDifferenceDivider(int sectionIndex, int lane)
        {
            LaneDifferenceRewardDivider = 1.0f;
        }

        /**
         * Set the reward divider for RL agents based on velocity difference
        **/
        protected virtual void setVelocityDifferenceDivider(int sectionIndex, float velocity)
        {
            VelocityDifferenceRewardDivider = 1.0f;
        }

        /**
         * Set random target velocity and lanes. Used for generating plans during RL training.
        **/
        public virtual void planRandomly()
        {
            for (int i = m_SectionIndex + 1; i < Math.Min(m_SectionIndex + sectionHorizon,1000) + 1; i++)
            {
                if (!m_UpcomingLanes.ContainsKey(i % m_envController.Sections.Length))
                {
                    m_UpcomingLanes[i % m_envController.Sections.Length] = Random.Range(1, 4);
                    m_UpcomingVelocities[i % m_envController.Sections.Length] = Random.Range(5f, m_Kart.GetMaxSpeed());
                }
            }
        }

        /**
         * Calculate initial plan for agent for waking up the GameObject
        **/
        public virtual void initialPlan()
        {
            planRandomly();
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
                // If section is in plan, then calculate errors and set reward dividers
                if (m_UpcomingLanes.ContainsKey(index % m_envController.Sections.Length))
                {
                    setLaneDifferenceDivider(index, lane);
                    setVelocityDifferenceDivider(index, m_Kart.Rigidbody.velocity.magnitude);
                    UpdateLaneDifferenceCalculation(index, lane);
                    UpdateVelocityDifferenceCalculation(index, m_Kart.Rigidbody.velocity.magnitude);
                    m_UpcomingLanes.Remove(index % m_envController.Sections.Length);
                    m_UpcomingVelocities.Remove(index % m_envController.Sections.Length);
                }
                if (name.Equals(m_envController.Agents[0].name) && m_envController.highlightWaypoints)
                    m_envController.Sections[index % m_envController.Sections.Length].resetColors();
                if (m_LaneChanges + Math.Abs(m_Lane-lane) > m_envController.MaxLaneChanges && m_envController.sectionIsStraight(m_SectionIndex))
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

            }
            else if ((triggered > 0 && index !=-1) && ((index < m_SectionIndex) || (m_SectionIndex % m_envController.Sections.Length == 0 && index % m_envController.Sections.Length == m_envController.Sections.Length - 1)))
            {
                // print("going backwards");
                AddReward(m_envController.ReversePenalty * (m_SectionIndex - index + 1));
                m_SectionIndex = index;
            } else if ((triggered > 0 && index == -1))
            {
                m_envController.ResolveEvent(Event.DroveReverseLimit, this, null);
            }

        }

        protected void FindSectionIndex(Collider checkPointTrigger, out int index, out int lane)
        {
            for (int i = Math.Max(m_SectionIndex - sectionHorizon, InitCheckpointIndex); i < m_SectionIndex + sectionHorizon; i++)
            {
                int idx = i < 0 ? i + m_envController.Sections.Length : i;
                if (m_envController.Sections[idx % m_envController.Sections.Length].Trigger.GetInstanceID() == checkPointTrigger.GetInstanceID())
                {
                    index = idx;
                    //print("Next Section" + index);
                    lane = m_envController.Sections[idx % m_envController.Sections.Length].CalculateLane(m_Kart);
                    return;
                }
            }
            index = -1;
            lane = -1;
            // print("Original Section " + m_SectionIndex + ", New Section: " + index);
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

        #region Penalties/Rewards for RL agents
        public void ApplyHitWallPenalty(float factor=1f)
        {
            AddReward(factor*m_envController.WallHitPenalty);
        }

        public void ApplyHitOpponentPenalty(float factor=1f)
        {
            AddReward(factor*m_envController.OpponentHitPenalty);
        }

        public void ApplyHitByOpponentPenalty(float factor=1f)
        {
            AddReward(factor*m_envController.HitByOpponentPenalty);
        }

        public void ApplySectionReward()
        {
            AddReward(m_envController.PassCheckpointLaneReward /LaneDifferenceRewardDivider);
            AddReward(m_envController.PassCheckpointVelocityReward / VelocityDifferenceRewardDivider);
        }
        #endregion

        /**
         * Gracefully disable the gameobject when reaching finishline or driving too far in reverse
        **/
        public void Deactivate(bool disable=true)
        {
            SetZeroInputs();
            m_Kart.Rigidbody.velocity = Vector3.zero;
            m_Kart.Rigidbody.angularVelocity = Vector3.zero;
            m_Kart.Rigidbody.constraints = RigidbodyConstraints.FreezeAll;
            m_Kart.SetCanMove(false);
            if (disable)
                gameObject.SetActive(false);
            gameObject.transform.localScale = Vector3.one*0.001f;
            is_active = false;
        }

        /**
         * Re-enable gameObject and reset some parameters when re-activating it 
        **/
        public void Activate()
        {
            m_LastAccumulatedReward = 0.0f;
            m_timeSteps = 0;
            m_HitOccured = false;
            m_Kart.Rigidbody.freezeRotation = false;
            gameObject.SetActive(true);
            gameObject.transform.localScale = Vector3.one;
            m_Kart.Rigidbody.velocity = Vector3.zero;
            m_Kart.Rigidbody.angularVelocity = Vector3.zero;
            // m_Kart.Rigidbody.WakeUp();
            SetZeroInputs();
            is_active = true;
            // m_Kart.SetCanMove(true);
        }

        /**
         * Process actions received from the Agent Brain for ML Agents
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
            var nextCollider = m_UpcomingLanes.ContainsKey(next) ? m_envController.Sections[next].getBoxColliderForLane(m_UpcomingLanes[next]) : m_envController.Sections[next].Trigger;
            var direction = (nextCollider.transform.position - m_Kart.transform.position).normalized;
            var reward = Vector3.Dot(m_Kart.Rigidbody.velocity.normalized, direction);

            // if (ShowRaycasts) Debug.DrawRay(AgentSensorTransform.position, m_Kart.Rigidbody.velocity, Color.blue);

            // Add rewards if the agent is heading in the right direction
            var speedProportion = 0.00f;
            AddReward(reward * m_envController.TowardsCheckpointReward);
            AddReward((m_Acceleration && !m_Brake ? 1.0f : 0.0f) * m_envController.AccelerationReward);
            if (speedProportion > 0.01f && m_Kart.LocalSpeed() < speedProportion)
            {
                AddReward(m_envController.SlowMovingPenalty + (-m_envController.SlowMovingPenalty) * (m_Kart.LocalSpeed()/speedProportion));
            }
            else
            {
                AddReward((m_Kart.LocalSpeed()- speedProportion) /(1- speedProportion) * m_envController.SpeedReward);
            }
        }

        public virtual void InterpretDiscreteActions(ActionBuffers actions)
        {
            // print("Here ida");
            m_Steering = actions.ContinuousActions[0];
            m_Acceleration = actions.DiscreteActions[0] > 1;
            m_Brake = actions.DiscreteActions[0] < 1;
        }

        public void SetZeroInputs()
        {
            // print("Here ida");
            m_Steering = 0f;
            m_Acceleration = false;
            m_Brake = false;
        }

        public virtual InputData GenerateInput()
        {
            if (!is_active)
                return new InputData
                {
                    Accelerate = false,
                    Brake = false,
                    TurnInput = 0
                };
            return new InputData
            {
                Accelerate = m_Acceleration,
                Brake = m_Brake,
                TurnInput = m_Steering
            };
        }

        /**
         * Provide Heuristic function for ML agents to test that the environment is setup correctly using human input 
        **/
        public override void Heuristic(in ActionBuffers actionsOut)
        {
            ActionSegment<float> continuousActions = actionsOut.ContinuousActions;
            ActionSegment<int> discreteActions = actionsOut.DiscreteActions;
            continuousActions[0] = Input.GetAxisRaw("Horizontal");
            if (Input.GetButton("Accelerate") && Input.GetButton("Brake"))
            {
                discreteActions[0] = 1;
            } else if (Input.GetButton("Accelerate"))
            {
                discreteActions[0] = 2;
            } else  if (Input.GetButton("Brake"))
            {
                discreteActions[0] = 0;
            }
            else
            {
                discreteActions[0] = 1;
            }

        }


        public override void OnEpisodeBegin()
        {
            base.OnEpisodeBegin();
        }

    }
}

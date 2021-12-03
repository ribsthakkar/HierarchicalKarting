using System;
using UnityEngine;
using System.Collections.Generic;
using UnityEngine.VFX;

namespace KartGame.KartSystems
{
    public class ArcadeKart : MonoBehaviour
    {
        [System.Serializable]
        public class StatPowerup
        {
            public ArcadeKart.Stats modifiers;
            public string PowerUpID;
            public float ElapsedTime;
            public float MaxTime;
        }

        [System.Serializable]
        public struct Stats
        {
            [Header("Movement Settings")]
            [Min(0.001f), Tooltip("Top speed attainable when moving forward.")]
            public float TopSpeed;

            [Tooltip("How quickly the kart reaches top speed.")]
            public float Acceleration;

            [Min(0.001f), Tooltip("Top speed attainable when moving backward.")]
            public float ReverseSpeed;

            [Tooltip("How quickly the kart reaches top speed, when moving backward.")]
            public float ReverseAcceleration;

            [Tooltip("How quickly the kart starts accelerating from 0. A higher number means it accelerates faster sooner.")]
            [Range(0.2f, 1)]
            public float AccelerationCurve;

            [Tooltip("How quickly the kart slows down when the brake is applied.")]
            public float Braking;

            [Tooltip("How quickly the kart will reach a full stop when no inputs are made.")]
            public float CoastingDrag;

            [Range(0.0f, 1.0f)]
            [Tooltip("The amount of side-to-side friction.")]
            public float Grip;

            [Tooltip("How tightly the kart can turn left or right.")]
            public float Steer;

            [Tooltip("Max of how tightly the kart can turn left or right.")]
            public float MaxSteer;

            [Tooltip("Min of how tightly the kart can turn left or right.")]
            public float MinSteer;

            [Tooltip("Coeffecient of tire wear for this car")]
            public float TireWearFactor;

            [Tooltip("Minimum Lateral Gs the car will always be able to sustain")]
            public float MinGs;

            [Tooltip("Maximum Lateral Gs of the tire performance")]
            public float MaxGs;

            [Tooltip("Additional gravity for when the kart is in the air.")]
            public float AddedGravity;

            // allow for stat adding for powerups.
            public static Stats operator +(Stats a, Stats b)
            {
                return new Stats
                {
                    Acceleration = a.Acceleration + b.Acceleration,
                    AccelerationCurve = a.AccelerationCurve + b.AccelerationCurve,
                    Braking = a.Braking + b.Braking,
                    CoastingDrag = a.CoastingDrag + b.CoastingDrag,
                    AddedGravity = a.AddedGravity + b.AddedGravity,
                    Grip = a.Grip + b.Grip,
                    ReverseAcceleration = a.ReverseAcceleration + b.ReverseAcceleration,
                    ReverseSpeed = a.ReverseSpeed + b.ReverseSpeed,
                    TopSpeed = a.TopSpeed + b.TopSpeed,
                    MaxSteer = a.MaxSteer + b.MaxSteer,
                    MinSteer = a.MinSteer + b.MinSteer,
                    Steer = a.Steer + b.Steer,
                    TireWearFactor = a.TireWearFactor + b.TireWearFactor,
                    MinGs = a.MinGs + b.MinGs,
                    MaxGs = a.MaxGs + b.MaxGs
                };
            }
        }

        public Rigidbody Rigidbody { get; private set; }
        public InputData Input     { get; private set; }
        public float AirPercent    { get; private set; }
        public float GroundPercent { get; private set; }

        public ArcadeKart.Stats baseStats = new ArcadeKart.Stats
        {
            TopSpeed            = 10f,
            Acceleration        = 5f,
            AccelerationCurve   = 4f,
            Braking             = 10f,
            ReverseAcceleration = 5f,
            ReverseSpeed        = 5f,
            Steer               = 5f,
            MaxSteer            = 5f,
            MinSteer            = 1f,
            CoastingDrag        = 4f,
            Grip                = .95f,
            AddedGravity        = 1f,
            TireWearFactor      = 0.0001f,
            MinGs               = 0.5f,
            MaxGs               = 1.5f
        };

        [Header("Vehicle Visual")] 
        public List<GameObject> m_VisualWheels;

        [Header("Vehicle Physics")]
        [Tooltip("The transform that determines the position of the kart's mass.")]
        public Transform CenterOfMass;

        [Range(0.0f, 20.0f), Tooltip("Coefficient used to reorient the kart in the air. The higher the number, the faster the kart will readjust itself along the horizontal plane.")]
        public float AirborneReorientationCoefficient = 3.0f;

        [Header("Drifting")]
        [Range(0.01f, 1.0f), Tooltip("The grip value when drifting.")]
        public float DriftGrip = 0.4f;
        [Range(0.0f, 10.0f), Tooltip("Additional steer when the kart is drifting.")]
        public float DriftAdditionalSteer = 5.0f;
        [Range(1.0f, 30.0f), Tooltip("The higher the angle, the easier it is to regain full grip.")]
        public float MinAngleToFinishDrift = 10.0f;
        [Range(0.01f, 0.99f), Tooltip("Mininum speed percentage to switch back to full grip.")]
        public float MinSpeedPercentToFinishDrift = 0.5f;
        [Range(1.0f, 20.0f), Tooltip("The higher the value, the easier it is to control the drift steering.")]
        public float DriftControl = 10.0f;
        [Range(0.0f, 20.0f), Tooltip("The lower the value, the longer the drift will last without trying to control it by steering.")]
        public float DriftDampening = 10.0f;

        [Header("VFX")]
        [Tooltip("VFX that will be placed on the wheels when drifting.")]
        public ParticleSystem DriftSparkVFX;
        [Range(0.0f, 0.2f), Tooltip("Offset to displace the VFX to the side.")]
        public float DriftSparkHorizontalOffset = 0.1f;
        [Range(0.0f, 90.0f), Tooltip("Angle to rotate the VFX.")]
        public float DriftSparkRotation = 17.0f;
        [Tooltip("VFX that will be placed on the wheels when drifting.")]
        public GameObject DriftTrailPrefab;
        [Range(-0.1f, 0.1f), Tooltip("Vertical to move the trails up or down and ensure they are above the ground.")]
        public float DriftTrailVerticalOffset;
        [Tooltip("VFX that will spawn upon landing, after a jump.")]
        public GameObject JumpVFX;
        [Tooltip("VFX that is spawn on the nozzles of the kart.")]
        public GameObject NozzleVFX;
        [Tooltip("List of the kart's nozzles.")]
        public List<Transform> Nozzles;

        [Header("Suspensions")]
        [Tooltip("The maximum extension possible between the kart's body and the wheels.")]
        [Range(0.0f, 1.0f)]
        public float SuspensionHeight = 0.2f;
        [Range(10.0f, 100000.0f), Tooltip("The higher the value, the stiffer the suspension will be.")]
        public float SuspensionSpring = 20000.0f;
        [Range(0.0f, 5000.0f), Tooltip("The higher the value, the faster the kart will stabilize itself.")]
        public float SuspensionDamp = 500.0f;
        [Tooltip("Vertical offset to adjust the position of the wheels relative to the kart's body.")]
        [Range(-1.0f, 1.0f)]
        public float WheelsPositionVerticalOffset = 0.0f;

        [Header("Physical Wheels")]
        [Tooltip("The physical representations of the Kart's wheels.")]
        public WheelCollider FrontLeftWheel;
        public WheelCollider FrontRightWheel;
        public WheelCollider RearLeftWheel;
        public WheelCollider RearRightWheel;

        [Tooltip("Which layers the wheels will detect.")]
        public LayerMask GroundLayers = Physics.DefaultRaycastLayers;

        // the input sources that can control the kart
        IInput[] m_Inputs;

        const float k_NullInput = 0.01f;
        const float k_NullSpeed = 0.01f;
        Vector3 m_VerticalReference = Vector3.up;

        float m_CurrentGrip = 1.0f;
        [HideInInspector] public float m_AccumulatedAngularV = 0.0f;
        [HideInInspector] public float TireWearRate = 10000.0f;
        float m_PreviousGroundPercent = 1.0f;

        // can the kart move?
        bool m_CanMove = true;
        List<StatPowerup> m_ActivePowerupList = new List<StatPowerup>();
        [HideInInspector] public ArcadeKart.Stats m_FinalStats;

        Quaternion m_LastValidRotation;
        Vector3 m_LastValidPosition;
        Vector3 m_LastCollisionNormal;
        bool m_HasCollision;
        bool m_InAir = false;

        [HideInInspector] public Vector3 acc = Vector3.zero;


        public void AddPowerup(StatPowerup statPowerup) => m_ActivePowerupList.Add(statPowerup);
        public void SetCanMove(bool move) => m_CanMove = move;
        public float GetMaxSpeed() => Mathf.Max(m_FinalStats.TopSpeed, m_FinalStats.ReverseSpeed);

        void UpdateSuspensionParams(WheelCollider wheel)
        {
            wheel.suspensionDistance = SuspensionHeight;
            wheel.center = new Vector3(0.0f, WheelsPositionVerticalOffset, 0.0f);
            JointSpring spring = wheel.suspensionSpring;
            spring.spring = SuspensionSpring;
            spring.damper = SuspensionDamp;
            wheel.suspensionSpring = spring;
        }

        void Awake()
        {
            Rigidbody = GetComponent<Rigidbody>();
            m_Inputs = GetComponents<IInput>();

            UpdateSuspensionParams(FrontLeftWheel);
            UpdateSuspensionParams(FrontRightWheel);
            UpdateSuspensionParams(RearLeftWheel);
            UpdateSuspensionParams(RearRightWheel);

            m_CurrentGrip = baseStats.Grip;
            if (NozzleVFX != null)
            {
                foreach (var nozzle in Nozzles)
                {
                    Instantiate(NozzleVFX, nozzle, false);
                }
            }
            UpdateStats();
        }

        void FixedUpdate()
        {
            UpdateSuspensionParams(FrontLeftWheel);
            UpdateSuspensionParams(FrontRightWheel);
            UpdateSuspensionParams(RearLeftWheel);
            UpdateSuspensionParams(RearRightWheel);

            GatherInputs();
            UpdateStats();
            // apply our physics properties
            Rigidbody.centerOfMass = transform.InverseTransformPoint(CenterOfMass.position);

            int groundedCount = 0;
            if (FrontLeftWheel.isGrounded && FrontLeftWheel.GetGroundHit(out WheelHit hit))
                groundedCount++;
            if (FrontRightWheel.isGrounded && FrontRightWheel.GetGroundHit(out hit))
                groundedCount++;
            if (RearLeftWheel.isGrounded && RearLeftWheel.GetGroundHit(out hit))
                groundedCount++;
            if (RearRightWheel.isGrounded && RearRightWheel.GetGroundHit(out hit))
                groundedCount++;

            // calculate how grounded and airborne we are
            GroundPercent = (float) groundedCount / 4.0f;
            AirPercent = 1 - GroundPercent;

            // apply vehicle physics
            if (m_CanMove)
            {
                MoveVehicle(Input.Accelerate, Input.Brake, Input.TurnInput, Input.HitWall);
            }
            GroundAirbourne();

            m_PreviousGroundPercent = GroundPercent;

        }

        void GatherInputs()
        {
            // reset input
            Input = new InputData();

            // gather nonzero input from our sources
            for (int i = 0; i < m_Inputs.Length; i++)
            {
                Input = m_Inputs[i].GenerateInput();
                // print("Input " + i + " accelerate is " + Input.Accelerate);
                // print("Input " + i + " brake is " + Input.Brake);
            }
        }

        public void UpdateStats()
        {
            m_FinalStats = baseStats;
            // clamp values in finalstats
            m_FinalStats.Grip = Mathf.Clamp(m_FinalStats.Grip, 0, 1);
            m_FinalStats.Steer = Mathf.Clamp((baseStats.MaxSteer * Mathf.Exp(-m_AccumulatedAngularV/TireWearRate)), baseStats.MinSteer, baseStats.MaxSteer);
            // print(m_FinalStats.Steer);
        }

        public float TireWearProportion()
        {
            return (baseStats.MaxSteer - m_FinalStats.Steer) / (baseStats.MaxSteer - baseStats.MinSteer);
        }

        void GroundAirbourne()
        {
            // while in the air, fall faster
            if (AirPercent >= 1)
            {
                Rigidbody.velocity += Physics.gravity * Time.fixedDeltaTime * m_FinalStats.AddedGravity;
            }
        }

        public void Reset()
        {
            Vector3 euler = transform.rotation.eulerAngles;
            euler.x = euler.z = 0f;
            transform.rotation = Quaternion.Euler(euler);
        }

        public float LocalSpeed()
        {
            if (m_CanMove)
            {
                float dot = Vector3.Dot(transform.forward, Rigidbody.velocity);
                if (Mathf.Abs(dot) > 0.1f)
                {
                    float speed = Rigidbody.velocity.magnitude;
                    return dot < 0 ? -(speed / m_FinalStats.ReverseSpeed) : (speed / m_FinalStats.TopSpeed);
                }
                return 0f;
            }
            else
            {
                // use this value to play kart sound when it is waiting the race start countdown.
                return Input.Accelerate ? 1.0f : 0.0f;
            }
        }
        void OnEnable()
        {
            UpdateStats();
        }
        void OnCollisionEnter(Collision collision) => m_HasCollision = true;
        void OnCollisionExit(Collision collision) => m_HasCollision = false;

        void OnCollisionStay(Collision collision)
        {
            m_HasCollision = true;
            m_LastCollisionNormal = Vector3.zero;
            float dot = -1.0f;

            foreach (var contact in collision.contacts)
            {
                if (Vector3.Dot(contact.normal, Vector3.up) > dot)
                    m_LastCollisionNormal = contact.normal;
            }
        }

        void MoveVehicle(bool accelerate, bool brake, float turnInput, bool hitWall=false)
        {
            // print("actual moving Accelerate " + accelerate + " Brake: " + brake + " turning input: " + turnInput);

            float accelInput = (accelerate ? 1.0f : 0.0f) - (brake ? 1.0f : 0.0f);

            // manual acceleration curve coefficient scalar
            float accelerationCurveCoeff = 5;
            Vector3 localVel = transform.InverseTransformVector(Rigidbody.velocity);

            bool accelDirectionIsFwd = accelInput >= 0;
            bool localVelDirectionIsFwd = localVel.z >= 0;

            // use the max speed for the direction we are going--forward or reverse.
            float maxSpeed = localVelDirectionIsFwd ? m_FinalStats.TopSpeed : m_FinalStats.ReverseSpeed;
            var maxAllowedSpeed = Mathf.Sqrt(getMaxLateralGs() * 9.81f * (Mathf.Abs(getTurningRadius())));
            // print(maxAllowedSpeed);
            if (!(float.IsInfinity(maxAllowedSpeed) || float.IsNaN(maxAllowedSpeed)))
            {
                maxSpeed = Mathf.Clamp(maxSpeed, 0.001f, Mathf.Max(maxAllowedSpeed, 0.001f));
            }
            float accelPower = accelDirectionIsFwd ? m_FinalStats.Acceleration : m_FinalStats.ReverseAcceleration;

            float currentSpeed = Rigidbody.velocity.magnitude;
            float accelRampT = currentSpeed / maxSpeed;
            float multipliedAccelerationCurve = m_FinalStats.AccelerationCurve * accelerationCurveCoeff;
            float accelRamp = Mathf.Lerp(multipliedAccelerationCurve, 1, accelRampT * accelRampT);

            bool isBraking = (localVelDirectionIsFwd && brake) || (!localVelDirectionIsFwd && accelerate);

            // if we are braking (moving reverse to where we are going)
            // use the braking accleration instead
            float finalAccelPower = isBraking ? m_FinalStats.Braking : accelPower;

            float finalAcceleration = finalAccelPower * accelRamp;

            // apply inputs to forward/backward
            float turningPower = turnInput * m_FinalStats.Steer * (Mathf.Abs(currentSpeed) > 0.5f ? 1f: 0f);

            Quaternion turnAngle = Quaternion.AngleAxis(turningPower, transform.up);
            Vector3 fwd = turnAngle * transform.forward;
            acc = fwd * accelInput * finalAcceleration * ((m_HasCollision || GroundPercent > 0.0f) ? 1.0f : 0.0f);

            // forward movement
            bool wasOverMaxSpeed = currentSpeed >= maxSpeed;

            // if over max speed, cannot accelerate faster.
            if (wasOverMaxSpeed && !isBraking) 
                acc *= 0.0f;

            Vector3 newVelocity = (Rigidbody.velocity + acc * Time.fixedDeltaTime) * (hitWall ? 0.95f: 1.0f);
            newVelocity.y = Rigidbody.velocity.y;

            //  clamp max speed if we are on ground
            if (GroundPercent > 0.0f && wasOverMaxSpeed)
            {
                newVelocity = Vector3.ClampMagnitude(newVelocity, maxSpeed);
            }

            // coasting is when we aren't touching accelerate
            if (Mathf.Abs(accelInput) < k_NullInput && GroundPercent > 0.0f)
            {
                newVelocity = Vector3.MoveTowards(newVelocity, new Vector3(0, Rigidbody.velocity.y, 0), Time.fixedDeltaTime * m_FinalStats.CoastingDrag);
            }

            Rigidbody.velocity = newVelocity;

            // Drift
            if (GroundPercent > 0.0f)
            {
                if (m_InAir)
                {
                    m_InAir = false;
                    Instantiate(JumpVFX, transform.position, Quaternion.identity);
                }

                // manual angular velocity coefficient
                float angularVelocitySteering = 0.4f;
                float angularVelocitySmoothSpeed = 20f;

                // turning is reversed if we're going in reverse and pressing reverse
                if (!localVelDirectionIsFwd && !accelDirectionIsFwd) 
                    angularVelocitySteering *= -1.0f;

                var angularVel = Rigidbody.angularVelocity;

                // move the Y angular velocity towards our target
                angularVel.y = Mathf.MoveTowards(angularVel.y, turningPower * angularVelocitySteering, Time.fixedDeltaTime * angularVelocitySmoothSpeed);
                m_AccumulatedAngularV += Mathf.Abs(angularVel.y);
                // apply the angular velocity
                Rigidbody.angularVelocity = angularVel;

                // rotate rigidbody's velocity as well to generate immediate velocity redirection
                // manual velocity steering coefficient
                float velocitySteering = 25f;

                // rotate our velocity based on current steer value
                Rigidbody.velocity = Quaternion.AngleAxis(turningPower * Mathf.Sign(localVel.z) * velocitySteering * m_CurrentGrip * Time.fixedDeltaTime, transform.up) * Rigidbody.velocity;
            }
            else
            {
                m_InAir = true;
            }

            bool validPosition = false;
            if (Physics.Raycast(transform.position + (transform.up * 0.1f), -transform.up, out RaycastHit hit, 3.0f, 1 << 9 | 1 << 10 | 1 << 11)) // Layer: ground (9) / Environment(10) / Track (11)
            {
                Vector3 lerpVector = (m_HasCollision && m_LastCollisionNormal.y > hit.normal.y) ? m_LastCollisionNormal : hit.normal;
                m_VerticalReference = Vector3.Slerp(m_VerticalReference, lerpVector, Mathf.Clamp01(AirborneReorientationCoefficient * Time.fixedDeltaTime * (GroundPercent > 0.0f ? 10.0f : 1.0f)));    // Blend faster if on ground
            }
            else
            {
                Vector3 lerpVector = (m_HasCollision && m_LastCollisionNormal.y > 0.0f) ? m_LastCollisionNormal : Vector3.up;
                m_VerticalReference = Vector3.Slerp(m_VerticalReference, lerpVector, Mathf.Clamp01(AirborneReorientationCoefficient * Time.fixedDeltaTime));
            }

            validPosition = GroundPercent > 0.7f && !m_HasCollision && Vector3.Dot(m_VerticalReference, Vector3.up) > 0.9f;

            // Airborne / Half on ground management
            if (GroundPercent < 0.7f)
            {
                Rigidbody.angularVelocity = new Vector3(0.0f, Rigidbody.angularVelocity.y * 0.98f, 0.0f);
                Vector3 finalOrientationDirection = Vector3.ProjectOnPlane(transform.forward, m_VerticalReference);
                finalOrientationDirection.Normalize();
                if (finalOrientationDirection.sqrMagnitude > 0.0f)
                {
                    Rigidbody.MoveRotation(Quaternion.Lerp(Rigidbody.rotation, Quaternion.LookRotation(finalOrientationDirection, m_VerticalReference), Mathf.Clamp01(AirborneReorientationCoefficient * Time.fixedDeltaTime)));
                }
            }
            else if (validPosition)
            {
                m_LastValidPosition = transform.position;
                m_LastValidRotation.eulerAngles = new Vector3(0.0f, transform.rotation.y, 0.0f);
            }
        }

        public float getMaxAngularVelocity()
        {
            float angularVelocitySteering = 0.4f;
            float turningPower =  m_FinalStats.Steer;
            return turningPower * angularVelocitySteering;
        }

        public float getMaxLateralGs()
        {
            return (1 - TireWearProportion()) * (m_FinalStats.MaxGs - m_FinalStats.MinGs) + m_FinalStats.MinGs;
        }

        public float getTurningRadius()
        {
            
            var output = (Vector3.Dot(Rigidbody.velocity, Rigidbody.transform.forward) / Rigidbody.angularVelocity.y);
            if (float.IsInfinity(output) || float.IsNaN(output))
                return 1000;
            return output;
        }

        public float getMaxSpeedForState()
        {
            var maxAllowedSpeed = Mathf.Sqrt(getMaxLateralGs() * 9.81f * (Mathf.Abs(getTurningRadius())));
            // print(maxAllowedSpeed);
            if (!(float.IsInfinity(maxAllowedSpeed) || float.IsNaN(maxAllowedSpeed)))
            {
                maxAllowedSpeed = m_FinalStats.TopSpeed;
            }
            return maxAllowedSpeed;
        }
    }
}

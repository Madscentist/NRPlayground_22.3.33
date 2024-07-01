using System;
using System.Collections.Generic;
using CarControllerTest.Inpt;
using CarControllerTest.Terminal;
using UnityEngine;
using UnityEngine.Serialization;

namespace CarControllerTest
{
    public class NrCarController : MonoBehaviour
    {
        private Rigidbody rb;

        #region Wheels

        public Wheels wheels;

        [Serializable]
        public struct Wheels
        {
            public NrWheel fl;
            public NrWheel fr;
            public NrWheel rl;
            public NrWheel rr;

            public NrWheel this[int x]
            {
                get
                {
                    switch (x)
                    {
                        case 0: return fl;
                        case 1: return fr;
                        case 2: return rl;
                        case 3: return rr;
                        default: throw new ArgumentOutOfRangeException();
                    }
                }
            }

            public IEnumerator<NrWheel> GetEnumerator()
            {
                yield return fl;
                yield return fr;
                yield return rl;
                yield return rr;
            }
        }

        #endregion

        #region Input

        [HideInInspector] public CarInputTest CarControls;

        public Inputs inputs;

        [Serializable]
        public struct Inputs
        {
            public float turnWeight;
            public float acceleratorWeight;

            public float brakeWeight;

            /// <summary>
            /// 漂移
            /// </summary>
            public bool drift;

            /// <summary>
            /// 氮气加速
            /// </summary>
            public bool nos;
        }

        #endregion

        #region Layer

        public CtrlLayer layer;

        [Serializable]
        public struct CtrlLayer
        {
            public LayerMask carBody;
            public LayerMask wheels;
        }

        #endregion

        #region Status

        public Status status;
        public CarSetting setting;

        [Serializable]
        public struct Status
        {
            /// <summary>
            /// Grounded
            /// </summary>
            public bool grounded;

            /// <summary>
            /// the velocity to world
            /// </summary>
            public Vector3 worldCarVelocity;

            /// <summary>
            /// the velocity to self
            /// </summary>
            public Vector3 localCarVelocity;

            public int kmH;

            public bool isSlip;
        }

        [Serializable]
        public struct CarSetting
        {
            public bool useAirDrag;
            public float airDrag;
            public float gravity;

            public float maxTurningAngle;
            public float maxAccelerateForce;
            public float maxBrakeForce;
            public float nosForce;
            public float dragFactor;

            public AnimationCurve accelerationCurve;
            public AnimationCurve turningCurve;
            public AnimationCurve turningBrakingCurve;

            public Transform centerOfMass;
        }

        #endregion

        private void Awake()
        {
            CarControls = new CarInputTest();
            rb = GetComponent<Rigidbody>();
            rb.centerOfMass = setting.centerOfMass.localPosition;

            status.grounded = false;

            if (GetComponent<NrGravity>())
            {
                setting.gravity = GetComponent<NrGravity>().gravityScale;
            }
            else
            {
                //setting.gravity = NoobApi.Instance.Settings.gravity;
                //print("Get from Api, value: " + setting.gravity);
            }
        }

        private void OnEnable()
        {
            CarControls.Enable();
        }

        private void OnDisable()
        {
            CarControls.Disable();
        }

        private void FixedUpdate()
        {
            //main status check
            InputUpdate();
            GroundCheck();
            StatusCheck();

            // main movement calculation loop

            AccelerationUpdate();
            TurningUpdate();
            FrictionUpdate();
            BrakeUpdate();
            //DriftUpdate();

            // apply movement
            UpdateInWheels();
        }

        private void Update()
        {
            UpdateVisualTire();
            //DriftingTrail();

            foreach (var wheel in wheels)
            {
                wheel.DriftingTrail(status.isSlip);
            }
        }

        #region Input Methods

        private float LinearInput(float recent, float target, float t, float min, float max, bool minus = true)
        {
            var result = Mathf.Lerp(recent, target, t);

            if (Mathf.Abs(result - min) < 0.005f)
            {
                result = min;
            }

            if (Mathf.Abs(result + max) < 0.005f && minus)
            {
                result = -max;
            }

            if (Mathf.Abs(result - max) < 0.005f)
            {
                result = max;
            }

            return result;
        }

        private void InputUpdate()
        {
            var moveH = CarControls.RaceControl.moveH.ReadValue<float>();
            inputs.turnWeight = LinearInput(inputs.turnWeight, moveH, 0.25f, 0f, 1f);

            var moveV = CarControls.RaceControl.moveV.ReadValue<float>();
            inputs.acceleratorWeight = LinearInput(inputs.acceleratorWeight, moveV, 0.25f, 0f, 1f);
            //inputs.acceleratorWeight = moveV;

            var brake = CarControls.RaceControl.brake.IsPressed();
            inputs.brakeWeight = LinearInput(inputs.brakeWeight, brake ? 1f : 0f, 0.25f, 0f, 1f, false);

            inputs.nos = CarControls.RaceControl.speedup.IsPressed();
            inputs.drift = CarControls.RaceControl.drift.IsPressed();
        }

        #endregion

        #region StatusCheck Methods

        private void StatusCheck()
        {
            status.worldCarVelocity = rb.velocity;
            status.localCarVelocity = transform.InverseTransformDirection(status.worldCarVelocity);
            status.kmH = (int)(status.worldCarVelocity.magnitude * 3.6f) *
                         (Vector3.Dot(transform.forward, status.worldCarVelocity) > 0f ? 1 : -1);

            //判断是否打滑
            status.isSlip = Vector3.Angle(status.worldCarVelocity.normalized, transform.forward) >
                            setting.maxTurningAngle;
        }

        private void GroundCheck()
        {
        }

        #endregion

        #region Movement Logic

        [SerializeField] private float currentAcceleration;
        [SerializeField] private float currentBrakeForce;
        [SerializeField] private float currentTurningAngle;
        private float currentDragForce;

        [SerializeField] private bool _isDrifting;

        private void AccelerationUpdate()
        {
            currentDragForce = status.kmH * status.kmH * setting.dragFactor;

            currentAcceleration = inputs.acceleratorWeight *
                                  setting.maxAccelerateForce +
                                  (inputs.nos ? setting.nosForce : 0f);
        }

        private void TurningUpdate()
        {
            currentTurningAngle = setting.maxTurningAngle * inputs.turnWeight *
                                  setting.turningCurve.Evaluate(status.kmH / 100f);
        }

        private void FrictionUpdate()
        {
            rb.AddForce(-rb.velocity.normalized *
                        (currentDragForce + (status.isSlip ? currentDragForce / 2f : 0f)));
        }

        private void BrakeUpdate()
        {
            currentBrakeForce = inputs.brakeWeight * setting.maxBrakeForce;
            //* setting.turningBrakingCurve.Evaluate(Mathf.Abs(inputs.turnWeight));
        }

        private float driftFactor;
        private float driftSmoothFactor;
        private WheelFrictionCurve forwardFriction;
        private WheelFrictionCurve sidewaysFriction;

        [SerializeField] [Range(0f, 3f)] private float handBrakeFrictionMultiplier = 2f;

        [SerializeField] [Range(0f, 1f)] private float driftFactorNumber = 0.7f;

        [SerializeField] private float driftMax = 0.3f;


        private void DriftUpdate()
        {
            driftSmoothFactor = driftFactorNumber * Time.fixedDeltaTime;

            if (inputs.drift && inputs.turnWeight is > 0.5f or < -0.5f)
            {
                Drifting_On();
            }
            else
            {
                Drifting_Off();
            }
        }

        private void Drifting_On()
        {
            _isDrifting = true;

            sidewaysFriction = wheels.fl.collider.sidewaysFriction;
            forwardFriction = wheels.fl.collider.forwardFriction;

            var velocity = 0f;

            sidewaysFriction.extremumValue = sidewaysFriction.asymptoteValue = forwardFriction.extremumValue =
                forwardFriction.asymptoteValue
                    = Mathf.SmoothDamp(forwardFriction.asymptoteValue, driftFactor * handBrakeFrictionMultiplier,
                        ref velocity, driftSmoothFactor);

            foreach (var wheel in wheels)
            {
                SetWheelFriction(wheel.collider, sidewaysFriction, forwardFriction);
            }

            sidewaysFriction.extremumValue = sidewaysFriction.asymptoteValue =
                forwardFriction.extremumValue = forwardFriction.asymptoteValue = 1.1f;

            SetWheelFriction(wheels[0].collider, sidewaysFriction, forwardFriction);
            SetWheelFriction(wheels[1].collider, sidewaysFriction, forwardFriction);
            SetWheelFriction(wheels[2].collider, sidewaysFriction, forwardFriction);
            SetWheelFriction(wheels[3].collider, sidewaysFriction, forwardFriction);


            rb.AddForce(transform.forward * ((status.kmH / 400f) * 1000f));
        }

        private void Drifting_Off()
        {
            _isDrifting = false;
            forwardFriction = wheels[0].collider.forwardFriction;
            sidewaysFriction = wheels[0].collider.sidewaysFriction;

            forwardFriction.extremumValue = forwardFriction.asymptoteValue =
                sidewaysFriction.extremumValue = sidewaysFriction.asymptoteValue = ((status.kmH *
                    handBrakeFrictionMultiplier / 300f) + 1);

            foreach (var wheel in wheels)
            {
                SetWheelFriction(wheel.collider, forwardFriction, sidewaysFriction);
            }

            for (int i = 2; i < 4; i++)
            {
                wheels[i].collider.GetGroundHit(out var hit);

                if (hit.sidewaysSlip >= driftMax || hit.sidewaysSlip <= -driftMax || hit.forwardSlip >= driftMax ||
                    hit.forwardSlip <= -driftMax)
                {
                    //wheels[i].DriftingTrail(true);
                }
                else
                {
                    //wheels[i].DriftingTrail(false);
                }

                if (hit.sidewaysSlip < 0f)
                {
                    driftFactor = (1 - inputs.turnWeight) * Mathf.Abs(hit.sidewaysSlip);
                }

                if (hit.sidewaysSlip > 0f)
                {
                    driftFactor = (1 + inputs.turnWeight) * Mathf.Abs(hit.sidewaysSlip);
                }
            }
        }


        private void SetWheelFriction(WheelCollider wheelCollider, WheelFrictionCurve sideways,
            WheelFrictionCurve forward)
        {
            wheelCollider.sidewaysFriction = sideways;
            wheelCollider.forwardFriction = forward;
        }

        private void UpdateInWheels()
        {
            // Stabilizer bar
            StabilizerTwoWheels(wheels.fl.collider, wheels.fr.collider);
            StabilizerTwoWheels(wheels.rl.collider, wheels.rr.collider);

            // Acceleration
            wheels.fr.collider.motorTorque = currentAcceleration;
            wheels.fl.collider.motorTorque = currentAcceleration;


            // Brake force
            wheels.fr.collider.brakeTorque = currentBrakeForce;
            wheels.fl.collider.brakeTorque = currentBrakeForce;
            wheels.rr.collider.brakeTorque = currentBrakeForce;
            wheels.rl.collider.brakeTorque = currentBrakeForce;

            wheels.fr.collider.steerAngle = currentTurningAngle;
            wheels.fl.collider.steerAngle = currentTurningAngle;
        }

        private const float AntiRoll = 5000f;

        private void StabilizerTwoWheels(WheelCollider left, WheelCollider right)
        {
            var travelL = 1f;
            var travelR = 1f;

            var groundedL = left.GetGroundHit(out var hitL);
            var groundedR = right.GetGroundHit(out var hitR);

            if (groundedL)
            {
                travelL = -(left.transform.InverseTransformPoint(hitL.point).y - left.radius) / left.suspensionDistance;
            }

            if (groundedR)
            {
                travelR = -(right.transform.InverseTransformPoint(hitR.point).y - right.radius) /
                          right.suspensionDistance;
            }

            var antiRollForce = (travelL - travelR) * AntiRoll;

            if (groundedL)
            {
                rb.AddForceAtPosition(left.transform.up * -antiRollForce, left.transform.position);
            }

            if (groundedR)
            {
                rb.AddForceAtPosition(right.transform.up * antiRollForce, right.transform.position);
            }
        }

        #endregion

        #region Effects logic

        private void UpdateVisualTire()
        {
            VisualWheelTurning(wheels.rr);
            VisualWheelTurning(wheels.rl);
            VisualWheelTurning(wheels.fl);
            VisualWheelTurning(wheels.fr);
        }

        private void DriftingTrail()
        {
            var carDir = rb.velocity.normalized;

            foreach (var wheel in wheels)
            {
                var angle = Vector3.Angle(wheel.mesh.forward, carDir);

                wheel.DriftingTrail(angle > setting.maxTurningAngle);
            }
        }

        private void VisualWheelTurning(NrWheel wheel)
        {
            var col = wheel.collider;
            var mesh = wheel.mesh;

            col.GetWorldPose(out var pos, out var ro);
            mesh.position = pos;
            mesh.rotation = ro;
        }

        #endregion

        private void DrawDebugBox(Vector3 center, Vector3 size, Color color)
        {
            Vector3 halfSize = size * 0.5f;

            Vector3[] vertices = new Vector3[]
            {
                center + new Vector3(-halfSize.x, -halfSize.y, -halfSize.z),
                center + new Vector3(-halfSize.x, -halfSize.y, halfSize.z),
                center + new Vector3(halfSize.x, -halfSize.y, halfSize.z),
                center + new Vector3(halfSize.x, -halfSize.y, -halfSize.z),
                center + new Vector3(-halfSize.x, halfSize.y, -halfSize.z),
                center + new Vector3(-halfSize.x, halfSize.y, halfSize.z),
                center + new Vector3(halfSize.x, halfSize.y, halfSize.z),
                center + new Vector3(halfSize.x, halfSize.y, -halfSize.z)
            };

            // Draw the bottom lines of the box
            Debug.DrawLine(vertices[0], vertices[1], color);
            Debug.DrawLine(vertices[1], vertices[2], color);
            Debug.DrawLine(vertices[2], vertices[3], color);
            Debug.DrawLine(vertices[3], vertices[0], color);

            // Draw the top lines of the box
            Debug.DrawLine(vertices[4], vertices[5], color);
            Debug.DrawLine(vertices[5], vertices[6], color);
            Debug.DrawLine(vertices[6], vertices[7], color);
            Debug.DrawLine(vertices[7], vertices[4], color);

            // Draw the connecting lines between top and bottom
            Debug.DrawLine(vertices[0], vertices[4], color);
            Debug.DrawLine(vertices[1], vertices[5], color);
            Debug.DrawLine(vertices[2], vertices[6], color);
            Debug.DrawLine(vertices[3], vertices[7], color);
        }
    }
}
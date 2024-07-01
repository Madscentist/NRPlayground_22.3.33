using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;


namespace AshVP
{
    public class carController : MonoBehaviour
    {
        [Header("Suspension")]
        [Range(0, 5)]
        public float SuspensionDistance = 0.2f;
        public float suspensionForce = 30000f;
        public float suspensionDamper = 200f;
        public Transform groundCheck;
        public Transform fricAt;
        public Transform CenterOfMass;

        [HideInInspector]
        public CarInputs carControls;
        private Rigidbody rb;

        [Header("Car Stats")]
        public float accelerationForce = 200f;
        public float turnTorque = 100f;
        public float brakeForce = 150;
        public float frictionForce = 70f;
        public float dragAmount = 4f;
        public float TurnAngle = 30f;

        public float maxRayLength = 0.8f, slerpTime = 0.2f;
        [HideInInspector]
        public bool grounded;

        [Header("Visuals")]
        public Transform[] TireMeshes;
        public Transform[] TurnTires;

        [Header("Curves")]
        public AnimationCurve frictionCurve;
        public AnimationCurve accelerationCurve;
        public bool separateReverseCurve = false;
        public AnimationCurve ReverseCurve;
        public AnimationCurve turnCurve;
        public AnimationCurve driftCurve;
        public AnimationCurve engineCurve;

        private float speedValue, fricValue, turnValue, curveVelocity, brakeInput;
        [HideInInspector]
        public Vector3 carVelocity;
        [HideInInspector]
        public RaycastHit hit;

        [Header("Other Settings")]
        public AudioSource[] engineSounds;
        public bool airDrag;
        public float SkidEnable = 20f;
        public float skidWidth = 0.12f;
        private float frictionAngle;


        [HideInInspector]
        public Vector3 normalDir;

        private float VehicleGravity = -30;
        private Vector3 centerOfMass_ground;

        private void Awake()
        {
            carControls = new CarInputs();

            rb = GetComponent<Rigidbody>();
            grounded = false;
            engineSounds[1].mute = true;
            rb.centerOfMass = CenterOfMass.localPosition;
            Vector3 centerOfMass_ground_temp = Vector3.zero;
            for (int i = 0; i < TireMeshes.Length; i++)
            {
                centerOfMass_ground_temp += TireMeshes[i].parent.parent.localPosition;
            }
            centerOfMass_ground_temp.y = 0;
            centerOfMass_ground = centerOfMass_ground_temp / 4;

            // for bike
            if (TireMeshes.Length < 3)
            {
                centerOfMass_ground = centerOfMass_ground_temp / 2;
            }

            if (GetComponent<GravityCustom>())
            {
                VehicleGravity = GetComponent<GravityCustom>().gravity;
            }
            else
            {
                VehicleGravity = Physics.gravity.y;
            }

            //ground boxcast size
            if (TireMeshes.Length < 3)
            {
                raycast_boxLength = Vector3.Distance(TireMeshes[0].position, TireMeshes[1].position);
                raycast_boxWidth = 0.1f;
            }
            else
            {
                raycast_boxLength = Vector3.Distance(TireMeshes[0].position, TireMeshes[2].position);
                raycast_boxWidth = Vector3.Distance(TireMeshes[0].position, TireMeshes[1].position);
            }
        }
        private void OnEnable()
        {
            carControls.Enable();
        }
        private void OnDisable()
        {
            carControls.Disable();
        }

        void FixedUpdate()
        {
            carVelocity = transform.InverseTransformDirection(rb.velocity); //local velocity of car

            curveVelocity = Mathf.Abs(carVelocity.magnitude) / 100;

            //inputs
            float turnInput = turnTorque * carControls.carAction.moveH.ReadValue<float>() * Time.fixedDeltaTime * 1000;
            float speedInput = accelerationForce * carControls.carAction.moveV.ReadValue<float>() * Time.fixedDeltaTime * 1000;
            brakeInput = brakeForce * carControls.carAction.brake.ReadValue<float>() * Time.fixedDeltaTime * 1000;

            //helping veriables

            speedValue = speedInput * accelerationCurve.Evaluate(Mathf.Abs(carVelocity.z) / 100);
            if (separateReverseCurve && carVelocity.z < 0 && speedInput < 0)
            {
                speedValue = speedInput * ReverseCurve.Evaluate(Mathf.Abs(carVelocity.z) / 100);
            }
            fricValue = frictionForce * frictionCurve.Evaluate(carVelocity.magnitude / 100);
            turnValue = turnInput * turnCurve.Evaluate(carVelocity.magnitude / 100);

            //grounded check
            GroundCheck();

            if (grounded)
            {
                accelerationLogic();
                turningLogic();
                frictionLogic();
                brakeLogic();
                //for drift behaviour
                rb.angularDrag = dragAmount * driftCurve.Evaluate(Mathf.Abs(carVelocity.x) / 70);

                //draws green ground checking ray ....ingnore
                Debug.DrawLine(groundCheck.position, hit.point, Color.green);

                rb.centerOfMass = centerOfMass_ground;

                normalDir = hit.normal;

            }
            else
            {
                grounded = false;
                rb.drag = 0.1f;
                rb.centerOfMass = CenterOfMass.localPosition;
                if (!airDrag)
                {
                    rb.angularDrag = 0.1f;
                }
            }

        }

        void Update()
        {

            tireVisuals();
            audioControl();

        }

        float raycast_boxWidth, raycast_boxLength;
        public void GroundCheck()
        {
            int wheelsLayerMask = 1 << LayerMask.NameToLayer("wheels");
            int layerMask = ~wheelsLayerMask;
            float rayMultiplier = 1 / Mathf.Clamp(Mathf.Cos(transform.rotation.eulerAngles.z), 0.8f, 1);
            Vector3 boxSize = new Vector3(raycast_boxWidth / 2, 0.01f, raycast_boxLength / 2);
            if (Physics.BoxCast(groundCheck.position, boxSize, -transform.up, out hit, transform.rotation, maxRayLength* rayMultiplier, layerMask))
            {
                grounded = true;
                DrawDebugBox(groundCheck.position - transform.up * (hit.distance - 0.5f * boxSize.y), 2*boxSize, Color.blue);
            }
            else
            {
                grounded = false;
                DrawDebugBox(groundCheck.position - transform.up * maxRayLength * rayMultiplier, 2*boxSize, Color.red);
            }
        }

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

        public void audioControl()
        {
            //audios
            if (grounded)
            {
                if (Mathf.Abs(carVelocity.x) > SkidEnable - 0.1f)
                {
                    engineSounds[1].mute = false;
                }
                else { engineSounds[1].mute = true; }
            }
            else
            {
                engineSounds[1].mute = true;
            }

            engineSounds[1].pitch = 1f;

            engineSounds[0].pitch = 2 * engineCurve.Evaluate(curveVelocity);
            if (engineSounds.Length == 2)
            {
                return;
            }
            else { engineSounds[2].pitch = 2 * engineCurve.Evaluate(curveVelocity); }

        }

        public void tireVisuals()
        {
            //Tire mesh rotate
            foreach (Transform mesh in TireMeshes)
            {
                mesh.transform.RotateAround(mesh.transform.position, mesh.transform.right, carVelocity.z / 3);
                mesh.transform.localPosition = Vector3.zero;
            }

            //TireTurn
            foreach (Transform FM in TurnTires)
            {
                FM.localRotation = Quaternion.Slerp(FM.localRotation, Quaternion.Euler(FM.localRotation.eulerAngles.x,
                                   TurnAngle * carControls.carAction.moveH.ReadValue<float>(), FM.localRotation.eulerAngles.z), slerpTime);
            }
        }

        public void accelerationLogic()
        {
            //speed control
            if (carControls.carAction.moveV.ReadValue<float>() > 0.1f)
            {
                rb.AddForceAtPosition(transform.forward * speedValue, groundCheck.position);
            }
            if (carControls.carAction.moveV.ReadValue<float>() < -0.1f)
            {
                rb.AddForceAtPosition(transform.forward * speedValue, groundCheck.position);
            }
        }

        public void turningLogic()
        {
            //turning
            if (carVelocity.z > 0.1f)
            {
                rb.AddTorque(transform.up * turnValue);
            }
            else if (carControls.carAction.moveV.ReadValue<float>() > 0.1f)
            {
                rb.AddTorque(transform.up * turnValue);
            }
            else if (carVelocity.z < -0.1f && carControls.carAction.moveV.ReadValue<float>() > 0.1f)
            {
                rb.AddTorque(transform.up * turnValue);
            }
            else if (carVelocity.z < -0.1f)
            {
                rb.AddTorque(transform.up * -turnValue);
            }
        }
        
        public void frictionLogic()
        {
            Vector3 sideVelocity = carVelocity.x * transform.right;

            Vector3 contactDesiredAccel = -sideVelocity/ Time.fixedDeltaTime;

            float clampedFrictionForce = rb.mass * contactDesiredAccel.magnitude;

            Vector3 gravityForce = VehicleGravity * rb.mass * Vector3.up;

            Vector3 gravityFriction = -Vector3.Project( gravityForce, transform.right);

            Vector3 maxfrictionForce = Vector3.ClampMagnitude(fricValue * 50 * (-sideVelocity.normalized), clampedFrictionForce);
            rb.AddForceAtPosition(maxfrictionForce + gravityFriction, fricAt.position);
        }

        public void brakeLogic()
        {
            Vector3 forwardVelocity = carVelocity.z * transform.forward;
            Vector3 DesiredAccel = -forwardVelocity / Time.fixedDeltaTime;
            float clampedBrakeForce = rb.mass * DesiredAccel.magnitude;
            Vector3 maxBrakeForce = Vector3.ClampMagnitude(brakeInput * (-forwardVelocity.normalized), clampedBrakeForce);

            //brake
            rb.AddForceAtPosition(maxBrakeForce, groundCheck.position);


            if (carVelocity.magnitude < 1)
            {
                rb.drag = 5f;
            }
            else
            {
                rb.drag = 0.1f;
            }
        }

        #region Gizmos

#if UNITY_EDITOR
        private void OnDrawGizmos()
        {
            Gizmos.DrawSphere(transform.position + centerOfMass_ground, 0.05f);

            if (!Application.isPlaying)
            {
                Gizmos.color = Color.yellow;
                Gizmos.DrawLine(groundCheck.position, groundCheck.position - maxRayLength * groundCheck.up);
                Gizmos.DrawWireCube(groundCheck.position - maxRayLength * (groundCheck.up.normalized), new Vector3(5, 0.02f, 10));
                Gizmos.color = Color.magenta;
                if (GetComponent<BoxCollider>())
                {
                    var Box_collider = GetComponent<BoxCollider>();
                    Gizmos.DrawWireCube(Box_collider.bounds.center, Box_collider.size);
                }
                if (GetComponent<CapsuleCollider>())
                {
                    var Capsule_collider = GetComponent<CapsuleCollider>();
                    Gizmos.DrawWireCube(Capsule_collider.bounds.center, Capsule_collider.bounds.size);
                }

                Gizmos.color = Color.red;
                foreach (Transform mesh in TireMeshes)
                {
                    var ydrive = mesh.parent.parent.GetComponent<ConfigurableJoint>().yDrive;
                    ydrive.positionDamper = suspensionDamper;
                    ydrive.positionSpring = suspensionForce;


                    mesh.parent.parent.GetComponent<ConfigurableJoint>().yDrive = ydrive;

                    var jointLimit = mesh.parent.parent.GetComponent<ConfigurableJoint>().linearLimit;
                    jointLimit.limit = SuspensionDistance;
                    mesh.parent.parent.GetComponent<ConfigurableJoint>().linearLimit = jointLimit;

                    Handles.color = Color.red;
                    //Handles.DrawWireCube(mesh.position, new Vector3(0.02f, 2 * jointLimit.limit, 0.02f));
                    Handles.ArrowHandleCap(0, mesh.position, mesh.rotation * Quaternion.LookRotation(Vector3.up), jointLimit.limit, EventType.Repaint);
                    Handles.ArrowHandleCap(0, mesh.position, mesh.rotation * Quaternion.LookRotation(Vector3.down), jointLimit.limit, EventType.Repaint);

                }
                float wheelRadius = TurnTires[0].parent.GetComponent<SphereCollider>().radius;
                float wheelYPosition = TurnTires[0].parent.parent.localPosition.y + TurnTires[0].parent.localPosition.y;
                maxRayLength = (groundCheck.localPosition.y - wheelYPosition + (0.05f + wheelRadius));

            }

        }
#endif
        #endregion
    }
}

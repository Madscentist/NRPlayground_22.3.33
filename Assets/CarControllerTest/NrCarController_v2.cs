using System;
using CarControllerTest.Inpt;
using UnityEngine;
using UnityEngine.Serialization;

namespace CarControllerTest
{
    public class NrCarController_v2 : MonoBehaviour
    {
        public Rigidbody sphereRigidBody;

        public Transform carMesh;

        public GroundCheck groundCheck;



        private float _rotate;


        #region Car Status

        public Status status;
        
        [Serializable]
        public struct Status
        {
            public int kmH;
        }
        

        #endregion
        

        #region CarSettings

        public CarSetting setting;

        [Serializable]
        public struct CarSetting
        {
            public float turnSpeed;
            public float forwardSpeed;
        }

        #endregion
        

        #region Inputs

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
        
        [HideInInspector] public CarInputTest CarControls;

        public Inputs inputs;

        #endregion

        private void Awake()
        {
            CarControls = new CarInputTest();
        }

        private void Start()
        {
            sphereRigidBody.transform.parent = null;
        }

        private void OnEnable()
        {
            CarControls.Enable();
        }

        
        
        private float _forwardAmount;
        private float _turnAmount;
        private float _currentSpeed;
        private float _currentRotate;
        
        private void Update()
        {
            InputUpdate();
            
            // Copy position
            transform.position = sphereRigidBody.transform.position;

            _forwardAmount = inputs.acceleratorWeight;
            _turnAmount = inputs.turnWeight;

            if (_forwardAmount != 0f && groundCheck.isGrounded)
            {
                Drive();
            }
            else
            {
                DriveNoWhere();
            }

            TurnHandler();
            GroundNormalHandler();
        }

        private void GroundNormalHandler()
        {
            transform.rotation = Quaternion.Lerp(transform.rotation,
                Quaternion.FromToRotation(transform.up, groundCheck.groundNormal) * transform.rotation,
                5f * Time.deltaTime);
        }

        private void FixedUpdate()
        {
            StatusCheck();
            sphereRigidBody.AddForce(_currentSpeed * transform.forward, ForceMode.Acceleration);
        }

        private void Drive()
        {
            _currentSpeed = _forwardAmount *= setting.forwardSpeed;
        }

        private void DriveNoWhere()
        {
            _currentSpeed = 0f;
        }

        private void TurnHandler()
        {
            var newRotation = _turnAmount * setting.turnSpeed * Time.deltaTime;
            
            if (_currentSpeed > 0.1f)
            {
                transform.Rotate(0f, newRotation, 0f, Space.World);

            }
            
        }

        private void StatusCheck()
        {
            var velocity = sphereRigidBody.velocity;
            
            
            status.kmH = (int)(velocity.magnitude * 3.6f) *
                         (Vector3.Dot(transform.forward, velocity) > 0f ? 1 : -1);
        }

        #region Input Meshthods

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

        #endregion
        
        
    }
}
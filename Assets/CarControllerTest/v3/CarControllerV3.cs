using System;
using AshVP;
using DavidJalbert;
using UnityEditor;
using UnityEngine;
using UnityEngine.Serialization;

namespace CarControllerTest.v3
{
    public class CarControllerV3 : TinyCarController
    {
        [Header("Drifting")]
        public float driftingStrongerWeight = 1.5f;
        public float driftingWeakerWeight = 0.5f;
        
        public bool _isDrifting;
        public DriftingDir _driftingDir;
        
        public enum DriftingDir
        {
            Left,
            Right,
            None
        }
        
        protected override void FixedUpdate()
        {

            if (_isDrifting)
            {
                if (!isGrounded())
                {
                    _isDrifting = false;
                }
                
                switch (_driftingDir)
                {
                    case DriftingDir.Left:

                        if (inputSteering > 0f)
                        {
                            inputSteering *= driftingWeakerWeight;
                        }

                        if (inputSteering < 0f)
                        {
                            inputSteering *= driftingStrongerWeight;
                        }
                        
                        break;
                    case DriftingDir.Right:
                        
                        if (inputSteering < 0f)
                        {
                            inputSteering *= driftingWeakerWeight;
                        }

                        if (inputSteering > 0f)
                        {
                            inputSteering *= driftingStrongerWeight;
                        }
                        
                        break;
                    case DriftingDir.None:

                        _isDrifting = false;
                        
                        break;
                    default:
                        throw new ArgumentOutOfRangeException();
                }
                
                //print(inputSteering);

            }
            
            
            base.FixedUpdate();
        }

        public void startDrift(float dir)
        {
            if (dir == 0f)
            {
                return;
            }
            
            if (dir < 0f)
            {
                _driftingDir = DriftingDir.Left;
            }

            if (dir > 0f)
            {
                _driftingDir = DriftingDir.Right;
            }

            _isDrifting = true;
        }

        public void endDrift()
        {
            _isDrifting = false;
            _driftingDir = DriftingDir.None;
        }
    }
}
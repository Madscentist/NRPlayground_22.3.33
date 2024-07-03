using System;
using System.Collections;
using System.Collections.Generic;
using CarControllerTest.Inpt;
using CarControllerTest.v3;
using DavidJalbert;
using UnityEngine;
using UnityEngine.InputSystem;

public class CarInput : MonoBehaviour
{
    public CarControllerV3 carController;

    [Tooltip("For how long the boost should last in seconds.")]
    public float boostDuration = 1;

    [Tooltip("How long to wait after a boost has been used before it can be used again, in seconds.")]
    public float boostCoolOff = 0;

    [Tooltip("The value by which to multiply the speed and acceleration of the car when a boost is used.")]
    public float boostMultiplier = 2;

    [SerializeField] private float boostTimer = 0;

    private CarInputV3Asset _inputV3Asset;

    private bool _isBoost;

    private void Awake()
    {
        _inputV3Asset = new CarInputV3Asset();
    }

    private void OnEnable()
    {
        _inputV3Asset.Enable();
    }


    private void Update()
    {
        float motorDelta = 0f;

        float steeringDelta = 0f;

        _isBoost = _inputV3Asset.Drive.Boost.WasPressedThisFrame();

        if (_inputV3Asset.Drive.Brake.IsPressed())
        {
            if (carController.getMotor() > 0f)
            {
                motorDelta = -1f;
            }else if (carController.getMotor() < 0f)
            {
                motorDelta = 1f;
            }
            else
            {
                motorDelta = 0f;
            }
        }
        else
        {
            motorDelta = _inputV3Asset.Drive.Vertical.ReadValue<float>();
            steeringDelta = _inputV3Asset.Drive.Horizontal.ReadValue<float>();
        }

        if (_inputV3Asset.Drive.Drift.WasPressedThisFrame() && steeringDelta != 0f)
        {
            carController.startDrift(steeringDelta);
        }

        if (carController._isDrifting)
        {
            boostTimer += Time.deltaTime;
            boostTimer = Mathf.Clamp(boostTimer, 0f, boostDuration);

            if (!_inputV3Asset.Drive.Drift.IsPressed())
            {
                carController.endDrift();
                _isBoost = true;
            }
        }

        if (_isBoost && boostTimer == 0)
        {
            boostTimer = boostCoolOff + boostDuration;
        }
        else if (boostTimer > 0)
        {
            if (carController._isDrifting)
            {
                
            }
            else
            {
                boostTimer = Mathf.Max(boostTimer - Time.deltaTime, 0);
                carController.setBoostMultiplier(boostTimer > boostCoolOff ? boostMultiplier : 1);
            }
            
        }


        carController.setSteering(steeringDelta);
        carController.setMotor(motorDelta);
    }
}
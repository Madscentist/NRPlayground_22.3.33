using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class NrWheel : MonoBehaviour
{
    public Transform mesh;
    public WheelCollider collider;

    public TrailRenderer driftingTrail;
    
    private void FixedUpdate()
    {
    }

    public void DriftingTrail(bool on)
    {
        driftingTrail.emitting = on;
    }
}

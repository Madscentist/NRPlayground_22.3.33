using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Serialization;

public class GroundCheck : MonoBehaviour
{
    public LayerMask mask;
    
    public Transform center;

    public Vector3 groundNormal;

    public bool isGrounded;
    
    private void FixedUpdate()
    {
        isGrounded = Physics.Raycast(center.position, Vector3.down, out var hit, 1.1f, mask);
        Physics.Raycast(center.position, Vector3.down, out var hitNear, 2.0f, mask);

        groundNormal = hitNear.normal;
    }

    private void OnDrawGizmos()
    {
        Gizmos.color = Color.red;
        Gizmos.DrawLine(center.position, center.position - center.up * 2f);
        Gizmos.color = Color.blue;
        Gizmos.DrawLine(center.position, center.position - center.up * 1.1f);

    }
}

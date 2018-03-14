using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SteeringRB : MonoBehaviour {

    public Transform target;
    public float maxSpeed;
    public float maxForce;

    // arrival
    public float stoppingDistance;

    Rigidbody rigidBody;
    Vector3 vDesired;
    Vector3 vSteer;

    void Start()
    {
        rigidBody = GetComponent<Rigidbody>();
        if (rigidBody == null)
        {
            Debug.Log("No Rigidbody Detected!");
        }
    }

    void Seek()
    {
        vDesired = (target.position - rigidBody.position).normalized * maxSpeed;
        vSteer = vDesired - rigidBody.velocity;

        if (vSteer.sqrMagnitude > Mathf.Pow(maxForce, 2))
        {
            vSteer = vSteer.normalized * maxForce;
        }

        rigidBody.AddForce(vSteer, ForceMode.Acceleration);
    }

    void SeekArrive()
    {
        Vector3 vDesiredMaxSpeed = (target.position - rigidBody.position).normalized * maxSpeed;

        if ((target.position - rigidBody.position).sqrMagnitude < stoppingDistance * stoppingDistance)
        {
            float interpolationValue = 1 - (target.position - rigidBody.position).magnitude / stoppingDistance;
            vDesired = Vector3.Lerp((target.position - rigidBody.position).normalized * maxSpeed, Vector3.zero, interpolationValue);

        }
        else
        {
            vDesired = vDesiredMaxSpeed;
        }

        vSteer = vDesired - rigidBody.velocity;

        if (vSteer.sqrMagnitude > maxForce * maxForce)
        {
            vSteer = vSteer.normalized * maxForce;
        }

        rigidBody.AddForce(vSteer, ForceMode.VelocityChange);
    }

    void FixedUpdate()
    {
        //Seek();
        SeekArrive();

    }

    void OnDrawGizmos()
    {
        if (rigidBody != null)
        {
            // Black: current velocity
            Gizmos.color = Color.black;
            Gizmos.DrawLine(rigidBody.position, rigidBody.position + rigidBody.velocity);

            // Red: desired velocity
            Gizmos.color = Color.red;
            Gizmos.DrawLine(rigidBody.position, rigidBody.position + vDesired);

            // Blue: steering velocity
            Gizmos.color = Color.blue;
            Gizmos.DrawLine(rigidBody.position, rigidBody.position + vSteer);
        }
    }
}

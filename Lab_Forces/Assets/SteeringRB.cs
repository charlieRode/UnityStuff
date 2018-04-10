using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SteeringRB : MonoBehaviour {

    public Transform A;
    public Transform B;
    public float pathRadius;

    public float maxSpeed;
    public float maxForce;

    public float PREDICT_AHEAD;
    public float NORMAL_AHEAD;

    // arrival
    public float stoppingDistance;

    Rigidbody rigidBody;
    Vector3 vDesired;
    Vector3 vSteer;

    Vector3 normalPoint;
    Vector3 pPredict;

    Vector3 theTarget;

    void Start()
    {
        rigidBody = GetComponent<Rigidbody>();
        if (rigidBody == null)
        {
            Debug.Log("No Rigidbody Detected!");
        }

    }

    void Seek(Vector3 target)
    {
        vDesired = (target - rigidBody.position).normalized * maxSpeed;
        vSteer = vDesired - rigidBody.velocity;

        if (vSteer.sqrMagnitude > Mathf.Pow(maxForce, 2))
        {
            vSteer = vSteer.normalized * maxForce;
        }

        rigidBody.AddForce(vSteer, ForceMode.VelocityChange);
    }

    void SeekArrive(Vector3 target)
    {
        Vector3 vDesiredMaxSpeed = (target - rigidBody.position).normalized * maxSpeed;

        if ((target - rigidBody.position).sqrMagnitude < stoppingDistance * stoppingDistance)
        {
            float interpolationValue = 1 - (target - rigidBody.position).magnitude / stoppingDistance;
            vDesired = Vector3.Lerp((target - rigidBody.position).normalized * maxSpeed, Vector3.zero, interpolationValue);

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


    // As straighforward an implimentation of the algorithm described in The Nature of Code video as I can manage.
    void FollowPath()
    {
        if (A != null && B != null)
        {
            Vector3 vPredict = rigidBody.velocity;
            vPredict = vPredict.normalized * PREDICT_AHEAD;
            pPredict = rigidBody.position + vPredict;

            normalPoint = A.position + Vector3.Project(pPredict - A.position, (B.position - A.position).normalized);
            Vector3 dir = (B.position - A.position).normalized;
            Vector3 target = normalPoint + dir * NORMAL_AHEAD;

            float distance = Vector3.Distance(pPredict, normalPoint);
            if (distance > pathRadius)
            {
                Seek(target);
            }
            else
            {
                //Move();
                //rigidBody.velocity = transform.forward * 5f;
                Seek(pPredict);
            }
        }
    }

    void Move()
    {
        Vector3 vDesired = transform.forward * maxSpeed;
        vSteer = vDesired - rigidBody.velocity;

        if (vSteer.sqrMagnitude > Mathf.Pow(maxForce, 2))
        {
            vSteer = vSteer.normalized * maxForce;
        }

        rigidBody.AddForce(vSteer, ForceMode.VelocityChange);
    }

    void FixedUpdate()
    {

        if (rigidBody.velocity != Vector3.zero)
        {
            Vector3 lookDirection = new Vector3(rigidBody.velocity.x, 0, rigidBody.velocity.z);
            rigidBody.rotation = Quaternion.LookRotation(lookDirection);
        }
        
    }

    void FollowCursor()
    {
        Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
        RaycastHit rHit;
        if (Physics.Raycast(ray, out rHit))
        {
            theTarget = rHit.point;
            Seek(theTarget);
        }
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
            Gizmos.DrawSphere(pPredict, 0.25f);

            // Blue: steering velocity
            Gizmos.color = Color.blue;
            Gizmos.DrawLine(rigidBody.position, rigidBody.position + vSteer);

            Gizmos.color = Color.green;
            Gizmos.DrawSphere(normalPoint, 0.25f);
            //Gizmos.DrawSphere(theTarget, 0.25f);
        }
    }
}

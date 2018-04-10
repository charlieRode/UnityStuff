using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Steering : MonoBehaviour {

    public Transform target;
    public float maxSpeed;
    public float maxForce;
    public float stoppingDistance;

    Vector3 vCurrent;
    Vector3 vDesired;

    void Update()
    {
        //Seek();
        SeekArrive();
    }

    void Seek()
    {
        Vector3 vDesired = (target.position - transform.position).normalized * maxSpeed;
        Vector3 vSteer = vDesired - vCurrent;

        if (vSteer.magnitude > maxForce)
        {
            vSteer = vSteer.normalized * maxForce;
        }

        vCurrent += vSteer;
        transform.Translate(vCurrent * Time.deltaTime);
    }

    void SeekArrive()
    {
        Vector3 vDesiredMaxSpeed = (target.position - transform.position).normalized * maxSpeed;

        if ((target.position - transform.position).sqrMagnitude < stoppingDistance * stoppingDistance)
        {
            float interpolationValue = 1 - (target.position - transform.position).magnitude / stoppingDistance;
            vDesired = Vector3.Lerp(vDesiredMaxSpeed, Vector3.zero, interpolationValue);
        }
        else
        {
            vDesired = vDesiredMaxSpeed;
        }

        Vector3 vSteer = vDesired - vCurrent;

        if (vSteer.sqrMagnitude > maxForce * maxForce)
        {
            vSteer = vSteer.normalized * maxForce;
        }

        vCurrent += vSteer;
        transform.Translate(vCurrent * Time.deltaTime);

    }
}

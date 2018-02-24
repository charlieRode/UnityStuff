using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent (typeof(Rigidbody))]
public class PlayerController : MonoBehaviour {

    Rigidbody rBody;
    Vector3 velocity;

    float lookRotation;
    void Start()
    {
        rBody = GetComponent<Rigidbody>();
    }

	public void Move(Vector3 _velocity)
    {
        velocity = _velocity;
    }

    public void LookAt(Vector3 _point)
    {
        Vector3 lookPosition = (_point - transform.position).normalized;
        lookRotation = Mathf.Atan2(lookPosition.x, lookPosition.z) * Mathf.Rad2Deg;
    }

    void FixedUpdate()
    {
        rBody.MovePosition(rBody.position + velocity * Time.fixedDeltaTime);
        rBody.rotation = Quaternion.Euler(Vector3.up * lookRotation);
    }
}

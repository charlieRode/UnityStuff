using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class PlayerCtrl : MonoBehaviour {

    public bool moveWhileRotating;
    public bool rotateToMove;

    public float rotationSpeed;
    public float moveSpeed;

    public float rotateTowardsSpeed;
    public float angleThreshold;

    Rigidbody rb;

    Vector3 targetPosition;

    IEnumerator currentMoveCoroutine;
    IEnumerator currentRotateCoroutine;
    IEnumerator currentRotateThenMoveCoroutine;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
    }

    void Update()
    {
        if (Input.GetMouseButtonDown(0))
        {
            Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
            RaycastHit hit;
            if (Physics.Raycast(ray, out hit))
            {
                targetPosition = hit.point += (Vector3.up * 0.5f); //set target position hieght equal to the trasnform's height;
                StopCoroutine("RotateTowards");
                StartCoroutine("RotateTowards");
            }
        }


    }

    void MoveToTargetPosition()
    {
        if (rotateToMove)
        {
            if (moveWhileRotating)
            {
                if (currentRotateCoroutine != null)
                    StopCoroutine(currentRotateCoroutine);

                if (currentMoveCoroutine != null)
                    StopCoroutine(currentMoveCoroutine);

                currentRotateCoroutine = RotateToFace();
                currentMoveCoroutine = MoveToTarget();

                StartCoroutine(currentRotateCoroutine);
                StartCoroutine(currentMoveCoroutine);
            }
            else
            {
                if (currentRotateThenMoveCoroutine != null)
                    StopCoroutine(currentRotateThenMoveCoroutine);
                if (currentMoveCoroutine != null)
                    StopCoroutine(currentMoveCoroutine);

                currentRotateThenMoveCoroutine = RotateThenMoveToTarget();
                StartCoroutine(currentRotateThenMoveCoroutine);
            }
        }
        else
        {
            StopCoroutine("MoveToTarget");
            StartCoroutine("MoveToTarget");
        }
    }

    IEnumerator TurnToFace()
    {
        Vector3 _dir = (targetPosition - transform.position).normalized;
        float _theta = Mathf.Atan2(_dir.x, _dir.z) * Mathf.Rad2Deg;

        while (Mathf.Abs(Mathf.DeltaAngle(transform.eulerAngles.y, _theta)) > 0.05f)
        {
            float rotationStep = Mathf.MoveTowardsAngle(transform.eulerAngles.y, _theta, rotationSpeed * Time.fixedDeltaTime);
            rb.MoveRotation(Quaternion.Euler(Vector3.up * rotationStep));
            yield return new WaitForFixedUpdate();
        }
    }

    IEnumerator RotateThenMoveToTarget()
    {
        Vector3 targetDirection = (targetPosition - transform.position).normalized;
        Quaternion lookRotation = Quaternion.FromToRotation(Vector3.forward, targetDirection);
        Quaternion initalRotation = transform.rotation;
        float percent = 0f;

        while (percent < 1)
        {
            percent = Mathf.Clamp01(percent + rotationSpeed * Time.fixedDeltaTime);
            rb.MoveRotation(Quaternion.Slerp(initalRotation, lookRotation, percent));
            yield return new WaitForFixedUpdate();
        }

        currentMoveCoroutine = MoveToTarget();
        yield return StartCoroutine(currentMoveCoroutine);
    }

    IEnumerator RotateToFace()
    {
        Vector3 targetDirection = (targetPosition - transform.position).normalized;
        Quaternion lookRotation = Quaternion.FromToRotation(Vector3.forward, targetDirection);
        Quaternion initalRotation = transform.rotation;
        float percent = 0f;

        while (percent < 1)
        {
            percent = Mathf.Clamp01(percent + rotationSpeed * Time.fixedDeltaTime);
            rb.MoveRotation(Quaternion.Slerp(initalRotation, lookRotation, percent));
            yield return new WaitForFixedUpdate();
        }

        if (!moveWhileRotating)
            yield return StartCoroutine(MoveToTarget());
    }

    IEnumerator RotateTowards()
    {
        Vector3 targetDirection = (targetPosition - transform.position).normalized;
        Quaternion lookRotation = Quaternion.FromToRotation(Vector3.forward, targetDirection);
        
        while ((transform.rotation.eulerAngles - lookRotation.eulerAngles).sqrMagnitude > Mathf.Pow(angleThreshold, 2))
        {
            Debug.Log((transform.rotation.eulerAngles - lookRotation.eulerAngles).magnitude);

            rb.MoveRotation( Quaternion.RotateTowards(transform.rotation, lookRotation, rotateTowardsSpeed * Time.fixedDeltaTime) );
            yield return new WaitForFixedUpdate();
        }
        yield return null;
    }

    IEnumerator MoveToTarget()
    {
        while (transform.position != targetPosition)
        {
            Vector3 newPosition = Vector3.MoveTowards(transform.position, targetPosition, moveSpeed * Time.fixedDeltaTime);
            rb.MovePosition(newPosition);
            yield return new WaitForFixedUpdate();
        }

    }

}

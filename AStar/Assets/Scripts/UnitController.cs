using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class UnitController : MonoBehaviour {

    public float movementSpeed;
    public float closeEnough;

    public bool moveWhileRotating;
    public bool rotateToMove;

    public bool usePathFinding;

    public LayerMask groundLayer;

    // Debug purposes
    public Transform target;
    

    Rigidbody rb;
    Vector3 destination;

    public Transform navPoint;
    public Transform cubeBody;
    Vector3 verticalOffset;

    // waypoints to target
    Vector3[] pathToDestination;
    Vector3 targetPosition;
    int waypointIdx;
    bool isFollowingPath;

    public enum RotationLogic { SingleAxisConstantSpeed, MultiAxisConstantSpeed, MultiAxisConstantTime, LookRotation };
    public RotationLogic rotationLogic;

    [Header("SingleAxisConstantSpeed Settings")]
    public float singleAxisTurnSpeed;
    public float moveAngleThreshold;

    [Header("MultiAxisConstantSpeed Settings")]
    public float angleThreshold;
    public float rotateTowardsSpeed;

    [Header("MultiAxisConstantTime Settings")]
    public float turnSpeed;

    delegate IEnumerator rotateRoutine();
    rotateRoutine rotationRoutine;

    IEnumerator currentMoveCoroutine;
    IEnumerator currentRotateCoroutine;

    IEnumerator currentMoveToWaypointCoroutine;
    IEnumerator currentFollowPathCoroutine;

    void OnDrawGizmos()
    {
        Gizmos.color = Color.green;
        Gizmos.DrawCube(navPoint.position, Vector3.one * 0.25f);
        if (pathToDestination != null)
        {
            for (int i = waypointIdx; i < pathToDestination.Length; i++)
            {
                Gizmos.color = Color.black;
                //Gizmos.DrawSphere(pathToDestination[i] + verticalOffset, 0.2f);
                Gizmos.DrawCube(pathToDestination[i], Vector3.one * 0.25f);

                
                if (i == waypointIdx)
                {
                    Gizmos.DrawLine(cubeBody.position, pathToDestination[i]);
                }
                else
                {
                    Gizmos.DrawLine(pathToDestination[i - 1], pathToDestination[i]);
                }
                
            }
        }
    }

    void Awake()
    {
        switch(rotationLogic)
        {
            case RotationLogic.SingleAxisConstantSpeed:
                rotationRoutine = TurnToFace;
                break;

            case RotationLogic.MultiAxisConstantSpeed:
                rotationRoutine = RotateTowards;
                break;

            case RotationLogic.MultiAxisConstantTime:
                rotationRoutine = RotateToFace;
                break;

            case RotationLogic.LookRotation:
                rotationRoutine = LookToFace;
                break;

            default:
                rotationRoutine = null;
                print("No Recognized Rotation Routine!");
                break;
        }
    }

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        //PathRequestManager.RequestPath(transform.position, target.position, OnPathFound);
        verticalOffset = Vector3.up;
        print(verticalOffset);
    }

    void Update()
    {
        if (Input.GetMouseButtonDown(0))
        {
            Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
            RaycastHit hit;
            if (Physics.Raycast(ray, out hit, 1000, groundLayer))
            {
                if (usePathFinding)
                {
                    //targetPosition = hit.point += (Vector3.up * 0.5f); //set target position height equal to the trasnform's height;
                    destination = hit.point;
                    PathRequestManager.RequestPath(transform.position, destination, OnPathFound);
                }
                else
                {
                    targetPosition = hit.point + verticalOffset;
                    MoveToTargetPosition();
                }
            }
        }
    }

    void OnPathFound(Vector3[] _path, bool success)
    {
        if (success)
        {
            pathToDestination = _path;
            waypointIdx = 0;

            if (currentFollowPathCoroutine != null)
                StopCoroutine(currentFollowPathCoroutine);

            currentFollowPathCoroutine = FollowPath();
            StartCoroutine(currentFollowPathCoroutine);
        }
    }

    IEnumerator FollowPath()
    {
        while (waypointIdx < pathToDestination.Length)
        {
            targetPosition = pathToDestination[waypointIdx];
            if (currentMoveToWaypointCoroutine != null)
                StopCoroutine(currentMoveToWaypointCoroutine);

            currentMoveToWaypointCoroutine = MoveToWaypointPosition();
            yield return StartCoroutine(currentMoveToWaypointCoroutine);
        }

    }

    // To move without pathfinding.
    void MoveToTargetPosition()
    {
        if (rotateToMove)
        {
            if (currentRotateCoroutine != null)
                StopCoroutine(currentRotateCoroutine);
            if (currentMoveCoroutine != null)
                StopCoroutine(currentMoveCoroutine);

            currentRotateCoroutine = rotationRoutine();
            currentMoveCoroutine = MoveToWaypoint();

            if (moveWhileRotating)
            {
                StartCoroutine(currentRotateCoroutine);
                StartCoroutine(currentMoveCoroutine);
            }
            else
            {
                StartCoroutine(currentRotateCoroutine);
            }

        }
        else
        {
            StopCoroutine("MoveToWaypoint");
            StartCoroutine("MoveToWaypoint");
        }
    }

    IEnumerator MoveToWaypointPosition()
    {
        if (rotateToMove)
        {
            if (currentRotateCoroutine != null)
                StopCoroutine(currentRotateCoroutine);
            if (currentMoveCoroutine != null)
                StopCoroutine(currentMoveCoroutine);

            currentRotateCoroutine = rotationRoutine();
            currentMoveCoroutine = MoveToWaypoint();

            if (moveWhileRotating)
            {
                StartCoroutine(currentRotateCoroutine);
                yield return StartCoroutine(currentMoveCoroutine);
            }
            else
            {
                yield return StartCoroutine(currentRotateCoroutine);
                yield return StartCoroutine(currentMoveCoroutine);
            }

        }
        else
        {
            StopCoroutine("MoveToWaypoint");
            yield return StartCoroutine("MoveToWaypoint");
        }
    }

    IEnumerator TurnToFace()
    {
        Vector3 _dir = (targetPosition - transform.position).normalized;
        float _theta = Mathf.Atan2(_dir.x, _dir.z) * Mathf.Rad2Deg;

        while (Mathf.Abs(Mathf.DeltaAngle(transform.eulerAngles.y, _theta)) > moveAngleThreshold)
        {
            float rotationStep = Mathf.MoveTowardsAngle(transform.eulerAngles.y, _theta, singleAxisTurnSpeed * Time.fixedDeltaTime);
            rb.MoveRotation(Quaternion.Euler(Vector3.up * rotationStep));
            yield return new WaitForFixedUpdate();
        }
    }

    IEnumerator LookToFace()
    {
        Vector3 targetDirection = (targetPosition - transform.position).normalized;
        Vector3 localUp = Vector3.up;
        RaycastHit hit;
        if (Physics.Raycast(transform.position, Vector3.down, out hit, 10f, groundLayer))
        {
            localUp = hit.normal;
        }
        Quaternion targetRotation = Quaternion.LookRotation(targetDirection, localUp);
        while ((transform.rotation.eulerAngles - targetRotation.eulerAngles).sqrMagnitude > Mathf.Pow(angleThreshold, 2))
        {
            rb.MoveRotation(Quaternion.RotateTowards(transform.rotation, targetRotation, rotateTowardsSpeed * Time.fixedDeltaTime));
            yield return new WaitForFixedUpdate();
        }
    }


    IEnumerator RotateToFace()
    {
        Vector3 targetDirection = (targetPosition - transform.position).normalized;
        Quaternion lookRotation = Quaternion.FromToRotation(Vector3.forward, targetDirection);
        Quaternion initalRotation = transform.rotation;
        float percent = 0f;

        while (percent < 1)
        {
            percent = Mathf.Clamp01(percent + turnSpeed * Time.fixedDeltaTime);
            rb.MoveRotation(Quaternion.Slerp(initalRotation, lookRotation, percent));
            yield return new WaitForFixedUpdate();
        }
    }

    IEnumerator RotateTowards()
    {
        Vector3 targetDirection = (targetPosition - transform.position).normalized;
        Quaternion lookRotation = Quaternion.FromToRotation(Vector3.forward, targetDirection);

        while ((transform.rotation.eulerAngles - lookRotation.eulerAngles).sqrMagnitude > Mathf.Pow(angleThreshold, 2))
        {
            rb.MoveRotation(Quaternion.RotateTowards(transform.rotation, lookRotation, rotateTowardsSpeed * Time.fixedDeltaTime));
            yield return new WaitForFixedUpdate();
        }
    }

    IEnumerator MoveToWaypoint()
    {
        while ((targetPosition - navPoint.position).sqrMagnitude > Mathf.Pow(closeEnough, 2))
        {
            Vector3 newPosition = Vector3.MoveTowards(transform.position, targetPosition + verticalOffset, movementSpeed * Time.fixedDeltaTime);
            rb.MovePosition(newPosition);
            yield return new WaitForFixedUpdate();
        }

        waypointIdx++;
    }
}

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class UnitControllerV2 : MonoBehaviour
{
    public float movementSpeed;
    public float accelerationRate;
    public float closeEnough;

    public bool moveWhileRotating;
    public bool rotateToMove;

    public bool usePathFinding;

    public bool useSeek;

    public LayerMask groundLayer;

    Rigidbody rb;
    Vector3 destination;

    public Transform navPoint;
    Vector3 verticalOffset;

    // waypoints to target
    //Vector3[] pathToDestination;
    //int waypointIdx;
    Path currentPath;
    float currentSpeed;

    public float singleAxisTurnSpeed;
    public float moveAngleThreshold;

    // The distance needed to fully decelerate when traveling at top speed (movementSpeed)
    float decelerationDistance;

    // The distance needed to fully decelerate for the currentPath;
    float stoppingDistance;

    IEnumerator currentMoveCoroutine;
    IEnumerator currentRotateCoroutine;

    IEnumerator currentMoveToWaypointCoroutine;
    IEnumerator currentFollowPathCoroutine;

    Vector3 vDesired;
    Vector3 vSteer;
    Transform target;
    public float maxForce;
    

    void OnDrawGizmos()
    {
        Gizmos.color = Color.green;
        Gizmos.DrawCube(navPoint.position, Vector3.one * 0.25f);
        if (currentPath != null)
        {
            for (int i = currentPath.waypointIdx; i < currentPath.waypoints.Length; i++)
            {
                Gizmos.color = Color.black;
                //Gizmos.DrawSphere(pathToDestination[i] + verticalOffset, 0.2f);
                Gizmos.DrawCube(currentPath.waypoints[i], Vector3.one * 0.25f);

                if (i == currentPath.waypointIdx)
                {
                    Gizmos.DrawLine(navPoint.position, currentPath.waypoints[i]);
                }
                else
                {
                    Gizmos.DrawLine(currentPath.waypoints[i - 1], currentPath.waypoints[i]);
                }
            }
        }
    }

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        verticalOffset = Vector3.up;
        
        // dx = -(V0^2) / 2a
        decelerationDistance = -Mathf.Pow(movementSpeed, 2) / (2 * -accelerationRate);
    }

    void Update()
    {
        //print(Time.time + ": " + currentSpeed);
        if (Input.GetMouseButtonDown(0))
        {
            Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
            RaycastHit hit;
            if (Physics.Raycast(ray, out hit, 1000, groundLayer))
            {
                destination = hit.point;
                PathRequestManager.RequestPath(transform.position, destination, OnPathFound);
            }
        }
    }

    void OnPathFound(Vector3[] _path, bool success)
    {
        if (success && _path.Length > 0)
        {
            currentPath = new Path(_path, navPoint.position);

            // Because the pathfinder doesn't immediately return a path (and the delay can get relatively long if many requests are in the queue), the unit may not be in the same position
            // it was in when the request was made, and the unit will be somewhat displaced from the starting node of the path. To avoid a kind of 'backtracking' behavior that can arise from
            // this, once the unit recieves its path, we use the same logic as in the pathfinding script to check to see if the SECOND point in the path is walkable from the unit's current position.
            // If it is, we can skip the first node in the path. If it isn't, then the backtracking behavior is acceptable, and in fact, ideal.         
            if (_path.Length > 1 && Walkable(navPoint.position, _path[1], 0.25f))
                currentPath.waypointIdx++;

            stoppingDistance = currentPath.pathDistance < decelerationDistance * 2 ? currentPath.pathDistance / 2f : decelerationDistance;

            if (currentFollowPathCoroutine != null)
                StopCoroutine(currentFollowPathCoroutine);

            currentFollowPathCoroutine = FollowPath();
            StartCoroutine(currentFollowPathCoroutine);
        }
    }

    IEnumerator FollowPath()
    {
        while (currentPath.waypointIdx < currentPath.waypoints.Length)
        {
            if (currentMoveToWaypointCoroutine != null)
                StopCoroutine(currentMoveToWaypointCoroutine);

            currentMoveToWaypointCoroutine = MoveToWaypointPosition();
            yield return StartCoroutine(currentMoveToWaypointCoroutine);
        }

        currentSpeed = 0f;
    }

    IEnumerator MoveToWaypointPosition()
    {
        if (rotateToMove)
        {
            if (currentRotateCoroutine != null)
                StopCoroutine(currentRotateCoroutine);
            if (currentMoveCoroutine != null)
                StopCoroutine(currentMoveCoroutine);

            currentRotateCoroutine = TurnToFace();
            currentMoveCoroutine = useSeek ? MoveSeek() : Move();

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
            if (useSeek)
            {
                StopCoroutine("MoveSeek");
                yield return StartCoroutine("MoveSeek");
            }
            else
            {
                StopCoroutine("Move");
                yield return StartCoroutine("Move");
            }

        }
    }

    IEnumerator TurnToFace()
    {
        Vector3 _dir = (currentPath.CurrentWaypoint - navPoint.position).normalized;
        float _theta = Mathf.Atan2(_dir.x, _dir.z) * Mathf.Rad2Deg;

        while (Mathf.Abs(Mathf.DeltaAngle(transform.eulerAngles.y, _theta)) > moveAngleThreshold)
        {
            float rotationStep = Mathf.MoveTowardsAngle(transform.eulerAngles.y, _theta, singleAxisTurnSpeed * Time.fixedDeltaTime);
            rb.MoveRotation(Quaternion.Euler(Vector3.up * rotationStep));
            yield return new WaitForFixedUpdate();
        }
    }

    bool IsWithinStoppingDistance()
    {
        float remainingDistance = (currentPath.CurrentWaypoint - navPoint.position).magnitude;
        for (int i = currentPath.waypointIdx + 1; i < currentPath.waypoints.Length; i++)
        {
            remainingDistance += (currentPath.waypoints[i] - currentPath.waypoints[i - 1]).magnitude;
        }
        return remainingDistance <= Mathf.Pow(decelerationDistance, 2);
    }

    bool Walkable(Vector3 A, Vector3 B, float checkRadius)
    {
        RaycastHit rHit1, rHit2;
        Vector3 step = (B - A).normalized * checkRadius * 2f; // multiply by 2 to check every point instead of every half point to improve performance.
        Vector3 checkPoint = A + Vector3.up * (checkRadius + PathFindingV2.Instance.sphereColliderBuffer);
        while (Mathf.Abs(checkPoint.x - B.x) > checkRadius || Mathf.Abs(checkPoint.z - B.z) > checkRadius)
        {
            // First check that at each point, the delta between the point on the ground and the starting point on the ground is at most the maxStepHeight
            // Expanding to two check point on either edge of the check sphere. This will account for the unit's width when raycasting down.

            // The two points on the left and right (local) poles of the check sphere are given by the calculations below.
            Vector3 checkPoint1 = checkPoint + new Vector3(step.z, step.y, -step.x).normalized * checkRadius;
            Vector3 checkPoint2 = checkPoint + new Vector3(-step.z, step.y, step.x).normalized * checkRadius;
            if (Physics.Raycast(checkPoint1, Vector3.down, out rHit1, 100, groundLayer) && Physics.Raycast(checkPoint2, Vector3.down, out rHit2, 100, groundLayer))
            {
                if (checkPoint.y - rHit1.point.y > PathFindingV2.Instance.maxStepHeight || checkPoint.y - rHit2.point.y > PathFindingV2.Instance.maxStepHeight)
                    return false;
            }

            // Next we check that the sphere of checkRadius at each checkPoint along the line does not collide with anything.
            if (Physics.CheckSphere(checkPoint, checkRadius, PathFindingV2.Instance.obsticleLayer))
                return false;

            checkPoint += step;
        }

        return true;
    }

    IEnumerator Move()
    {
        while ((currentPath.CurrentWaypoint - navPoint.position).sqrMagnitude > Mathf.Pow(closeEnough, 2))
        {
            if (currentPath.RemainingDistanceInPath <= stoppingDistance)
            {
                currentSpeed = Mathf.Clamp(currentSpeed - accelerationRate * Time.fixedDeltaTime, 0, movementSpeed);
            }
            else if (currentSpeed < movementSpeed)
            {
                currentSpeed = Mathf.Clamp(currentSpeed + accelerationRate * Time.fixedDeltaTime, 0, movementSpeed);
            }

            Vector3 newPosition = Vector3.MoveTowards(transform.position, currentPath.CurrentWaypoint + verticalOffset, currentSpeed * Time.fixedDeltaTime);
            rb.MovePosition(newPosition);
            currentPath.distanceTraveled += currentSpeed * Time.fixedDeltaTime;
            yield return new WaitForFixedUpdate();
        }

        currentPath.waypointIdx++;
    }

    /*
    void Seek()
    {
        vDesired = (currentPath.CurrentWaypoint + verticalOffset - rb.position).normalized * movementSpeed;
        vSteer = vDesired - rb.velocity;

        if (vSteer.sqrMagnitude > Mathf.Pow(movementSpeed, 2))
        {
            vSteer = vSteer.normalized * maxForce;
        }

        rb.AddForce(vSteer, ForceMode.VelocityChange);
    }
    */

    IEnumerator MoveSeek()
    {
        while((currentPath.CurrentWaypoint - navPoint.position).sqrMagnitude > Mathf.Pow(closeEnough, 2))
        {
            Vector3 desiredVelocity = (currentPath.CurrentWaypoint - navPoint.position).normalized * movementSpeed;
            Vector3 vSteer = desiredVelocity - rb.velocity;

            if (vSteer.sqrMagnitude > movementSpeed * movementSpeed)
            {
                vSteer = vSteer.normalized * maxForce;
            }

            rb.AddForce(vSteer, ForceMode.VelocityChange);
            yield return new WaitForFixedUpdate();
        }

        currentPath.waypointIdx++;
    }

    class Path
    {
        public Vector3[] waypoints;
        public int waypointIdx;
        public float pathDistance;
        public float distanceTraveled = 0f;

        public Path(Vector3[] _path, Vector3 startPosition)
        {
            waypoints = _path;
            waypointIdx = 0;
            float _distance = Vector3.Distance(startPosition, waypoints[0]);
            for (int i = 1; i < waypoints.Length; i++)
            {
                _distance += (waypoints[i] - waypoints[i - 1]).magnitude;
            }
            pathDistance = _distance;
        }

        public Vector3 CurrentWaypoint
        {
            get { return waypoints[waypointIdx]; }
        }

        public float RemainingDistanceInPath
        {
            get { return pathDistance - distanceTraveled; }
        }
        
    }
}

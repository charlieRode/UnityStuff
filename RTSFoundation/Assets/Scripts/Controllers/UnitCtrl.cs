using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using UnityEngine;

// The newest iteration of the unit controller. We are scrapping the rotation logic entirely,
// going for a pure steering-behavior approach. The rotation of the unit should always be in the direction
// of the velocity vector.


[RequireComponent(typeof(Rigidbody))]
public class UnitCtrl : MonoBehaviour, IBoid, IObsticle
{
    // Testing Steering Behaviors
    public Transform fleeTarget;
    public Transform seekTarget;
    public UnitCtrl pursueTarget;
    bool initializedVelocity = false;

    Vector3 vSteer;

    Vector3 pNormal;
    Vector3 currentTarget;
    Vector3 vPredict;
    Vector3 prevWaypoint;

    public float FACE_VELOCITY_THREASHOLD_MAGNITUDE;
    public float FACE_VELOCITY_THREASHOLD_DELTA_ANGLE;
    Vector3 prevVelocity;

    public bool doCollisionAvoidance;
    public bool usePathFollowingAlgo;
    public bool useSeparation;

    public UnitCtrl evadeTarget;

    // For Path Following
    public float PREDICT_AHEAD;
    public float NORMAL_AHEAD;

    public float fallingDistance;
    bool isGrounded;
    Vector3 groundNormal;

    public float pathRadius;

    public float FallDistance
    {
        get
        {
            return fallingDistance;
        }
    }

    public bool doCA
    {
        get
        {
            return doCollisionAvoidance;
        }
    }

    public Vector3 Center
    {
        get
        {
            return transform.position;
        }
    }

    public float collisionRadius;
    public float CollisionRadius
    {
        get
        {
            return collisionRadius;
        }
    }

    public string Name
    {
        get
        {
            return gameObject.name;
        }
    }

    public Vector3 Velocity
    {
        get
        {
            if (rb != null)
            {
                // We want to ignore velocity not along the ground plane. Specifically, we are negating
                // the velocity component affected by gravity.
                // Since we are resetting the transform position to the vertical offset each frame,
                // the unit is effectively continuously falling, and this shows up in rigidbody.velocity.
                return Vector3.ProjectOnPlane(rb.velocity, groundNormal);
            }

            return Vector3.zero;
        }
    }

    public float MaxSpeed
    {
        get
        {
            return movementSpeed;
        }
    }

    public float MaxForce
    {
        get
        {
            return maxForce;
        }
    }

    public float SlowingDistance
    {
        get
        {
            return slowingDistance;
        }
    }

    public float StoppingDistance
    {
        get
        {
            return pathRadius;
        }
    }

    public Transform TransformPoint
    {
        get
        {
            return transform;
        }
    }

    public Transform NavPoint
    {
        get
        {
            return navPoint;
        }
    }

    public Vector3 GroundNormal
    {
        get
        {
            return groundNormal;
        }
    }

    private bool IsGrounded
    {
        get
        {
            return isGrounded;
        }

        set
        {
            isGrounded = value;
            rb.useGravity = !value;
            LockRotation(value);
        }
    }

    public void ScaleVelocity(float scalar)
    {
        rb.velocity *= scalar;
    }

    public GameObject arrow;
    public float maxForce;
    public float movementSpeed;
    public float slowingDistance;

    public float verticalOffset; // distance off ground.

    MovementManager steeringManager;

    public LayerMask groundLayer;

    Rigidbody rb;
    Vector3 destination;

    public Transform navPoint;

    Path currentPath;
    float currentSpeed;

    IEnumerator pathFollowRoutine;

    bool isSelected = false;

    void OnDrawGizmos()
    {

        Gizmos.color = Color.green;
        Gizmos.DrawLine(transform.position, transform.position + vSteer * 10f);
        Gizmos.color = Color.red;
        Gizmos.DrawLine(transform.position, transform.position + Velocity);

        Gizmos.color = Color.magenta;
        Gizmos.DrawLine(transform.position, transform.position + steeringManager.vSteerFlee);
        Gizmos.color = Color.cyan;
        Gizmos.DrawLine(transform.position, transform.position + steeringManager.vSteerSeek);

        if (currentPath != null)
        {
            for (int i = currentPath.waypointIdx; i < currentPath.waypoints.Length; i++)
            {
                Gizmos.color = Color.black;
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

    void FixedUpdate()
    {   
        // Testing Steering Behaviors
        /*
        if (fleeTarget != null)
        {
            steeringManager.Flee(fleeTarget.position);
        }
        if (seekTarget != null)
        {
            steeringManager.Seek(seekTarget.position);
        }
        if (pursueTarget != null)
        {
            steeringManager.Pursue(pursueTarget);
        }
        if (evadeTarget != null)
        {
            steeringManager.Evade(evadeTarget);
        }
        */

        if (doCollisionAvoidance)
        {
            steeringManager.CollisionAvoidance();
        }

        Ray ray = new Ray(rb.position, Vector3.down);
        RaycastHit rHit;
        if (Physics.Raycast(ray, out rHit, 100, groundLayer))
        {
            groundNormal = rHit.normal;

            // Initiate Falling Condition
            if (rHit.distance >= fallingDistance)
            {
                IsGrounded = false;
            }

            // Terminate Falling Condition
            if (!IsGrounded && rHit.distance < fallingDistance)
            {
                IsGrounded = true;

                // Must reset velocity vector otherwise it will continue to point downward from falling.
                rb.velocity = Vector3.zero;
            }

            // Only allow steering forces to be applied if Grounded.
            // While Grounded, if the unit height deviates from the vertical offset, reset it.
            if (IsGrounded)
            {
                if (rHit.distance != verticalOffset)
                {
                    rb.MovePosition(new Vector3(rb.position.x, rHit.point.y + verticalOffset, rb.position.z));
                    rb.velocity = new Vector3(rb.velocity.x, 0, rb.velocity.z);
                }

                if (useSeparation)
                    steeringManager.Separate();

                steeringManager.Flock();

                /* FOLLOW MOUSE POINT
                RaycastHit mousePoint;
                if (Physics.Raycast(Camera.main.ScreenPointToRay(Input.mousePosition), out mousePoint, 100f, groundLayer))
                {
                    steeringManager.Seek(mousePoint.point);
                }
                */

                vSteer = steeringManager.Steering;
                rb.AddForce(vSteer, ForceMode.VelocityChange);
                
            }
        }
        else
        {
            UnityEngine.Debug.Log("ERR: Raycast did not intersect with Ground Layer");
        }

        // Face in the direction of movement. 
        if (rb.velocity.sqrMagnitude > FACE_VELOCITY_THREASHOLD_MAGNITUDE * FACE_VELOCITY_THREASHOLD_MAGNITUDE && Vector3.Angle(rb.velocity, prevVelocity) > FACE_VELOCITY_THREASHOLD_DELTA_ANGLE)
        {
            prevVelocity = rb.velocity;
            Vector3 lookDirection = new Vector3(rb.velocity.x, 0, rb.velocity.z);
            rb.rotation = Quaternion.LookRotation(lookDirection);
        }

    }

    void LockRotation(bool val)
    {
        rb.freezeRotation = val;
    }   

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        steeringManager = GetComponent<MovementManager>();
        steeringManager.Invoke(this);
        GameCtrl.Instance.RegisterActiveUnit(this);
    }

    public void DestroyUnit()
    {
        GameCtrl.Instance.UnregisterActiveUnit(this);
        Destroy(gameObject);
    }

    void RegisterWithOrdersCtrl()
    {
        UnitOrdersCtrl.Instance.moveUnitsConvergeOnPoint += MoveConvergeOnPoint;
        UnitOrdersCtrl.Instance.moveUnitsKeepFormation += MoveKeepFormation;
    }

    void UnregisterWithOrdersCtrl()
    {
        UnitOrdersCtrl.Instance.moveUnitsConvergeOnPoint -= MoveConvergeOnPoint;
        UnitOrdersCtrl.Instance.moveUnitsKeepFormation -= MoveKeepFormation;
    }

    public void Select()
    {
        if (!isSelected)
        {
            isSelected = true;
            arrow.SetActive(true);
            RegisterWithOrdersCtrl();
        }
    }

    public void Deselect()
    {
        if (isSelected)
        {
            isSelected = false;
            arrow.SetActive(false);
            UnregisterWithOrdersCtrl();
        }
    }

    void MoveKeepFormation(Vector3 destination, Vector3 anchor)
    {
        Vector3 relativePosition = transform.position - anchor;
        PathRequestManager.RequestPath(navPoint.position, destination + relativePosition, OnPathFound);
    }

    void MoveConvergeOnPoint(Vector3 destination)
    {
        PathRequestManager.RequestPath(navPoint.position, destination, OnPathFound);
    }

    void OnPathFound(Vector3[] _path, bool success)
    {
        if (success && _path.Length > 0)
        {
            // Because the pathfinder doesn't immediately return a path (and the delay can get relatively long if many requests are in the queue), the unit may not be in the same position
            // it was in when the request was made, and the unit will be somewhat displaced from the starting node of the path. To avoid a kind of 'backtracking' behavior that can arise from
            // this, once the unit recieves its path, we use the same logic as in the pathfinding script to check to see if the SECOND point in the path is walkable from the unit's current position.
            // If it is, we can skip the first node in the path. If it isn't, then the backtracking behavior is acceptable, and in fact, ideal.    

            // GOTCHA: This approach doesn't work well when traveling up a slope. An alternative approach would be to check if the first point in the path is sufficiently close to
            // the current position of the nav point. If it is, skip it.

            //print(Walkable(navPoint.position, _path[1], 0.5f));

            if (_path.Length > 1 && Walkable(navPoint.position, _path[1], 0.5f))
            {
                _path[0] = navPoint.position;
            }

            currentPath = new Path(_path);
            currentPath.radius = pathRadius;

            
            if (pathFollowRoutine != null)
                StopCoroutine(pathFollowRoutine);

            pathFollowRoutine = FollowPath();
            StartCoroutine(pathFollowRoutine);

            // To make path following algo work, must set inital V.
            // rb.velocity = transform.forward * MaxSpeed;
        }
    }

    IEnumerator FollowPath()
    {
        while (currentPath.waypointIdx < currentPath.waypoints.Length)
        {
            if (usePathFollowingAlgo)
            {
                StopCoroutine("MoveToNextWaypointN");
                yield return StartCoroutine("MoveToNextWaypointN");
            }
            else
            {
                StopCoroutine("MoveToNextWaypoint");
                yield return StartCoroutine("MoveToNextWaypoint");
            }
        }

        currentPath = null;
    }

    bool Walkable(Vector3 A, Vector3 B, float checkRadius)
    {
        RaycastHit rHit1, rHit2;
        Vector3 step = (B - A).normalized * checkRadius * 2f; // multiply by 2 to check every point instead of every half point to improve performance.
        Vector3 checkPoint = A + Vector3.up * (checkRadius + PathFinding.Instance.sphereColliderBuffer);
        while (Mathf.Abs(checkPoint.x - B.x) > checkRadius || Mathf.Abs(checkPoint.z - B.z) > checkRadius)
        {
            // First check that at each point, the delta between the point on the ground and the starting point on the ground is at most the maxStepHeight
            // Expanding to two check points on either edge of the check sphere. This will account for the unit's width when raycasting down.

            // The two points on the left and right (local) poles of the check sphere are given by the calculations below.
            Vector3 checkPoint1 = checkPoint + new Vector3(step.z, step.y, -step.x).normalized * checkRadius;
            Vector3 checkPoint2 = checkPoint + new Vector3(-step.z, step.y, step.x).normalized * checkRadius;
            if (Physics.Raycast(checkPoint1, Vector3.down, out rHit1, 100, groundLayer) && Physics.Raycast(checkPoint2, Vector3.down, out rHit2, 100, groundLayer))
            {
                if (checkPoint.y - rHit1.point.y > Grid.Instance.maxStepHeight + checkRadius || checkPoint.y - rHit2.point.y > Grid.Instance.maxStepHeight + checkRadius)
                    return false;
            }

            // Next we check that the sphere of checkRadius at each checkPoint along the line does not collide with anything.
            if (Physics.CheckSphere(checkPoint, checkRadius, PathFinding.Instance.obsticleLayer))
                return false;

            checkPoint += step;
        }

        return true;
    }

    IEnumerator MoveToNextWaypoint()
    {
        while ((currentPath.CurrentWaypoint - navPoint.position).sqrMagnitude > Mathf.Pow(pathRadius, 2))
        {
            if (currentPath.OnLastLeg)
            {
                steeringManager.SeekArrive(currentPath.CurrentWaypoint);
            }
            else
            {
                steeringManager.Seek(currentPath.CurrentWaypoint);
            }
     
            yield return new WaitForFixedUpdate();
        }

        currentPath.waypointIdx++;
    }
    
    IEnumerator MoveToNextWaypointN()
    {
        //steeringManager.Seek(currentPath.CurrentWaypoint);
        //rb.AddForce(steeringManager.Steering, ForceMode.VelocityChange);
        while ((currentPath.CurrentWaypoint - navPoint.position).sqrMagnitude > Mathf.Pow(currentPath.radius, 2))
        {
            //vPredict = navPoint.position + transform.forward * PREDICT_AHEAD;
            vPredict = navPoint.position + Velocity;
            prevWaypoint = currentPath.waypointIdx == 0 ? currentPath.StartPoint : currentPath.waypoints[currentPath.waypointIdx - 1];
            pNormal = prevWaypoint + Vector3.Project(vPredict - prevWaypoint, (currentPath.CurrentWaypoint - prevWaypoint).normalized);

            bool vPredictOnPath = (pNormal - vPredict).sqrMagnitude <= currentPath.SqrRadius;
            if (!vPredictOnPath)
            {
                Vector3 incrementNormal = (currentPath.CurrentWaypoint - prevWaypoint).normalized * NORMAL_AHEAD;
                currentTarget = pNormal + incrementNormal;

                steeringManager.Seek(currentTarget);
            }
            else if (Velocity.sqrMagnitude < MaxSpeed * MaxSpeed)
            {
                Vector3 vAhead = navPoint.position + transform.forward * MaxSpeed;
                steeringManager.Seek(vAhead);
            }

            yield return new WaitForFixedUpdate();
        }

        currentPath.waypointIdx++;
    }
    

    class Path
    {
        public Vector3[] waypoints;
        public int waypointIdx;
        public float radius;

        Vector3 startPoint;

        public Path(Vector3[] _path)
        {
            waypoints = _path;
            waypointIdx = 0;
            startPoint = waypoints[0];
        }

        public Vector3 CurrentWaypoint
        {
            get { return waypoints[waypointIdx]; }
        }

        public Vector3 Destination
        {
            get { return waypoints[waypoints.Length - 1]; }
        }

        public bool OnLastLeg
        {
            get { return CurrentWaypoint == Destination; }
        }

        public Vector3 StartPoint
        {
            get { return startPoint; }
        }

        public float SqrRadius
        {
            get
            {
                return radius * radius;
            }
        }
    }
}

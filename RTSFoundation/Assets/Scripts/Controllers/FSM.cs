using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FSM : MonoBehaviour {

    // experiment. test constructed path
    public bool testMode;
    public Transform[] testWaypoints;

    
    enum AI_STATES
    {
        EN_ROUTE, // has a path, moving toward destination;
        IDLE, // no path, no place to be, man.
        AVOID_FRIENDLY // get out of the way!
    }
    AI_STATES currentState;
    private AI_STATES CurrentState
    {
        get
        {
            return currentState;
        }
        set
        {
            switch (value)
            {
                case AI_STATES.IDLE:
                    SetIdle();
                    break;

                case AI_STATES.EN_ROUTE:
                    SetEnRoute();
                    break;

                case AI_STATES.AVOID_FRIENDLY:
                    SetAvoidFriendly();
                    break;
            }

            currentState = value;
        }
    }

    MovementManager movementManager;
    Path currentPath;
    public float pathRadius;

    IEnumerator pathFollowRoutine;

	void Start ()
    {
        movementManager = GetComponent<MovementManager>();
        CurrentState = AI_STATES.IDLE;

        if (testMode)
        {
            Vector3[] path = new Vector3[testWaypoints.Length];
            for (int i = 0; i < testWaypoints.Length; i++)
            {
                path[i] = testWaypoints[i].position;
            }
            currentPath = new Path(path);

            if (pathFollowRoutine != null)
                StopCoroutine(pathFollowRoutine);

            pathFollowRoutine = FollowPath();
            StartCoroutine(pathFollowRoutine);

            CurrentState = AI_STATES.EN_ROUTE;
        }
	}

    void SetIdle()
    {
        print("STATE: IDLE");
        // idle animation.
        // watch for incoming friendlies.
        movementManager._flockWeight = movementManager.W_FLOCK_IDLE;
    }

    void SetEnRoute()
    {
        print("STATE: EN ROUTE");
        // movement animation.
        movementManager._flockWeight = movementManager.W_FLOCK_ENROUTE;
    }

    void SetAvoidFriendly()
    {
        print("STATE: AVOID FRIENDLY");
        // settimer.
        // set avoid friendly movement weights
        // return to original position and change state to idle
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
            if (Physics.Raycast(checkPoint1, Vector3.down, out rHit1, 100, movementManager.groundLayer) && Physics.Raycast(checkPoint2, Vector3.down, out rHit2, 100, movementManager.groundLayer))
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

    public void MoveKeepFormation(Vector3 destination, Vector3 anchor)
    {
        print("Orders recieved. Requesting path. " + gameObject.name);
        Vector3 relativePosition = transform.position - anchor;
        PathRequestManager.RequestPath(movementManager.NavPoint, destination + relativePosition, OnPathFound);
    }

    public void MoveConvergeOnPoint(Vector3 destination)
    {
        print("Orders recieved. Requesting path. " + gameObject.name);
        PathRequestManager.RequestPath(movementManager.NavPoint, destination, OnPathFound);
    }

    void OnPathFound(Vector3[] _path, bool success)
    {
        // test manual path
        if (testMode)
            return;

        if (success && _path.Length > 0)
        {
            // Because the pathfinder doesn't immediately return a path (and the delay can get relatively long if many requests are in the queue), the unit may not be in the same position
            // it was in when the request was made, and the unit will be somewhat displaced from the starting node of the path. To avoid a kind of 'backtracking' behavior that can arise from
            // this, once the unit recieves its path, we use the same logic as in the pathfinding script to check to see if the SECOND point in the path is walkable from the unit's current position.
            // If it is, we can skip the first node in the path. If it isn't, then the backtracking behavior is acceptable, and in fact, ideal.    

            // GOTCHA: This approach doesn't work well when traveling up a slope. An alternative approach would be to check if the first point in the path is sufficiently close to
            // the current position of the nav point. If it is, skip it.

            //print(Walkable(NavPoint, _path[1], 0.5f));

            if (_path.Length > 1 && Walkable(movementManager.NavPoint, _path[1], 0.5f))
            {
                _path[0] = movementManager.NavPoint;
            }

            currentPath = new Path(_path);
            currentPath.radius = pathRadius;

            if (pathFollowRoutine != null)
                StopCoroutine(pathFollowRoutine);

            pathFollowRoutine = FollowPath();
            StartCoroutine(pathFollowRoutine);

            CurrentState = AI_STATES.EN_ROUTE;
        }
    }

    IEnumerator FollowPath()
    {
        while (currentPath.waypointIdx < currentPath.waypoints.Length)
        {
            StopCoroutine("MoveToNextWaypoint");
            yield return StartCoroutine("MoveToNextWaypoint");

            print("current waypoint index: " + currentPath.waypointIdx + "| waypoints.Length: " + currentPath.waypoints.Length);
        }

        currentPath = null;
        CurrentState = AI_STATES.IDLE;
    }

    IEnumerator MoveToNextWaypoint()
    {
        while ((currentPath.CurrentWaypoint - movementManager.NavPoint).sqrMagnitude > Mathf.Pow(pathRadius, 2))
        {
            if (currentPath.OnLastLeg)
            {
                movementManager.SeekArrive(currentPath.CurrentWaypoint);
            }
            else
            {
                movementManager.Seek(currentPath.CurrentWaypoint);
            }

            yield return new WaitForFixedUpdate();
        }

        currentPath.waypointIdx++;
    }

    private void OnDrawGizmos()
    {
        if (currentPath != null)
        {
            for (int i = currentPath.waypointIdx; i < currentPath.waypoints.Length; i++)
            {
                Gizmos.color = Color.black;
                Gizmos.DrawCube(currentPath.waypoints[i], Vector3.one * 0.25f);

                if (i == currentPath.waypointIdx)
                {
                    Gizmos.DrawLine(movementManager.NavPoint, currentPath.waypoints[i]);
                }
                else
                {
                    Gizmos.DrawLine(currentPath.waypoints[i - 1], currentPath.waypoints[i]);
                }
            }
        }
    }
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
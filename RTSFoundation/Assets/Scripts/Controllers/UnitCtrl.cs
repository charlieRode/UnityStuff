using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using UnityEngine;

// The newest iteration of the unit controller. We are scrapping the rotation logic entirely,
// going for a pure steering-behavior approach. The rotation of the unit should always be in the direction
// of the velocity vector.


[RequireComponent(typeof(Rigidbody))]
public class UnitCtrl : MonoBehaviour, IBoid
{
    bool initializedVelocity = false;

    Vector3 pNormal;
    Vector3 currentTarget;
    Vector3 vPredict;
    Vector3 prevWaypoint;

    public bool usePathFollowingAlgo;

    // For Path Following
    public float PREDICT_AHEAD;
    public float NORMAL_AHEAD;

    public float collisionRadius;
    public float CollisionRadius
    {
        get
        {
            return collisionRadius;
        }
    }

    public Vector3 Position
    {
        get
        {
            return transform.position;
        }
    }

    public Vector3 Velocity
    {
        get
        {
            if (movementManager != null)
            {
                return movementManager.Velocity;
            }

            return Vector3.zero;
        }
    }

    public Vector3 NavPoint
    {
        get
        {
            if (movementManager != null)
            {
                return movementManager.NavPoint;
            }

            return Vector3.zero;
        }
    }

    public GameObject arrow;

    MovementManager movementManager;
    FSM brain;

    Vector3 destination;

    public Transform navPoint;

    Path currentPath;
    float currentSpeed;

    IEnumerator pathFollowRoutine;

    bool isSelected = false;

    void Start()
    {
        brain = GetComponent<FSM>();
        movementManager = GetComponent<MovementManager>();
        movementManager.Invoke(this);
        GameCtrl.Instance.RegisterActiveUnit(this);
    }

    public void DestroyUnit()
    {
        GameCtrl.Instance.UnregisterActiveUnit(this);
        Destroy(gameObject);
    }

    void RegisterWithOrdersCtrl()
    {
        UnitOrdersCtrl.Instance.moveUnitsConvergeOnPoint += brain.MoveConvergeOnPoint;
        UnitOrdersCtrl.Instance.moveUnitsKeepFormation += brain.MoveKeepFormation;
    }

    void UnregisterWithOrdersCtrl()
    {
        UnitOrdersCtrl.Instance.moveUnitsConvergeOnPoint -= brain.MoveConvergeOnPoint;
        UnitOrdersCtrl.Instance.moveUnitsKeepFormation -= brain.MoveKeepFormation;
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

    /*
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
    */
    /*
    IEnumerator MoveToNextWaypoint()
    {
        while ((currentPath.CurrentWaypoint - navPoint.position).sqrMagnitude > Mathf.Pow(pathRadius, 2))
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
    */

    /*
    
    IEnumerator MoveToNextWaypointN()
    {
        //movementManager.Seek(currentPath.CurrentWaypoint);
        //rb.AddForce(movementManager.Steering, ForceMode.VelocityChange);
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

                movementManager.Seek(currentTarget);
            }
            else if (Velocity.sqrMagnitude < MaxSpeed * MaxSpeed)
            {
                Vector3 vAhead = navPoint.position + transform.forward * MaxSpeed;
                movementManager.Seek(vAhead);
            }

            yield return new WaitForFixedUpdate();
        }

        currentPath.waypointIdx++;
    }
    */
}
    
    /*
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
    }*/

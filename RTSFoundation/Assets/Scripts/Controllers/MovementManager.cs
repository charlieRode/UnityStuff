using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(UnitCtrl))]
[RequireComponent(typeof(Rigidbody))]
public class MovementManager : MonoBehaviour {

    Vector3 correctionPoint;
    public Vector3 Steering
    {
        get
        {
            UpdateManager();
            Vector3 _vSteer = vSteer;
            ResetManager();
            return _vSteer;
        }
    }

    Vector3 vBreak;

    Vector3 vDesired;
    Vector3 vSteer;
    public Vector3 vSteerSeek;
    public Vector3 vSteerFlee;
    Vector3 _vDesired;

    Vector3 vAvoid;

    public Transform[] projectionPoints;
    public Transform centerProjectionPoint;
    public float collisionDetectionAngle;
    bool collisionDetected = false;
    public float wallBuffer;

    public LayerMask collisionAvoidanceLayer;
    public LayerMask groundLayer;

    public LayerMask boidLayer;

    public float SEPARATE_DISTANCE;
    float SeparateDistSqr;

    public float ALIGN_DISTANCE;
    float AlignDistSqr;

    public float COHERE_DISTANCE;
    float CohereDistSqr;

    float NeighborDistSqr;

    public bool linearSeparationForce;

    public bool inverseSquareLaw;
    public bool doCollisionAvoidance;
    public bool useSeparation;
    public float fallingDistance;
    public float verticalOffset;

    public float MAX_SEE_AHEAD;
    public float MAX_AVOIDANCE_FORCE;
    public float MAX_CLIFF_AVOIDANCE_FORCE;
    public float NEIGHBOR_DISTANCE;
    public float SEPARATION_WEIGHT;
    public float ALIGN_WEIGHT;
    public float COHERE_WEIGHT;
    public float BREAK_SCALE_DOWN;
    public float QUEUE_RADIUS;
    public float COLLISION_AVOIDANCE_WEIGHT;
    public float FACE_VELOCITY_THREASHOLD_MAGNITUDE;
    public float FACE_VELOCITY_THREASHOLD_DELTA_ANGLE;
    


    IBoid host;
    Rigidbody rigidBody;
    public Transform navPoint;
    public Vector3 NavPoint
    {
        get
        {
            return navPoint.position;
        }
    }
    public float maxSpeed;
    public float maxForce;
    public float slowingDistance;

    public float moveAngleThreshold;
    public float singleAxisTurnSpeed;

    Vector3 groundNormal;
    Vector3 prevVelocity;

    public float pathRadius;
    Path currentPath;
    IEnumerator pathFollowRoutine;


    public float W_FLOCK_ENROUTE;
    public float W_FLOCK_IDLE;
    [Header("Not for use in inspector")]
    public float _flockWeight;


    public Vector3 Velocity
    {
        get
        {
            if (rigidBody != null)
            {
                // We want to ignore velocity not along the ground plane. Specifically, we are negating
                // the velocity component affected by gravity.
                // Since we are resetting the transform position to the vertical offset each frame,
                // the unit is effectively continuously falling, and this shows up in Velocity.
                return Vector3.ProjectOnPlane(rigidBody.velocity, groundNormal);
            }

            return Vector3.zero;
        }
    }

    bool isGrounded;
    private bool IsGrounded
    {
        get
        {
            return isGrounded;
        }

        set
        {
            isGrounded = value;
            rigidBody.useGravity = !value;
            LockRotation(value);
        }
    }

    public void Invoke(IBoid _host)
    {
        host = _host;
    }

    void Awake()
    {
        rigidBody = GetComponent<Rigidbody>();
    }

    void Start()
    {
        vSteer = Vector3.zero;
        vDesired = Vector3.zero;
        _vDesired = Vector3.zero;
        vBreak = Vector3.zero;
        prevVelocity = Velocity;

        NeighborDistSqr = NEIGHBOR_DISTANCE * NEIGHBOR_DISTANCE;
        SeparateDistSqr = SEPARATE_DISTANCE * SEPARATE_DISTANCE;
        AlignDistSqr = ALIGN_DISTANCE * ALIGN_DISTANCE;
        CohereDistSqr = COHERE_DISTANCE * COHERE_DISTANCE;
    }

    public void Move()
    {
        vSteer += DoMove();
    }

    public void Seek(Vector3 target)
    {
        vSteerSeek = DoSeek(target);
        //vSteer += vSteerSeek;
        vDesired += vSteerSeek;
    }
    public void SeekArrive(Vector3 target)
    {
        vSteerSeek = DoSeekArrive(target);
        //vSteer += DoSeekArrive(target);
        vDesired += vSteerSeek;
    }
    public void Flee(Vector3 target)
    {
        vSteerFlee = DoFlee(target);
        //vSteer += vSteerFlee;
        vDesired += vSteerFlee;
    }
    public void Wander() { }
    public void Evade(IBoid target)
    {
        vSteer += DoEvade(target);
    }
    public void Pursue(IBoid target)
    {
        vSteer += DoPursue(target);
    }

    public void CollisionAvoidance()
    {
        //vSteer += DoCollisionAvoidance();
        vDesired += (DoCollisionAvoidance() + DoCliffAvoidance()) * COLLISION_AVOIDANCE_WEIGHT;
    }

    public void Separate()
    {
        vDesired += DoSeparate() * SEPARATION_WEIGHT;
    }

    public void Flock()
    {
        vDesired += DoFlock() * _flockWeight;
    }

    void UpdateManager()
    {
        if (vDesired.sqrMagnitude > maxSpeed * maxSpeed)
        {
            vDesired = vDesired.normalized * maxSpeed;
        }
        vSteer = vDesired - Velocity;
        if (vSteer.sqrMagnitude > maxForce * maxForce)
        {
            vSteer = vSteer.normalized * maxForce;
        }

        vBreak += DoQueue();

        vSteer += vBreak;
    }
    public void ResetManager()
    {
        vSteer = Vector3.zero;
        vDesired = Vector3.zero;
        vBreak = Vector3.zero;
    }

    Vector3 DoMove()
    {
        _vDesired = transform.forward * maxSpeed;
        return _vDesired;
    }

    Vector3 DoSeek(Vector3 target)
    {
        // For each steering force, we need to take the planar projection of the vector that points from
        // the unit to the target, onto the ground normal. This is necessary because the unit cannot
        // move except in directions along the ground. Else the unit would be applying an upward force to reach an elevated target... But our unit cannot fly.
        Vector3 planarProj = Vector3.ProjectOnPlane((target - NavPoint), groundNormal);
        _vDesired = planarProj.normalized * maxSpeed;

        return _vDesired;
    }

    Vector3 DoSeekArrive(Vector3 target)
    {
        Vector3 planarProj = Vector3.ProjectOnPlane((target - NavPoint), groundNormal);
        Vector3 vDesiredMax = planarProj.normalized * maxSpeed;
        if (planarProj.sqrMagnitude < slowingDistance * slowingDistance)
        {
            float interpolationVal = (planarProj.magnitude - pathRadius) / (slowingDistance - pathRadius);
            _vDesired = Vector3.Lerp(Vector3.zero, vDesiredMax, interpolationVal);
        }
        else
        {
            _vDesired = vDesiredMax;
        }

        return _vDesired;
    }

    Vector3 DoFlee(Vector3 target)
    {
        Vector3 planarProj = Vector3.ProjectOnPlane((NavPoint - target), groundNormal);
        _vDesired = planarProj.normalized * maxSpeed;
        return _vDesired;
    }

    Vector3 DoWander()
    {
        return Vector3.zero;
    }

    Vector3 DoEvade(IBoid target)
    {
        float distanceToTarget = (NavPoint - target.NavPoint).magnitude;
        int t = Mathf.RoundToInt(distanceToTarget / maxSpeed);
        Vector3 futurePosition = target.NavPoint + target.Velocity * t;

        return DoFlee(futurePosition);
    }

    Vector3 DoPursue(IBoid target)
    {
        float distanceToTarget = (NavPoint - target.NavPoint).magnitude;
        int t = Mathf.RoundToInt(distanceToTarget / maxSpeed);
        Vector3 futurePosition = target.NavPoint + target.Velocity * t;

        return DoSeek(futurePosition);
    }

    // -1 for left, 0 for center, 1 for right
    int ToMyLeftOrRight(Vector3 point)
    {
        return AngleDir(transform.forward, point - transform.position, transform.up);
    }

    int AngleDir(Vector3 fwd, Vector3 targetDir, Vector3 up)
    {
        Vector3 perp = Vector3.Cross(fwd, targetDir);
        float dir = Vector3.Dot(perp, up);

        if (dir > 0)
        {
            return 1;
        }
        else if (dir < 0)
        {
            return -1;
        }
        else
        {
            return 0;
        }
    }

    Vector3 DoCollisionAvoidance()
    {
        RaycastHit hitData;
        if (GetClosestObsticle(out hitData))
        {
            // Secondary Check: What type of edge did we detect?
            int leftRight = ToMyLeftOrRight(hitData.point);
            // left
            if (leftRight == -1)
            {
                // straight wall -> turn away from the wall
                if (Physics.Raycast(transform.position, -transform.right, 2.5f, groundLayer))
                {
                    collisionDetected = true;
                    correctionPoint = hitData.point + hitData.normal * Velocity.magnitude;
                    return hitData.distance > wallBuffer ? DoSeek(correctionPoint) : Vector3.ProjectOnPlane(hitData.normal, groundNormal).normalized * maxSpeed;
                }
                // corner -> turn away from corner
                else
                {
                    Vector3 proj = centerProjectionPoint.position + transform.right * transform.localScale.x / 2f;
                    Vector3 projDir = Quaternion.AngleAxis(collisionDetectionAngle, Vector3.up) * transform.forward;
                    Vector3 target = proj + projDir * MAX_SEE_AHEAD;

                    return DoSeek(target);
                }
            }
            // right
            else if (leftRight == 1)
            {
                if (Physics.Raycast(transform.position, transform.right, 2.5f, groundLayer))
                {
                    collisionDetected = true;
                    correctionPoint = hitData.point + hitData.normal * Velocity.magnitude;
                    return hitData.distance > wallBuffer ? DoSeek(correctionPoint) : Vector3.ProjectOnPlane(hitData.normal, groundNormal).normalized * maxSpeed;
                }
                else
                {
                    Vector3 proj = centerProjectionPoint.position - transform.right * transform.localScale.x / 2f;
                    Vector3 projDir = Quaternion.AngleAxis(collisionDetectionAngle, Vector3.up) * transform.forward;
                    Vector3 target = proj + projDir * MAX_SEE_AHEAD;

                    return DoSeek(target);
                }
            }

            collisionDetected = true;
            correctionPoint = hitData.point + hitData.normal * Velocity.magnitude;
            return hitData.distance > wallBuffer ? DoSeek(correctionPoint) : Vector3.ProjectOnPlane(hitData.normal, groundNormal).normalized * maxSpeed;
        }

        collisionDetected = false;
        return Vector3.zero;
    }

    // Not Currently Working
    Vector3 DoCliffAvoidance()
    {
        /*
        Vector3 checkPoint = transform.position;
        if (!Physics.Raycast(checkPoint, Vector3.down, fallingDistance, groundLayer))
        {
            collisionDetected = true;

            vAvoid = Vector3.Lerp(Vector3.zero, -Velocity.normalized * MAX_CLIFF_AVOIDANCE_FORCE, Velocity.magnitude / maxSpeed);
            return vAvoid;
        }

        collisionDetected = false;
        */
        return Vector3.zero;
    }

    Vector3 DoFlock()
    {
        Vector3 sumSeparationForce = Vector3.zero;
        Vector3 sumNeighborPosition = Vector3.zero;
        Vector3 sumNeighborVelocity = Vector3.zero;
        int neighborsSeparateCount = 0;
        int neighborsAlignCount = 0;
        int neighborsCohereCount = 0;
        foreach (IBoid neighbor in GameCtrl.Instance.activeUnitsInGame)
        {
            if (neighbor == host)
                continue;

            Vector3 neighborToHost = NavPoint - neighbor.NavPoint;

            if (neighborToHost.sqrMagnitude <= SeparateDistSqr)
            {
                neighborsSeparateCount++;
                if (linearSeparationForce)
                {
                    if (inverseSquareLaw)
                    {
                        Vector3 vDesiredN = neighborToHost.normalized * maxSpeed / neighborToHost.magnitude;
                        sumSeparationForce += vDesiredN;
                    }
                    else
                    {
                        Vector3 vDesiredN = neighborToHost.normalized * Mathf.Lerp(maxSpeed, 0, neighborToHost.magnitude / NEIGHBOR_DISTANCE);
                        sumSeparationForce += vDesiredN;
                    }
                }
                else
                {
                    if (inverseSquareLaw)
                    {
                        Vector3 vDesiredN = neighborToHost.normalized * maxSpeed / neighborToHost.sqrMagnitude;
                        sumSeparationForce += vDesiredN;
                    }
                    else
                    {
                        Vector3 vDesiredN = neighborToHost.normalized * Mathf.Lerp(maxSpeed, 0, neighborToHost.sqrMagnitude / NeighborDistSqr);
                        sumSeparationForce += vDesiredN;
                    }
                }
            }

            if (neighborToHost.sqrMagnitude <= AlignDistSqr)
            {
                neighborsAlignCount++;
                sumNeighborVelocity += neighbor.Velocity;
            }

            if (neighborToHost.sqrMagnitude > SeparateDistSqr && neighborToHost.sqrMagnitude <= CohereDistSqr)
            {
                neighborsCohereCount++;
                sumNeighborPosition += neighbor.NavPoint;
            }
        }

        Vector3 separationForce = neighborsSeparateCount > 0 ? sumSeparationForce / neighborsSeparateCount : Vector3.zero;
        Vector3 alignmentForce = neighborsAlignCount > 0 ? sumNeighborVelocity / neighborsAlignCount : Vector3.zero;

        Vector3 cohesionForce = Vector3.zero;
        if (neighborsCohereCount > 0)
        {
            Vector3 avgNeighborPosition = sumNeighborPosition / neighborsCohereCount;
            cohesionForce = (avgNeighborPosition - transform.position).normalized * maxSpeed;
        }

        if (separationForce.sqrMagnitude > maxSpeed * maxSpeed)
            separationForce = separationForce.normalized * maxSpeed;

        if (alignmentForce.sqrMagnitude > maxSpeed * maxSpeed)
            alignmentForce = alignmentForce.normalized * maxSpeed;

        if (cohesionForce.sqrMagnitude > maxSpeed * maxSpeed)
            cohesionForce = cohesionForce.normalized * maxSpeed;

        return separationForce * SEPARATION_WEIGHT + alignmentForce * ALIGN_WEIGHT + cohesionForce * COHERE_WEIGHT;
    }

    // DoQueue() MUST be run AFTER vSteer has been calculated. It must be run in the UpdateManager function for now... (EVEN THOUGH THAT'S NOT WHERE IT GOES, I KNOW!)
    Vector3 DoQueue()
    {
        IBoid neighborAhead;
        if (GetNeighborAhead(out neighborAhead))
        {
            Vector3 breakForce = -Velocity * BREAK_SCALE_DOWN;
            breakForce += -vSteer * BREAK_SCALE_DOWN;

            if (Vector3.Distance(NavPoint, neighborAhead.NavPoint) < QUEUE_RADIUS)
            {
                rigidBody.velocity *= 0.3f;
            }
            return breakForce;
        }

        return Vector3.zero;
    }

    bool GetNeighborAhead(out IBoid neighbor)
    {
        neighbor = null;
        Vector3 qAhead = Velocity.normalized * QUEUE_RADIUS;
        Vector3 checkPoint = centerProjectionPoint.position + qAhead;
        foreach (IBoid agent in GameCtrl.Instance.activeUnitsInGame)
        {
            if (agent != host && Vector3.Distance(checkPoint, agent.Position) <= QUEUE_RADIUS)
            {
                neighbor = agent;
                return true;
            }
        }

        return false;
    }

    Vector3 DoSeparate()
    {
        Vector3 sumSeparationForce = Vector3.zero;
        int neighborCount = 0;
        foreach (IBoid neighbor in GameCtrl.Instance.activeUnitsInGame)
        {
            if (neighbor == host)
                continue;

            Vector3 neighborToHost = NavPoint - neighbor.NavPoint;
            if (neighborToHost.sqrMagnitude <= NeighborDistSqr)
            {

                neighborCount++;
                if (linearSeparationForce)
                {
                    Vector3 vDesiredN = neighborToHost.normalized * Mathf.Lerp(maxSpeed, 0, neighborToHost.magnitude / NEIGHBOR_DISTANCE);
                    sumSeparationForce += vDesiredN;
                }
                else
                {
                    Vector3 vDesiredN = neighborToHost.normalized * Mathf.Lerp(maxSpeed, 0, neighborToHost.sqrMagnitude / NeighborDistSqr);
                    sumSeparationForce += vDesiredN;
                }
            }
        }

        if (neighborCount > 0)
        {
            return sumSeparationForce / neighborCount;
        }
        else
        {
            return Vector3.zero;
        }
    }

    bool GetClosestObsticle(out RaycastHit hitData)
    {
        bool collisionFound = false;
        float closestDistance = -1f;
        hitData = default(RaycastHit);
        RaycastHit hit;

        // will need to do this differently to accomodate projection numbers other than 2.
        int numProjections = 2;
        for (int i = 0; i < numProjections; i++)
        {
            // Vector3 projPoint = projectionPoints[i].position;
            Vector3 projPoint = transform.position;
            Vector3 projDir = i % 2 == 0 ? Quaternion.AngleAxis(collisionDetectionAngle, Vector3.up) * transform.forward : Quaternion.AngleAxis(-collisionDetectionAngle, Vector3.up) * transform.forward;
            if (Physics.Raycast(projPoint, projDir, out hit, MAX_SEE_AHEAD, collisionAvoidanceLayer))
            {
                float angleOfSlope = Vector2.Angle(transform.forward, Vector3.Cross(hit.normal, -transform.right));
                if (angleOfSlope > Grid.Instance.maxInclineAngle)
                {
                    if (!collisionFound || hit.distance < closestDistance)
                    {
                        collisionFound = true;
                        closestDistance = hit.distance;
                        hitData = hit;
                    }
                }
            }
        }

        return collisionFound;
    }

    /*
    bool GetClosestObsticle_Old(out Vector3 collisionPointNormal)
    {
        Vector3 leftSide = transform.position + new Vector3(-transform.forward.z, transform.forward.y, transform.forward.x) * 0.5f;
        Vector3 rightSide = transform.position + new Vector3(transform.forward.z, transform.forward.y, -transform.forward.x) * 0.5f;

        RaycastHit lHit, rHit;
        float angleOfSlope;
        if (Physics.Raycast(leftSide, transform.forward, out lHit, MAX_SEE_AHEAD, collisionAvoidanceLayer) && Physics.Raycast(rightSide, transform.forward, out rHit, MAX_SEE_AHEAD, collisionAvoidanceLayer))
        {
            if (lHit.distance <= rHit.distance)
            {
                angleOfSlope = Vector3.Angle(transform.forward, Vector3.Cross(lHit.normal, -transform.right));
                if (angleOfSlope > Grid.Instance.maxInclineAngle)
                {
                    collisionPointNormal = lHit.normal;
                    return true;
                }
                else
                {
                    angleOfSlope = Vector3.Angle(transform.forward, Vector3.Cross(rHit.normal, -transform.right));
                    if (angleOfSlope > Grid.Instance.maxInclineAngle)
                    {
                        collisionPointNormal = rHit.normal;
                        return true;
                    }
                }
            }
            else
            {
                angleOfSlope = Vector3.Angle(transform.forward, Vector3.Cross(rHit.normal, -transform.right));
                if (angleOfSlope > Grid.Instance.maxInclineAngle)
                {
                    collisionPointNormal = rHit.normal;
                    return true;
                }
                else
                {
                    angleOfSlope = Vector3.Angle(transform.forward, Vector3.Cross(lHit.normal, -transform.right));
                    if (angleOfSlope > Grid.Instance.maxInclineAngle)
                    {
                        collisionPointNormal = lHit.normal;
                        return true;
                    }
                }
            }
        }

        else if (Physics.Raycast(leftSide, transform.forward, out lHit, MAX_SEE_AHEAD, collisionAvoidanceLayer))
        {
            angleOfSlope = Vector3.Angle(transform.forward, Vector3.Cross(lHit.normal, -transform.right));
            if (angleOfSlope > Grid.Instance.maxInclineAngle)
            {
                collisionPointNormal = lHit.normal;
                return true;
            }
        }
        else if (Physics.Raycast(rightSide, transform.forward, out rHit, MAX_SEE_AHEAD, collisionAvoidanceLayer))
        {
            angleOfSlope = Vector3.Angle(transform.forward, Vector3.Cross(rHit.normal, -transform.right));
            if (angleOfSlope > Grid.Instance.maxInclineAngle)
            {
                collisionPointNormal = rHit.normal;
                return true;
            }
        }

        collisionPointNormal = Vector3.zero;
        return false;
    }
    */

    void FixedUpdate()
    {
        if (doCollisionAvoidance)
        {
            CollisionAvoidance();
        }

        Ray ray = new Ray(rigidBody.position, Vector3.down);
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
                rigidBody.velocity = Vector3.zero;
            }

            // Only allow steering forces to be applied if Grounded.
            // While Grounded, if the unit height deviates from the vertical offset, reset it.
            if (IsGrounded)
            {
                if (rHit.distance != verticalOffset)
                {
                    rigidBody.MovePosition(new Vector3(rigidBody.position.x, rHit.point.y + verticalOffset, rigidBody.position.z));
                    rigidBody.velocity = new Vector3(rigidBody.velocity.x, 0, rigidBody.velocity.z);
                }

                if (useSeparation)
                    Separate();

                Flock();

                rigidBody.AddForce(Steering, ForceMode.VelocityChange);

            }
        }
        else
        {
            UnityEngine.Debug.Log("ERR: Raycast did not intersect with Ground Layer");
        }

        // Face in the direction of movement. 
        /*
        if (rigidBody.velocity.sqrMagnitude > FACE_VELOCITY_THREASHOLD_MAGNITUDE * FACE_VELOCITY_THREASHOLD_MAGNITUDE && Vector3.Angle(rigidBody.velocity, prevVelocity) > FACE_VELOCITY_THREASHOLD_DELTA_ANGLE)
        {
            prevVelocity = Velocity;
            Vector3 lookDirection = new Vector3(rigidBody.velocity.x, 0, rigidBody.velocity.z);
            rigidBody.rotation = Quaternion.LookRotation(lookDirection);
        }
        */


        Vector3 _dir = Velocity.normalized;
        float _theta = Mathf.Atan2(_dir.x, _dir.z) * Mathf.Rad2Deg;

        if (Mathf.Abs(Mathf.DeltaAngle(transform.eulerAngles.y, _theta)) > moveAngleThreshold)
        {
            float rotationStep = Mathf.MoveTowardsAngle(transform.eulerAngles.y, _theta, singleAxisTurnSpeed * Time.fixedDeltaTime);
            rigidBody.MoveRotation(Quaternion.Euler(Vector3.up * rotationStep));
        }

    }

    void LockRotation(bool val)
    {
        rigidBody.freezeRotation = val;
    }

    void OnDrawGizmos()
    {
        if (doCollisionAvoidance)
        {
            Gizmos.color = Color.red;
            if (collisionDetected)
            {
                Gizmos.DrawLine(transform.position, transform.position + vAvoid);
            }

            Gizmos.color = collisionDetected ? Color.red : Color.green;

            int numProjections = 2;
            for (int i = 0; i < numProjections; i++)
            {
                Gizmos.color = collisionDetected ? Color.red : Color.green;
                //Vector3 projPoint = projectionPoints[i].position;
                Vector3 projPoint = transform.position;
                Vector3 projDir = i % 2 == 0 ? Quaternion.AngleAxis(collisionDetectionAngle, Vector3.up) * transform.forward : Quaternion.AngleAxis(-collisionDetectionAngle, Vector3.up) * transform.forward;
                Gizmos.DrawLine(projPoint, projPoint + projDir * MAX_SEE_AHEAD);
            }

        }
    }
}

public interface IBoid
{
    Vector3 NavPoint { get; }
    Vector3 Velocity { get; }
    Vector3 Position { get; }
}
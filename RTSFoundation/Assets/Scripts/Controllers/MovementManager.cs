using System.Collections;
using System.Collections.Generic;
using UnityEngine;

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
    IBoid host;

    Vector3 leftSide;
    Vector3 rightSide;
    public Transform[] projectionPoints;
    public Transform centerProjectionPoint;
    public float collisionDetectionAngle;
    bool collisionDetected = false;
    public float wallBuffer;

    public LayerMask collisionAvoidanceLayer;
    public LayerMask groundLayer;

    public LayerMask boidLayer;

    public float MAX_SEE_AHEAD;
    public float MAX_AVOIDANCE_FORCE;
    public float NEIGHBOR_DISTANCE;
    public float SEPARATION_WEIGHT;
    public float ALIGN_WEIGHT;
    public float COHERE_WEIGHT;

    public float BREAK_SCALE_DOWN;

    public float QUEUE_RADIUS;

    public float COLLISION_AVOIDANCE_WEIGHT;

    [Header("Master Weight")]
    public float FLOCK_WEIGHT;

    public float SEPARATE_DISTANCE;
    float SeparateDistSqr;

    public float ALIGN_DISTANCE;
    float AlignDistSqr;

    public float COHERE_DISTANCE;
    float CohereDistSqr;

    float NeighborDistSqr;

    public bool linearSeparationForce;

    public bool inverseSquareLaw;

    

    public void Invoke(IBoid _host)
    {
        host = _host;
        vSteer = Vector3.zero;
        vDesired = Vector3.zero;
        _vDesired = Vector3.zero;
        vBreak = Vector3.zero;

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
        vDesired += DoCollisionAvoidance() * COLLISION_AVOIDANCE_WEIGHT;

    }

    public void Separate()
    {
        vDesired += DoSeparate() * SEPARATION_WEIGHT;
    }

    public void Flock()
    {
        vDesired += DoFlock() * FLOCK_WEIGHT;
    }

    void UpdateManager()
    {
        if (vDesired.sqrMagnitude > host.MaxSpeed * host.MaxSpeed)
        {
            vDesired = vDesired.normalized * host.MaxSpeed;
        }
        vSteer = vDesired - host.Velocity;
        if (vSteer.sqrMagnitude > host.MaxForce * host.MaxForce)
        {
            vSteer = vSteer.normalized * host.MaxForce;
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
        _vDesired = transform.forward * host.MaxSpeed;
        return _vDesired;
    }

    Vector3 DoSeek(Vector3 target)
    {
        // For each steering force, we need to take the planar projection of the vector that points from
        // the unit to the target, onto the ground normal. This is necessary because the unit cannot
        // move except in directions along the ground. Else the unit would be applying an upward force to reach an elevated target... But our unit cannot fly.
        Vector3 planarProj = Vector3.ProjectOnPlane((target - host.NavPoint.position), host.GroundNormal);
        _vDesired = planarProj.normalized * host.MaxSpeed;

        return _vDesired;
    }

    Vector3 DoSeekArrive(Vector3 target)
    {
        Vector3 planarProj = Vector3.ProjectOnPlane((target - host.NavPoint.position), host.GroundNormal);
        Vector3 vDesiredMax = planarProj.normalized * host.MaxSpeed;
        if (planarProj.sqrMagnitude < host.SlowingDistance * host.SlowingDistance)
        {
            float interpolationVal = (planarProj.magnitude - host.StoppingDistance) / (host.SlowingDistance - host.StoppingDistance);
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
        Vector3 planarProj = Vector3.ProjectOnPlane((host.NavPoint.position - target), host.GroundNormal);
        _vDesired = planarProj.normalized * host.MaxSpeed;
        return _vDesired;
    }

    Vector3 DoWander()
    {
        return Vector3.zero;
    }

    Vector3 DoEvade(IBoid target)
    {
        float distanceToTarget = (host.NavPoint.position - target.NavPoint.position).magnitude;
        int t = Mathf.RoundToInt(distanceToTarget / host.MaxSpeed);
        Vector3 futurePosition = target.NavPoint.position + target.Velocity * t;

        return DoFlee(futurePosition);
    }

    Vector3 DoPursue(IBoid target)
    {
        float distanceToTarget = (host.NavPoint.position - target.NavPoint.position).magnitude;
        int t = Mathf.RoundToInt(distanceToTarget / host.MaxSpeed);
        Vector3 futurePosition = target.NavPoint.position + target.Velocity * t;

        return DoSeek(futurePosition);
    }

    Vector3 DoCollisionAvoidance()
    {
        RaycastHit hitData;
        if (GetClosestObsticle(out hitData))
        {
            collisionDetected = true;
            correctionPoint = hitData.point + hitData.normal * host.Velocity.magnitude;
            return hitData.distance > wallBuffer ? DoSeek(correctionPoint) : Vector3.ProjectOnPlane(hitData.normal, host.GroundNormal).normalized * host.MaxSpeed;
        }

        collisionDetected = false;
        return Vector3.zero;
    }

    // Not Currently Working
    Vector3 DoCliffAvoidance()
    {
        Vector3 proj1 = centerProjectionPoint.position + transform.right * host.TransformPoint.localScale.x / 2f;
        Vector3 proj2 = centerProjectionPoint.position - transform.right * host.TransformPoint.localScale.x / 2f;
        Vector3 projDir1 = Quaternion.AngleAxis(collisionDetectionAngle, Vector3.up) * transform.forward;
        Vector3 projDir2 = Quaternion.AngleAxis(-collisionDetectionAngle, Vector3.up) * transform.forward;
        Vector3 checkCliffPoint1 = proj1 + projDir1 * MAX_SEE_AHEAD;
        Vector3 checkCliffPoint2 = proj2 + projDir2 * MAX_SEE_AHEAD;

        if (!Physics.Raycast(checkCliffPoint1, Vector3.down, host.FallDistance, groundLayer))
        {
            collisionDetected = true;
            return DoSeek(checkCliffPoint2);
        }
        if (!Physics.Raycast(checkCliffPoint2, Vector3.down, host.FallDistance, groundLayer))
        {
            collisionDetected = true;
            return DoSeek(checkCliffPoint1);
        }

        collisionDetected = false;
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

            Vector3 neighborToHost = host.NavPoint.position - neighbor.NavPoint.position;

            if (neighborToHost.sqrMagnitude <= SeparateDistSqr)
            {
                neighborsSeparateCount++;
                if (linearSeparationForce)
                {
                    if (inverseSquareLaw)
                    {
                        Vector3 vDesiredN = neighborToHost.normalized * host.MaxSpeed / neighborToHost.magnitude;
                        sumSeparationForce += vDesiredN;
                    }
                    else
                    {
                        Vector3 vDesiredN = neighborToHost.normalized * Mathf.Lerp(host.MaxSpeed, 0, neighborToHost.magnitude / NEIGHBOR_DISTANCE);
                        sumSeparationForce += vDesiredN;
                    }
                }
                else
                {
                    if (inverseSquareLaw)
                    {
                        Vector3 vDesiredN = neighborToHost.normalized * host.MaxSpeed / neighborToHost.sqrMagnitude;
                        sumSeparationForce += vDesiredN;
                    }
                    else
                    {
                        Vector3 vDesiredN = neighborToHost.normalized * Mathf.Lerp(host.MaxSpeed, 0, neighborToHost.sqrMagnitude / NeighborDistSqr);
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
                sumNeighborPosition += neighbor.NavPoint.position;
            }
        }

        Vector3 separationForce = neighborsSeparateCount > 0 ? sumSeparationForce / neighborsSeparateCount : Vector3.zero;
        Vector3 alignmentForce = neighborsAlignCount > 0 ? sumNeighborVelocity / neighborsAlignCount : Vector3.zero;

        Vector3 cohesionForce = Vector3.zero;
        if (neighborsCohereCount > 0)
        {
            Vector3 avgNeighborPosition = sumNeighborPosition / neighborsCohereCount;
            cohesionForce = (avgNeighborPosition - transform.position).normalized * host.MaxSpeed;
        }

        if (separationForce.sqrMagnitude > host.MaxSpeed * host.MaxSpeed)
            separationForce = separationForce.normalized * host.MaxSpeed;

        if (alignmentForce.sqrMagnitude > host.MaxSpeed * host.MaxSpeed)
            alignmentForce = alignmentForce.normalized * host.MaxSpeed;

        if (cohesionForce.sqrMagnitude > host.MaxSpeed * host.MaxSpeed)
            cohesionForce = cohesionForce.normalized * host.MaxSpeed;

        return separationForce * SEPARATION_WEIGHT + alignmentForce * ALIGN_WEIGHT + cohesionForce * COHERE_WEIGHT;
    }

    // DoQueue() MUST be run AFTER vSteer has been calculated. It must be run in the UpdateManager function for now... (EVEN THOUGH THAT'S NOT WHERE IT GOES, I KNOW!)
    Vector3 DoQueue()
    {
        IBoid neighborAhead;
        if (GetNeighborAhead(out neighborAhead))
        {
            Vector3 breakForce = -host.Velocity * BREAK_SCALE_DOWN;
            breakForce += -vSteer * BREAK_SCALE_DOWN;

            if (Vector3.Distance(host.NavPoint.position, neighborAhead.NavPoint.position) < QUEUE_RADIUS)
            {
                host.ScaleVelocity(0.3f);
            }
            return breakForce;
        }

        return Vector3.zero;
    }

    bool GetNeighborAhead(out IBoid neighbor)
    {
        neighbor = null;
        Vector3 qAhead = host.Velocity.normalized * QUEUE_RADIUS;
        Vector3 checkPoint = centerProjectionPoint.position + qAhead;
        foreach (IBoid agent in GameCtrl.Instance.activeUnitsInGame)
        {
            if (agent != host && Vector3.Distance(checkPoint, agent.TransformPoint.position) <= QUEUE_RADIUS)
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

            Vector3 neighborToHost = host.NavPoint.position - neighbor.NavPoint.position;
            if (neighborToHost.sqrMagnitude <= NeighborDistSqr)
            {

                neighborCount++;
                if (linearSeparationForce)
                {
                    Vector3 vDesiredN = neighborToHost.normalized * Mathf.Lerp(host.MaxSpeed, 0, neighborToHost.magnitude / NEIGHBOR_DISTANCE);
                    sumSeparationForce += vDesiredN;
                }
                else
                {
                    Vector3 vDesiredN = neighborToHost.normalized * Mathf.Lerp(host.MaxSpeed, 0, neighborToHost.sqrMagnitude / NeighborDistSqr);
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

        for (int i = 0; i < projectionPoints.Length; i++)
        {
            Vector3 projPoint = projectionPoints[i].position;
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

    bool GetClosestObsticle_Old(out Vector3 collisionPointNormal)
    {
        leftSide = transform.position + new Vector3(-transform.forward.z, transform.forward.y, transform.forward.x) * 0.5f;
        rightSide = transform.position + new Vector3(transform.forward.z, transform.forward.y, -transform.forward.x) * 0.5f;

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

    /*
    // THIS DOESN'T WORK
    Vector3 DoCollisionAvoidanceTuts()
    {
        RaycastHit hit;
        if (Physics.Raycast(transform.position, transform.forward, out hit, MAX_SEE_AHEAD, collisionAvoidanceLayer))
        {
            IObsticle obsticle = hit.collider.GetComponent<IObsticle>();
            float dynamicLength = host.Velocity.magnitude / host.MaxSpeed;
            Vector3 ahead = transform.position + host.Velocity.normalized * dynamicLength * MAX_SEE_AHEAD;

            Vector3 vAvoidance = (ahead - obsticle.Center).normalized * MAX_AVOIDANCE_FORCE;
            return vAvoidance - host.Velocity;
        }
        return Vector3.zero;
    }

    // THIS DOESN'T WORK
    Vector3 DoCollisionAvoidance()
    {
        IObsticle target;
        Vector3 collisionPoint;
        if (GetMostThreatening(out target, out collisionPoint))
        {
            Vector3 vAvoid = (collisionPoint - target.Center).normalized;
            float avoidForce = Mathf.Lerp(MAX_AVOIDANCE_FORCE, 0, (target.Center - transform.position).magnitude / MAX_SEE_AHEAD);
            return vAvoid * avoidForce - host.Velocity;
        }
        return Vector3.zero;
    }

    bool GetMostThreatening(out IObsticle target, out Vector3 collisionPoint)
    {
        target = default(IObsticle);
        leftSide = transform.position + new Vector3(-transform.forward.z, transform.forward.y, transform.forward.x) * 0.5f;
        rightSide = transform.position + new Vector3(transform.forward.z, transform.forward.y, -transform.forward.x) * 0.5f;

        RaycastHit lHit, rHit;
        if (Physics.Raycast(leftSide, transform.forward, out lHit, MAX_SEE_AHEAD, collisionAvoidanceLayer) && Physics.Raycast(rightSide, transform.forward, out rHit, MAX_SEE_AHEAD, collisionAvoidanceLayer))
        {
            if (lHit.distance <= rHit.distance)
            {
                target = lHit.collider.GetComponent<IObsticle>();
                collisionPoint = lHit.point;
            }
            else
            {
                target = rHit.collider.GetComponent<IObsticle>();
                collisionPoint = rHit.point;
            }
            return true;
        }
        else if (Physics.Raycast(leftSide, transform.forward, out lHit, MAX_SEE_AHEAD, collisionAvoidanceLayer))
        {
            target = lHit.collider.GetComponent<UnitCtrl>() as IObsticle;
            collisionPoint = lHit.point;
            return true;
        }
        else if (Physics.Raycast(rightSide, transform.forward, out rHit, MAX_SEE_AHEAD, collisionAvoidanceLayer))
        {
            target = rHit.collider.GetComponent<UnitCtrl>() as IObsticle;
            collisionPoint = rHit.point;
            return true;
        }

        collisionPoint = Vector3.zero;
        return false;
    }
    */


    void OnDrawGizmos()
    {
        if (host == null)
            return;
        if (host.doCA)
        {
            /*
            Gizmos.color = collisionDetected ? Color.red : Color.green;
            Gizmos.DrawLine(leftSide, leftSide + transform.forward * MAX_SEE_AHEAD);
            Gizmos.DrawLine(rightSide, rightSide + transform.forward * MAX_SEE_AHEAD);
            */

            Gizmos.color = collisionDetected ? Color.red : Color.green;
            for (int i = 0; i < projectionPoints.Length; i++)
            {
                Gizmos.color = collisionDetected ? Color.red : Color.green;
                Vector3 projPoint = projectionPoints[i].position;
                Vector3 projDir = i % 2 == 0 ? Quaternion.AngleAxis(collisionDetectionAngle, Vector3.up) * transform.forward : Quaternion.AngleAxis(-collisionDetectionAngle, Vector3.up) * transform.forward;
                Gizmos.DrawLine(projPoint, projPoint + projDir * MAX_SEE_AHEAD);
                Gizmos.color = Color.blue;
                Gizmos.DrawSphere(projPoint + projDir * MAX_SEE_AHEAD, .25f);
            }

            Gizmos.color = Color.blue;
            Gizmos.DrawSphere(correctionPoint, 0.25f);

            Vector3 proj1 = centerProjectionPoint.position + transform.right * host.TransformPoint.localScale.x / 2f;
            Vector3 proj2 = centerProjectionPoint.position - transform.right * host.TransformPoint.localScale.x / 2f;
            Gizmos.color = Color.yellow;

            Vector3 projDir1 = Quaternion.AngleAxis(collisionDetectionAngle, Vector3.up) * transform.forward;
            Vector3 projDir2 = Quaternion.AngleAxis(-collisionDetectionAngle, Vector3.up) * transform.forward;
            Gizmos.DrawCube(proj1 + projDir1 * MAX_SEE_AHEAD, Vector3.one / 4f);
            Gizmos.DrawCube(proj2 + projDir2 * MAX_SEE_AHEAD, Vector3.one / 4f);

        }
    }
}

public interface IBoid
{
    Vector3 Velocity { get; } // rb.velocity
    float MaxSpeed { get; } // movementSpeed
    float MaxForce { get; } // maxForce
    float SlowingDistance { get; } // arrive radius
    float StoppingDistance { get; } // closeEnough
    Transform TransformPoint { get; } // transform
    Transform NavPoint { get; } // navPoint
    Vector3 GroundNormal { get; } // groundNormal

    bool doCA { get; }
    float FallDistance { get; }
    void ScaleVelocity(float scalar);
}

public interface IObsticle
{
    Vector3 Center { get; }
    float CollisionRadius { get; }
    string Name { get; }
}
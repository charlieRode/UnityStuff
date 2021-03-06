﻿using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MovementManager : MonoBehaviour {

    List<IBoid> neighbors = new List<IBoid>();

    public Vector3 Steering
    {
        get
        {
            UpdateManager();
            return vSteer;
        }
    }

    public bool printForces;

    Vector3 _vDesired;

    Vector3 vDesired;
    Vector3 vDesiredSeek;
    Vector3 vDesiredSeparate;
    Vector3 vSteer;
    Vector3 vSteerSeek;
    Vector3 vSteerSeparate;
    Vector3 N2H;
    IBoid host;

    Vector3 leftSide;
    Vector3 rightSide;

    public float flockNeighborCount;
    public float separateNeighborCount;

    public LayerMask collisionAvoidanceLayer;

    public float MAX_SEE_AHEAD;
    public float MAX_AVOIDANCE_FORCE;

    // I'd think align distance == cohere distance, and separation distance is somewhat smaller than those...

    public float NEIGHBOR_DISTANCE;
    float NeighborDistSqr;
    public float COHERE_DISTANCE;
    float CohereDistSqr;
    public float SEPARATE_DISTANCE;
    float SeparateDistSqr;
    public float ALIGN_DISTANCE;
    float AlignDistSqr;

    public bool linearSeparationForce;
    public bool inverseSqareLaw;
    public float SEPARATION_WEIGHT;
    public float ALIGN_WEIGHT;
    public float COHERE_WEIGHT;
    public float SEEK_WEIGHT;
    
    public void Invoke(IBoid _host)
    {
        host = _host;
        vSteer = Vector3.zero;
        vDesired = Vector3.zero;
        _vDesired = Vector3.zero;
        NeighborDistSqr = NEIGHBOR_DISTANCE * NEIGHBOR_DISTANCE;
        CohereDistSqr = COHERE_DISTANCE * COHERE_DISTANCE;
        SeparateDistSqr = SEPARATE_DISTANCE * SEPARATE_DISTANCE;
        AlignDistSqr = ALIGN_DISTANCE * ALIGN_DISTANCE;
    }

    public void Move()
    {
        vSteer += DoMove();
    }

    public void Seek(Vector3 target)
    {
        vSteerSeek = DoSeek(target) * SEEK_WEIGHT;
        //vSteer += vSteerSeek;
        _vDesired += vSteerSeek;
    }
    public void SeekArrive(Vector3 target)
    {
        vSteer += DoSeekArrive(target) * SEEK_WEIGHT;
    }
    public void Flee(Vector3 target)
    {
        vSteer += DoFlee(target);
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

    public void Align()
    {      
        Vector3 vAlign = DoAlign() * ALIGN_WEIGHT;
        //print("vAlign: " + vAlign);
        _vDesired += vAlign;
    }

    public void Separate()
    {
        vSteerSeparate = DoSeparate() * SEPARATION_WEIGHT;
        //print("vSeparate: " + vSteerSeparate);
        //vSteer += vSteerSeparate;
        _vDesired += vSteerSeparate;
    }

    public void Cohere()
    {
        Vector3 vCohere = DoCohere() * COHERE_WEIGHT;
        //print("vCohere: " + vCohere);
        _vDesired += vCohere;
    }

    public void Flock()
    {
        _vDesired += DoFlock();
    }

    public void CollisionAvoidance()
    {
        _vDesired += DoCollisionAvoidance();
    }

    void UpdateManager()
    {
        if (_vDesired.sqrMagnitude > host.MaxSpeed * host.MaxSpeed)
        {
            _vDesired = _vDesired.normalized * host.MaxSpeed;
        }
        vSteer = _vDesired - host.Velocity;
        if (vSteer.sqrMagnitude > host.MaxForce * host.MaxForce)
        {
            vSteer = vSteer.normalized * host.MaxForce;
        }
    }
    public void ResetManager()
    {
        vSteer = Vector3.zero;
        _vDesired = Vector3.zero;
    }

    Vector3 DoMove()
    {
        vDesired = transform.forward * host.MaxSpeed;
        // Seek(vDesired); ?
        return vDesired;
    }

    Vector3 DoSeek(Vector3 target)
    {
        // For each steering force, we need to take the planar projection of the vector that points from
        // the unit to the target, onto the ground normal. This is necessary because the unit cannot
        // move except in directions along the ground. Else the unit would be applying an upward force to reach an elevated target... But our unit cannot fly.
        Vector3 planarProj = Vector3.ProjectOnPlane((target - host.NavPoint.position), host.GroundNormal);
        vDesiredSeek = planarProj.normalized * host.MaxSpeed;

        return vDesiredSeek;
    }

    Vector3 DoSeekArrive(Vector3 target)
    {
        Vector3 planarProj = Vector3.ProjectOnPlane((target - host.NavPoint.position), host.GroundNormal);
        Vector3 vDesiredMax = planarProj.normalized * host.MaxSpeed;
        if (planarProj.sqrMagnitude < host.SlowingDistance * host.SlowingDistance)
        {
            float interpolationVal = (planarProj.magnitude - host.StoppingDistance) / (host.SlowingDistance - host.StoppingDistance);
            vDesired = Vector3.Lerp(Vector3.zero, vDesiredMax, interpolationVal);
        }
        else
        {
            vDesired = vDesiredMax;
        }

        return vDesired;
    }

    Vector3 DoFlee(Vector3 target)
    {
        Vector3 planarProj = Vector3.ProjectOnPlane((host.NavPoint.position - target), host.GroundNormal);
        vDesired = planarProj.normalized * host.MaxSpeed;
        return vDesired;
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

    Vector3 DoAlign()
    {
        Vector3 sumNeighborVelocity = Vector3.zero;
        int neighborCount = 0;
        foreach (IBoid neighbor in SimulationCtrl.Instance.agents)
        {
            if (neighbor == host)
                continue;

            Vector3 neighborToHost = host.NavPoint.position - neighbor.NavPoint.position;
            if (neighborToHost.sqrMagnitude <= AlignDistSqr)
            {
                float sqrMagnitude = (neighbor.NavPoint.position - host.NavPoint.position).sqrMagnitude;
                if (sqrMagnitude <= NeighborDistSqr)
                {
                    neighborCount++;

                    sumNeighborVelocity += neighbor.Velocity;
                }
            }
        }
        Vector3 avgNeighborVelocity = sumNeighborVelocity / neighborCount;

        vDesired = avgNeighborVelocity.normalized * host.MaxSpeed;
        return vDesired;
    }

    Vector3 DoSeparate()
    {
        neighbors.Clear();

        Vector3 sumSeparationForce = Vector3.zero;
        int neighborCount = 0;
        foreach(IBoid neighbor in SimulationCtrl.Instance.agents)
        {
            if (neighbor == host)
                continue;

            Vector3 neighborToHost = host.NavPoint.position - neighbor.NavPoint.position;
            if (neighborToHost.sqrMagnitude <= SeparateDistSqr)
            {
                neighbors.Add(neighbor);

                neighborCount++;
                if (linearSeparationForce)
                {
                    Vector3 vDesiredN = neighborToHost.normalized * Mathf.Lerp(host.MaxSpeed, 0, neighborToHost.magnitude / NEIGHBOR_DISTANCE);
                    //Vector3 vDesiredN = neighborToHost.normalized * host.MaxSpeed / neighborToHost.magnitude;
                    sumSeparationForce += vDesiredN;
                }
                else
                {
                    Vector3 vDesiredN = neighborToHost.normalized * Mathf.Lerp(host.MaxSpeed, 0, neighborToHost.sqrMagnitude / NeighborDistSqr);
                    //Vector3 vDesiredN = neighborToHost.normalized * host.MaxSpeed / neighborToHost.sqrMagnitude;
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

    Vector3 DoCohere()
    {
        Vector3 sumNeighborPosition = Vector3.zero;
        int neighborCount = 0;
        foreach(IBoid neighbor in SimulationCtrl.Instance.agents)
        {
            if (neighbor == host)
                continue;

            Vector3 neighborToHost = host.NavPoint.position - neighbor.NavPoint.position;
            if (neighborToHost.sqrMagnitude <= CohereDistSqr)
            {
                neighborCount++;
                Vector3 vPositionN = neighbor.NavPoint.position;
                sumNeighborPosition += vPositionN;
            }
        }

        if (neighborCount > 0)
        {
            Vector3 averageNeighborPosition = sumNeighborPosition / neighborCount;
            return (averageNeighborPosition - transform.position).normalized * host.MaxSpeed;
        }
        else
        {
            return Vector3.zero;
        }
    }

    Vector3 DoFlock()
    {
        Vector3 sumSeparationForce = Vector3.zero; 
        Vector3 sumNeighborPosition = Vector3.zero;
        Vector3 sumNeighborVelocity = Vector3.zero;
        int neighborsSeparateCount = 0;
        int neighborsAlignCount = 0;
        int neighborsCohereCount = 0;
        foreach (IBoid neighbor in SimulationCtrl.Instance.agents)
        {
            if (neighbor == host)
                continue;

            Vector3 neighborToHost = host.NavPoint.position - neighbor.NavPoint.position;

            if (neighborToHost.sqrMagnitude <= SeparateDistSqr)
            {
                neighborsSeparateCount++;
                if (linearSeparationForce)
                {
                    if (inverseSqareLaw)
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
                    if (inverseSqareLaw)
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
        
    Vector3 DoCollisionAvoidance()
    {
        Vector3 hitNormal;
        if (GetClosestObsticle(out hitNormal))
        {
            Vector3 vAvoid = Vector3.ProjectOnPlane(hitNormal * MAX_AVOIDANCE_FORCE, host.GroundNormal);
            return vAvoid - host.Velocity;
        }

        return Vector3.zero;
    }

    bool GetClosestObsticle(out Vector3 collisionPointNormal)
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
                if (angleOfSlope > 0)
                {
                    collisionPointNormal = lHit.normal;
                    return true;
                }
                else
                {
                    angleOfSlope = Vector3.Angle(transform.forward, Vector3.Cross(rHit.normal, -transform.right));
                    if (angleOfSlope > 0)
                    {
                        collisionPointNormal = rHit.normal;
                        return true;
                    }
                }
            }
            else
            {
                angleOfSlope = Vector3.Angle(transform.forward, Vector3.Cross(rHit.normal, -transform.right));
                if (angleOfSlope > 0)
                {
                    collisionPointNormal = rHit.normal;
                    return true;
                }
                else
                {
                    angleOfSlope = Vector3.Angle(transform.forward, Vector3.Cross(lHit.normal, -transform.right));
                    if (angleOfSlope > 0)
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
            if (angleOfSlope > 0)
            {
                collisionPointNormal = lHit.normal;
                return true;
            }
        }
        else if (Physics.Raycast(rightSide, transform.forward, out rHit, MAX_SEE_AHEAD, collisionAvoidanceLayer))
        {
            angleOfSlope = Vector3.Angle(transform.forward, Vector3.Cross(rHit.normal, -transform.right));
            if (angleOfSlope > 0)
            {
                collisionPointNormal = rHit.normal;
                return true;
            }
        }

        collisionPointNormal = Vector3.zero;
        return false;
    }

    void OnDrawGizmos()
    {
        if (host == null)
            return;

        Gizmos.color = Color.red;
        Gizmos.DrawLine(transform.position, transform.position + host.Velocity);

        //Gizmos.DrawLine(transform.position, transform.position + vDesiredSeek);
        //Gizmos.DrawLine(transform.position, transform.position + vDesiredSeparate);
        Gizmos.color = Color.blue;
        //Gizmos.DrawLine(transform.position, transform.position + vSteer * 10);
        Gizmos.DrawLine(transform.position, transform.position + vSteerSeek);
        Gizmos.color = Color.magenta;
        Gizmos.DrawLine(transform.position, transform.position + vSteerSeparate);

        if (neighbors.Count > 0)
        {
            foreach (IBoid n in neighbors)
            {
                //Gizmos.DrawLine(host.TransformPoint.position, n.TransformPoint.position);
            }
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
    Rigidbody rigidBody { get; }
}

public interface IObsticle
{
    Vector3 Center { get; }
    float CollisionRadius { get; }
    string Name { get; }
}
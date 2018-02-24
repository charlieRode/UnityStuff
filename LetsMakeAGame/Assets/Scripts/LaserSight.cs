using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class LaserSight : MonoBehaviour {

    public Color laserColor;
    public Transform scope;
    public LineRenderer lineRenderer;
    Ray gunFireRay;

    void Awake()
    {
        lineRenderer = GetComponent<LineRenderer>();
        lineRenderer.SetPosition(0, scope.position);
    }

    void Update()
    {
        lineRenderer.SetPosition(0, scope.position);
        gunFireRay.origin = scope.position;
        gunFireRay.direction = scope.forward;

        RaycastHit hit;
        if (Physics.Raycast(gunFireRay, out hit, 50f))
        {
            lineRenderer.SetPosition(1, hit.point);
        }
        else
        {
            lineRenderer.SetPosition(1, scope.position + gunFireRay.direction * 50);
        }

        Collider[] initialCollisions = Physics.OverlapSphere(scope.position, 0.1f);
        if (initialCollisions.Length > 0)
        {
            lineRenderer.SetPosition(1, scope.position);
        }
    }
}

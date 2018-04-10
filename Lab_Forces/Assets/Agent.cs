using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class Agent : MonoBehaviour, IBoid {

    Rigidbody rb;
    MovementManager moveManager;

    public LayerMask groundLayer;


    public bool useSeparation;
    public bool useAlignment;
    public bool useCohesion;
    
    public Rigidbody rigidBody
    {
        get
        {
            return rb;
        }
    }

    public Vector3 Velocity
    {
        get
        {
            if (rb != null)
            {
                return rb.velocity;
            }
            return Vector3.zero;
        }
    }

    public float maxForce;
    public float MaxForce
    {
        get
        {
            return maxForce;
        }
    }

    public float maxSpeed;
    public float MaxSpeed
    {
        get
        {
            return maxSpeed;
        }
    }

    public float slowingDistance;
    public float SlowingDistance
    {
        get
        {
            return slowingDistance;
        }
    }

    public float stoppingDistance;
    public float StoppingDistance
    {
        get
        {
            return stoppingDistance;
        }
    }

    public Transform TransformPoint
    {
        get
        {
            return transform;
        }
    }

    public Transform navPoint;
    public Transform NavPoint
    {
        get
        {
            return navPoint;
        }
    }

    Vector3 groundNormal;
    public Vector3 GroundNormal
    {
        get
        {
            return groundNormal;
        }
    }
    
    void Start()
    {
        rb = GetComponent<Rigidbody>();
        moveManager = GetComponent<MovementManager>();
        moveManager.Invoke(this);

        Initialize();
    }

    void FixedUpdate()
    {
        if (transform.position.x > SimulationCtrl.Instance.bounds.x)
        {
            transform.position = new Vector3(-SimulationCtrl.Instance.bounds.x, 1, transform.position.z);
        }
        if (transform.position.x < -SimulationCtrl.Instance.bounds.x)
        {
            transform.position = new Vector3(SimulationCtrl.Instance.bounds.x, 1, transform.position.z);
        }
        if (transform.position.z > SimulationCtrl.Instance.bounds.y)
        {
            transform.position = new Vector3(transform.position.x, 1, -SimulationCtrl.Instance.bounds.y);
        }
        if (transform.position.z < -SimulationCtrl.Instance.bounds.y)
        {
            transform.position = new Vector3(transform.position.x, 1, SimulationCtrl.Instance.bounds.y);
        }

        RaycastHit hit;
        if (Physics.Raycast(transform.position, Vector3.down, out hit, 10f, groundLayer))
        {
            groundNormal = hit.normal;
        }

        moveManager.ResetManager();
        moveManager.Flock();

        //moveManager.Separate();
        //moveManager.Align();
        //moveManager.Cohere();

        RaycastHit mousePoint;
        if (Physics.Raycast(Camera.main.ScreenPointToRay(Input.mousePosition), out mousePoint, 100f, groundLayer))
        {
            moveManager.Seek(mousePoint.point);
        }

        rb.AddForce(moveManager.Steering, ForceMode.VelocityChange);

    }

    void Initialize()
    {
        rb.velocity = Random.insideUnitSphere * Random.Range(MaxSpeed / 2f, MaxSpeed);
    }

}

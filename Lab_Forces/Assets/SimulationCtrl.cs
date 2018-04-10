using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SimulationCtrl : MonoBehaviour
{
    private static SimulationCtrl instance;
    public static SimulationCtrl Instance
    {
        get
        {
            return instance;
        }
    }

    public Vector2 bounds;
    public Agent[] agents;

    void Awake()
    {
        if (instance == null)
        {
            instance = this;
        }
        else
        {
            Destroy(gameObject);
        }
    }

    void Start()
    {
        agents = FindObjectsOfType<Agent>();
    }

    void OnDrawGizmos()
    {
        //Gizmos.DrawWireCube(Vector3.zero, new Vector3(bounds.x, 0, bounds.y));
    }
}

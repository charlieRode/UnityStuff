using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Obsticle : MonoBehaviour {

	public Vector3 Center
    {
        get
        {
            return transform.position;
        }
    }

    public float CollisionRadius
    {
        get
        {
            return transform.localScale.x / 2;
        }
    }

    public string Name
    {
        get
        {
            return gameObject.name;
        }
    }

}

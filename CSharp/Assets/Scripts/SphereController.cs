using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SphereController : MonoBehaviour {
    public float Control;
    public float OtherControl;
	// Use this for initialization
	void Start () {
		
	}
	
	// Update is called once per frame
	void Update () {

        Vector3 v = new Vector3();
        v.x = Mathf.Sin(Control) * OtherControl;
        v.y = Mathf.Cos(Control) * OtherControl;
        v.z = Control * OtherControl;

        transform.position = v;
	}
}

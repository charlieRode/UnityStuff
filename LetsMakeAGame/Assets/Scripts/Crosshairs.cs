using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Crosshairs : MonoBehaviour {

    public float rotationSpeed;
    public LayerMask targetMask;
    public SpriteRenderer centerDot;
    public Color centerDotColor;

    void Start()
    {
        Cursor.visible = false;
        centerDotColor = centerDot.color;
    }

    void Update()
    {
        transform.Rotate(Vector3.forward * rotationSpeed * Time.deltaTime);
    }

    public void DetectTargets(Ray ray)
    {
        if (Physics.Raycast(ray, 100f, targetMask))
        {
            centerDot.color = Color.red;
        }
        else
        {
            centerDot.color = centerDotColor;
        }
    }
}

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraCtrl : MonoBehaviour {

    public float scrollZoneSlow = 75f;
    public float scrollZoneFast = 1f;
    public float scrollSpeed = 5f;

    public float xMin = -30f;
    public float xMax = 30f;
    public float zMin = -30f;
    public float zMax = 30f;

    public int zoomIncrement = 1;
    public float yMin = 20f;
    public float yMax = 60f;

    public float cameraRotationSpeed;

    Vector3 desiredPosition;
    Vector3 movement;

    void Start()
    {
        desiredPosition = transform.position;
    }

    void Update()
    {
        if (Input.GetAxis("Mouse ScrollWheel") > 0 && transform.position.y > yMin)
            Zoom(1);
        else if (Input.GetAxis("Mouse ScrollWheel") < 0 && transform.position.y < yMax)
            Zoom(-1);

        float x = 0, z = 0;
        if (Input.mousePosition.x < scrollZoneFast)
            x -= (scrollSpeed * 2 * Time.deltaTime);
        else if (Input.mousePosition.x < scrollZoneSlow)
            x -= (scrollSpeed * Time.deltaTime);

        else if (Input.mousePosition.x > Screen.width - scrollZoneFast)
            x += (scrollSpeed * 2 * Time.deltaTime);
        else if (Input.mousePosition.x > Screen.width - scrollZoneSlow)
            x += (scrollSpeed * Time.deltaTime);


        if (Input.mousePosition.y < scrollZoneFast)
            z -= (scrollSpeed * 2 * Time.deltaTime);
        else if (Input.mousePosition.y < scrollZoneSlow)
            z -= (scrollSpeed * Time.deltaTime);

        else if (Input.mousePosition.y > Screen.height - scrollZoneFast)
            z += (scrollSpeed * 2 * Time.deltaTime);
        else if (Input.mousePosition.y > Screen.height - scrollZoneSlow)
            z += (scrollSpeed * Time.deltaTime);


        movement = new Vector3(x, 0, z) + desiredPosition;
        movement.x = Mathf.Clamp(movement.x, xMin, xMax);
        movement.z = Mathf.Clamp(movement.z, zMin, zMax);

        desiredPosition = movement;

        transform.position = Vector3.Lerp(transform.position, desiredPosition, 0.2f);
    }

    void Zoom(int orientation)
    {
        movement = transform.forward * zoomIncrement * orientation + desiredPosition;
        desiredPosition = movement;
        transform.position = Vector3.Lerp(transform.position, desiredPosition, 0.2f);
    }
}

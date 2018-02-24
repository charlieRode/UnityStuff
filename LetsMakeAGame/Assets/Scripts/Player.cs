using System.Collections;
using System.Collections.Generic;
using UnityEngine;


// By invoking GetComponent<PlayerController>() we are assuming that a PlayerController script
// will be attached to the same game object as this script.
// By adding the RequireComponent attribute to this class, we ensure that both scripts are present.
[RequireComponent (typeof (PlayerController))]
[RequireComponent (typeof (GunController))]
public class Player : Character {
    public Crosshairs crosshairs;
    public float _verticalOffset = 1f;
    public Transform weaponT;
    public float moveSpeed = 5f;
    PlayerController controller;
    public event System.Action OnTakeHit;
    GunController gunCtrl;
    public GunController GunCtrl
    {
        get { return gunCtrl; }
    }
    public float enableAimingRadius = 1f;

    public Camera viewCamera;
    Vector3 cameraOffset;

    public AudioClip deathAudioClip;

    public bool recievedWeaponUpgrade;
    public bool recievedAmmoUpgrade;

    void Awake()
    {
        controller = GetComponent<PlayerController>();
        gunCtrl = GetComponent<GunController>();
    }

    protected override void Start()
    {
        base.Start();
        cameraOffset = viewCamera.transform.position - transform.position;
    }

    protected override void Update()
    {
        base.Update();

        // Movement input
        Vector3 moveInput = new Vector3(Input.GetAxisRaw("Horizontal"), 0, Input.GetAxisRaw("Vertical")).normalized;
        Vector3 moveVelocity = moveInput * moveSpeed;
        controller.Move(moveVelocity);

        // Look input
        //Plane groundPlane = new Plane(Vector3.up, Vector3.zero);
        Plane groundPlane = new Plane(Vector3.up, Vector3.up * _verticalOffset);
        Ray ray = viewCamera.ScreenPointToRay(Input.mousePosition);
        float distanceToGroundPlane;
        if (groundPlane.Raycast(ray, out distanceToGroundPlane))
        {
            Vector3 pointOfIntersection = ray.GetPoint(distanceToGroundPlane);
            pointOfIntersection = new Vector3(pointOfIntersection.x, _verticalOffset, pointOfIntersection.z);
            controller.LookAt(pointOfIntersection);
            crosshairs.transform.position = pointOfIntersection;
            crosshairs.DetectTargets(ray);

            if (Vector2.Distance(new Vector2(pointOfIntersection.x, pointOfIntersection.z), new Vector2(transform.position.x, transform.position.z)) > enableAimingRadius)
            {
                gunCtrl.Aim(pointOfIntersection);
            }
            
        }

        // Weapon input
        if (Input.GetMouseButtonUp(0))
        {
            gunCtrl.OnTriggerRelease();
        }
        if (Input.GetMouseButtonDown(0))
        {
            gunCtrl.OnTriggerHold();
        }
        if (Input.GetMouseButtonDown(1))
        {
            gunCtrl.Reload();
        }

        CameraFollowPlayer();
    }

    void CameraFollowPlayer()
    {
        viewCamera.transform.position = transform.position + cameraOffset;
    }

    public override void TakeHit(float damage, Vector3 hitPoint, Vector3 hitDirection)
    {
        if (damage >= Health)
        {
            AudioManager.instance.PlaySound(deathAudioClip, transform.position);
            Destroy(Instantiate(deathEffect.gameObject, transform.position, Quaternion.FromToRotation(Vector3.forward, hitDirection)) as GameObject, deathEffect.main.startLifetime.constant);
        }
        base.TakeHit(damage, hitPoint, hitDirection);

        if (OnTakeHit != null)
            OnTakeHit();
    }

}

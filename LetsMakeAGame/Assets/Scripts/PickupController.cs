using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PickupController : MonoBehaviour {

    public float rotationSpeed;
    public ParticleSystem effect;
    Collider sphereCollider;

    void Awake()
    {
        sphereCollider = GetComponent<SphereCollider>();
    }

    protected virtual void Update()
    {
        transform.Rotate(Vector3.forward * rotationSpeed * Time.deltaTime);
    }

    protected virtual void PickUp(Collider c)
    {
        Instantiate(this.effect, transform.position, transform.rotation);
        AudioManager.instance.PlaySound("Pickup", transform.position);
        Destroy(gameObject);
    }

    void OnTriggerEnter(Collider c)
    {
        if (c.CompareTag("Player"))
        {
            PickUp(c);
        }
    }

}

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Projectile : MonoBehaviour {

    public static System.Random randGen;

    float projectileSpeed = 10f;
    public float damage = 1;
    public Color trailColor;
    // We are checking collisions using a ray projected out from the projectile, the length of the distance the projectile will travel the next frame. 
    // But. If the target is ALSO moving towards the projectile, the length of the array needs to be slightly longer. 
    // Specifically, I'd guess it needs to be the distance (magnitude) the projectile will travel in a frame + the distance (magnitude) the enemy 
    // will travel in a frame. But this would require getting a reference to the enemy in the projectile class, and the value would change depending on
    // how fast each enemy is traveling. So instead we'll just add a constant value, skinWidth, to the length of the array, and adjust according to the needs of our game.
    float skinWidth = 0.1f;
    float timeToExpiration = 3f;
    public bool piercing;
    bool hasPierced;

    float moveDistance;

    public LayerMask collisionMask;
    public ParticleSystem bulletImpactObsticleEffect;

    private void Awake()
    {
        if (randGen == null)
            randGen = new System.Random();
    }

    void Start()
    {
        Destroy(gameObject, timeToExpiration);
        TrailRenderer tr = GetComponentInChildren<TrailRenderer>();
        if (tr != null)
            tr.material.SetColor("_TintColor", trailColor);

        Collider[] initialCollisions = Physics.OverlapSphere(transform.position, 0.1f, collisionMask);
        if (initialCollisions.Length > 0)
        {
            OnHitObject(initialCollisions[0], transform.position);
        }
    }

    public void SetProjectileSpeed(float speed)
    {
        projectileSpeed = speed;
    }
	void FixedUpdate()
    {
        moveDistance = projectileSpeed * Time.deltaTime;
        // float moveDistance = projectileSpeed * Time.deltaTime;
        CheckCollisions(moveDistance);
        transform.Translate(Vector3.forward * moveDistance);
    }

    void CheckCollisions(float moveDistance)
    {
        Ray ray = new Ray(transform.position, transform.forward);
        RaycastHit hit;
        if (Physics.Raycast(ray, out hit, moveDistance + skinWidth, collisionMask, QueryTriggerInteraction.Collide))
        {
            OnHitObject(hit.collider, hit.point);
        }

    }

    void OnHitObject(Collider c, Vector3 hitPoint)
    {
        AudioManager.instance.PlaySound("ImpactSFX", c.transform.position);
        if (c.CompareTag("Enemy"))
        {
            IDamagable target = c.GetComponent<IDamagable>();
            if (target != null)
                target.TakeHit(damage, hitPoint, transform.forward);

            if (!piercing || hasPierced)
            {
                Destroy(gameObject);
            }
            else
            {
                hasPierced = true;
            }
        }
        else if (c.CompareTag("Obsticle"))
        {
            Destroy(Instantiate(bulletImpactObsticleEffect.gameObject, transform.position + (hitPoint - transform.position) / 2, Quaternion.FromToRotation(Vector3.forward, -transform.forward)) as GameObject, bulletImpactObsticleEffect.main.startLifetime.constant);
            Destroy(gameObject);
        }
        else
        {
            Destroy(gameObject);
        }
    }
}

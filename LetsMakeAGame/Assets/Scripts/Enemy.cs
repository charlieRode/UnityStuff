using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

[RequireComponent (typeof (NavMeshAgent))]
public class Enemy : Character {

    public enum State { IDLE, CHASING, ATTACKING };
    public State currentState = State.IDLE;

    public float damageEffectDuration = 0.5f;

    public int damage = 1;
    public float attackSpeed = 3f;

    protected float attackDistanceThreshold = 0.5f;
    float attackFrequency = 1f;
    float timeSinceLastAttack = 0f;

    bool hasTarget = false;
    float myCollisionRadius;
    float targetCollisionRadius;

    Material skinMaterial;
    Color skinColor;

    Character targetCharacter;
    Transform target;
    NavMeshAgent pathFinder;

    public AudioClip attackAudioClip;

    protected virtual void Awake()
    {
        pathFinder = GetComponent<NavMeshAgent>();
        // Need to enable pathfinder one frame after instantiation in order to aviod a strange bug that I can't make sense of.
        pathFinder.enabled = false;

        if (GameObject.FindWithTag("Player") != null)
        {
            target = GameObject.FindWithTag("Player").transform;
            targetCharacter = target.GetComponent<Character>();
            hasTarget = true;

            myCollisionRadius = GetComponent<CapsuleCollider>().radius;
            targetCollisionRadius = target.GetComponent<CapsuleCollider>().radius;
        }
    }

    protected override void Start()
    {
        base.Start();

        if (hasTarget)
        {
            targetCharacter.OnDeath += OnTargetDeath;
            StartCoroutine(UpdatePath());
        }
    }

    public void Activate()
    {
        if (pathFinder != null)
        {
            pathFinder.enabled = true;
            currentState = State.CHASING;
        }
    }

    public void SetCharacteristics(float moveSpeed, int damage, float health, Color skinColor)
    {
        if (pathFinder == null)
            return;

        pathFinder.speed = moveSpeed;
        this.damage = damage;
        this.startingHealth = health;
        this.Health = health;

        skinMaterial = GetComponent<Renderer>().material;
        skinMaterial.color = skinColor;
        this.skinColor = skinMaterial.color;
        this.deathEffect.GetComponent<Renderer>().sharedMaterial.color = skinColor;
    }

    protected override void Update()
    {
        if (hasTarget)
        {
            if (timeSinceLastAttack >= attackFrequency)
            {
                float sqrTargetDistance = (target.position - transform.position).sqrMagnitude;
                if (sqrTargetDistance < Mathf.Pow(attackDistanceThreshold + myCollisionRadius + targetCollisionRadius, 2))
                {
                    timeSinceLastAttack = 0f;
                    AudioManager.instance.PlaySound(attackAudioClip, transform.position);
                    StartCoroutine(AttackPlayer());
                }
            }
            timeSinceLastAttack += Time.deltaTime;
        }
    }

    public override void TakeHit(float damage, Vector3 hitPoint, Vector3 hitDirection)
    {
        if (damage >= health)
        {
            AudioManager.instance.PlaySound("EnemyDeathSFX", transform.position);
            Destroy(Instantiate(deathEffect.gameObject, hitPoint, Quaternion.FromToRotation(Vector3.forward, hitDirection)) as GameObject, deathEffect.main.startLifetime.constant);
        }

        base.TakeHit(damage, hitPoint, hitDirection);
        if (gameObject != null)
            StartCoroutine(DamageEffect());
    }

    void OnTargetDeath()
    {
        hasTarget = false;
        currentState = State.IDLE;
    }

    IEnumerator DamageEffect()
    {
        float pct = 0f;
        float speed = 1 / damageEffectDuration;
        while (pct < 1)
        {
            // f(x) = -4(x^2 - x)
            float interpolationValue = -4 * (Mathf.Pow(pct, 2) - pct);
            if (skinMaterial != null)
            {
                skinMaterial.color = Color.Lerp(skinColor, Color.white, interpolationValue);
            }

            pct += Time.deltaTime * speed;
            yield return null;
        }

        skinMaterial.color = skinColor;
    }

    IEnumerator AttackPlayer()
    {
        currentState = State.ATTACKING;
        pathFinder.enabled = false;
        skinMaterial.color = Color.red;

        Vector3 originalPosition = transform.position;

        Vector3 targetDirection = (target.position - transform.position).normalized;
        Vector3 attackPosition = target.position - targetDirection * myCollisionRadius;
        float percent = 0f;
        bool hasAppliedDamage = false;

        while (percent <= 1)
        {
            percent += Time.deltaTime * attackSpeed;

            // quadratic distribution : y = 4(-x^2 + x)
            // float interpolationValue = (-Mathf.Pow(percent, 2) + percent) * 4;

            // bell curve distribution
            float interpolationValue = MathUtils.Gaussian01(percent);        

            transform.position = Vector3.Lerp(originalPosition, attackPosition, interpolationValue);

            if (percent >= 0.5f && !hasAppliedDamage)
            {
                targetCharacter.TakeHit(damage, transform.position, targetDirection);
                hasAppliedDamage = true;
            }

            yield return null;
        }

        skinMaterial.color = skinColor;
        pathFinder.enabled = true;
        currentState = State.CHASING;
    }

    // It's important to understand the performance bottlenecks...
    // One area succeptible to performants hits by inefficient code is around calculations that run each frame.
    // At 60FPS, thats 60X the code is run per second. Values that are regularly calculated need not necessarily be
    // calculated every frame. By running this code every quarter of a second, the values are updated frequently enough to
    // be imperceptible, but not so frequently as to cause significant overhead.
    // 60 / 4 = 15. Thats a 15X performance improvement!
    IEnumerator UpdatePath()
    {
        float refreshRate = .25f;

        while (hasTarget)
        {
            if (currentState == State.CHASING && pathFinder.enabled)
            {
                Vector3 targetDirection = (target.position - transform.position).normalized;
                Vector3 targetPositionRaw = target.position - targetDirection * (myCollisionRadius + targetCollisionRadius + attackDistanceThreshold / 2f);
                Vector3 targetPosition = new Vector3(targetPositionRaw.x, 0, targetPositionRaw.z);
                if (!isDead)
                    pathFinder.SetDestination(targetPosition);
            }

            yield return new WaitForSeconds(refreshRate);
        }
    }
}

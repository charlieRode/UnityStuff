using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Character : MonoBehaviour, IDamagable
{
    public float startingHealth;
    protected float health;
    public float Health
    {
        get
        {
            return health;
        }
        set
        {
            health = value;
            if (OnUpdateHealth != null)
            {
                OnUpdateHealth();
            }
        }
    }

    protected bool isDead;
    public ParticleSystem deathEffect;

    public event Action OnUpdateHealth;
    public event Action OnDeath;

    protected virtual void Start()
    {
        health = startingHealth;
    }

    protected virtual void Update()
    {
        if (transform.position.y < -1f)
        {
            CharacterDeath();
        }
    }

    public virtual void TakeHit(float damage, Vector3 hitPoint, Vector3 hitDirection)
    {
        TakeDamage(damage);
    }

    public virtual void TakeDamage(float damage)
    {
        Health -= damage;
        if (Health <= 0 && !isDead)
        {
            CharacterDeath();
        }
    }

    [ContextMenu("Self Destruct")]
    protected void CharacterDeath()
    {
        isDead = true;
        OnDeath.Invoke();
        Destroy(gameObject);
    }

    public void ResetHealth()
    {
        Health = startingHealth;
    }
}

using UnityEngine;
using System;
using System.Collections;

public interface IDamagable
{
    void TakeHit(float damage, Vector3 position, Vector3 direction);

    void TakeDamage(float damage);
}
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

public class Boss : Enemy {
    
    public int _Bhealth;
    public int _Bdamage;
    public float _BmoveSpeed;
    public Color _BskinColor;

    protected override void Awake()
    {
        _BskinColor = MathUtils.RandomColor();
        base.Awake();
        this.SetCharacteristics(_BmoveSpeed, _Bdamage, _Bhealth, _BskinColor);
        this.attackDistanceThreshold = 2.5f;
    }
}

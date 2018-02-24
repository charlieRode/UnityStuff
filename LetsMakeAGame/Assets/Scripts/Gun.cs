using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

public class Gun : MonoBehaviour
{
    // Gun Configuration Settings
    public string name;
    public enum FireMode { AUTOMATIC, BURST, SINGLE_SHOT };
    public enum ReloadMode { TILT, SPIN, SHOTGUN };
    [Header("Configuration Settings")]
    public FireMode fireMode;
    public ReloadMode reloadMode;
    public event Action OnAmmoChange;
    public int magazineSize = 6;
    int ammoInMag;
    public int Ammo
    {
        get
        {
            return ammoInMag;
        }
        private set
        {
            ammoInMag = value;
            if (OnAmmoChange != null)
                OnAmmoChange();
        }
    }
    public float reloadAngle;
    public Transform[] projectileSpawns;
    public Projectile projectile;
    public float msBetweenShots = 100f;
    public float projectileVelocity = 35f;
    public float reloadTime = 1f;
    public Gun upgradedGun;

    public float randomAngleBound;
    [Range(0, 1)]
    public float accuracy;

    // Applicable if fire mode is BURST
    [Header("BURST Settings")]
    public int _burstCount;
    public float _burstSpeedMS;

    [Header("Recoil Settings")]
    public Vector2 recoilDistanceRange;
    public Vector2 recoilAngleStepRange;
    public float maxRecoilAngle;
    public float recoilTime;

    [Header("Effects Settings")]
    public Transform shellPrefab;
    public Transform[] shellEjectionPoints;
    public AudioClip fireAudioClip;
    public AudioClip reloadAudioClip;
    MuzzleFlash muzzleFlash;
    LaserSight laserScope;
    bool hasLasterScope = false;

    // Internal State
    IEnumerator reloadCR;
    float msSinceLastFired;
    float recoilAngle;
    float smoothReloadAngle;
    bool isReloading;
    bool triggerReleasedSinceLastShot;
    bool previousVal;
    bool initialized;

    // For the SmoothDamp function
    Vector3 _currentVelocity;
    float _recoilAngularVelocity;
    float _reloadAngularVelocity;

    void Awake()
    {
        laserScope = GetComponentInChildren<LaserSight>();
        hasLasterScope = true;
    }

    void Start()
    {
        muzzleFlash = GetComponent<MuzzleFlash>();
        msSinceLastFired = msBetweenShots;
        triggerReleasedSinceLastShot = true;
        Ammo = magazineSize;
    }

    void Update()
    {
        msSinceLastFired += Time.deltaTime * 1000;
    }

    void LateUpdate()
    {
        if (!isReloading)
        {
            transform.localPosition = Vector3.SmoothDamp(transform.localPosition, Vector3.zero, ref _currentVelocity, recoilTime);
            recoilAngle = Mathf.SmoothDamp(recoilAngle, 0, ref _recoilAngularVelocity, recoilTime);

            // I don't know how else to do this. If we are explicitly setting only 1 axis, and keeping the others the same,
            // I can't think of another pattern that will accomplish this.
            transform.localEulerAngles = new Vector3(-recoilAngle, transform.localEulerAngles.y, transform.localEulerAngles.z);

        }
    }

    void Fire()
    {
        switch (fireMode)
        {
            case FireMode.AUTOMATIC:          
                StartCoroutine(AutomaticFire());
                break;

            case FireMode.SINGLE_SHOT:
                if (msSinceLastFired >= msBetweenShots && triggerReleasedSinceLastShot && Ammo > 0 && !isReloading)
                {
                    FireRound();
                    msSinceLastFired = 0f;
                }
                break;

            case FireMode.BURST:
                if (msSinceLastFired >= msBetweenShots && triggerReleasedSinceLastShot && !isReloading)
                {
                    StartCoroutine(BurstFire());
                    msSinceLastFired = 0f;
                }
                break;

            default:
                break;
        }
    }

    public void OnTriggerHold()
    {
        Fire();
        triggerReleasedSinceLastShot = false;
    }

    void Recoil()
    {
        transform.localPosition -= Vector3.forward * UnityEngine.Random.Range(recoilDistanceRange.x, recoilDistanceRange.y);
        recoilAngle += UnityEngine.Random.Range(recoilAngleStepRange.x, recoilAngleStepRange.y);
        recoilAngle = Mathf.Clamp(recoilAngle, 0, maxRecoilAngle);
    }

    public void OnTriggerRelease()
    {
        triggerReleasedSinceLastShot = true;
    }

    public void Aim(Vector3 target)
    {
        if (isReloading)
            return;

        Vector3 aimPosition = (target - transform.position).normalized;
        float lookRotation = Mathf.Atan2(aimPosition.x, aimPosition.z) * Mathf.Rad2Deg;
        transform.rotation = Quaternion.Euler(new Vector3(transform.rotation.x, lookRotation, transform.rotation.z));

        // Are the above and the below the same?
        // transform.eulerAngles = new Vector3(transform.rotation.x, lookRotation, transform.rotation.z);
    }

    IEnumerator _TiltReload()
    {
        BeginReload();
        float pct = 0f;
        float _speed = 1f / reloadTime;

        Vector3 initialRotation = transform.localEulerAngles;

        while (pct < 1)
        {
            pct += _speed * Time.deltaTime;
            float interpolationValue = -4 * (Mathf.Pow(pct, 2) - pct);
            float gunAngle = Mathf.Lerp(0, reloadAngle, interpolationValue);
            transform.localEulerAngles = initialRotation + Vector3.left * gunAngle;

            yield return null;
        }
        Ammo = magazineSize;
        FinishReload();
    }

    IEnumerator _ShotgunReload()
    {
        BeginReload();
        float pct = 0f;
        float _speed = 1f / reloadTime;

        Vector3 initialRotation = transform.localEulerAngles;
        Vector3 stage3InitialPosition = Vector3.zero;


        while (pct < 1)
        {
            pct += _speed * Time.deltaTime;

            float t1 = 0.25f;
            float t2 = 0.9f;

            // Stage 1
            if (pct <= t1)
            {
                float x = pct / t1;
                float gunAngle = Mathf.Lerp(0, reloadAngle, x);
                transform.localEulerAngles = initialRotation + Vector3.left * gunAngle;
                transform.localPosition = Vector3.Lerp(Vector3.zero, Vector3.forward * -0.4f, x);
            }
            // Stage 2
            else if (pct <= t2)
            {
                float x = (pct - t1) / (t2 - t1);
                float interpolationValue = interpolationValue = -4 * (Mathf.Pow(x, 2) - x);

                Vector3 relativePos = new Vector3(transform.localPosition.x, 0, transform.localPosition.z);
                transform.localPosition = Vector3.Lerp(relativePos, relativePos + Vector3.up * -0.5f, interpolationValue);
            }
            // Stage 3
            else
            {
                float x = (pct - t2) / (1 - t2);

                if (stage3InitialPosition == Vector3.zero)
                {
                    stage3InitialPosition = transform.localPosition;
                }

                float gunAngle = Mathf.Lerp(reloadAngle, 0f, x);               
                transform.localEulerAngles = -Vector3.right * gunAngle;
                transform.localPosition = Vector3.Lerp(stage3InitialPosition, Vector3.zero, x);                       
            }
            yield return null;
        }
        Ammo = magazineSize;
        FinishReload();
    }

    IEnumerator _SpinReload()
    {
        BeginReload();
        smoothReloadAngle = 0f;
        while (smoothReloadAngle < 360)
        {
            smoothReloadAngle = Mathf.SmoothDamp(smoothReloadAngle, 360 + 0.1f, ref _reloadAngularVelocity, reloadTime);
            transform.localEulerAngles = (-Vector3.right * smoothReloadAngle);
            yield return null;
        }
        Ammo = magazineSize;
        FinishReload();
    }

    IEnumerator BurstFire()
    {
        int remainingBullets = Mathf.Min(_burstCount, Ammo);
        float timeBetweenBulletsMS = _burstSpeedMS / _burstCount;
        float timeSinceLastBulletMS = timeBetweenBulletsMS;

        while (remainingBullets > 0)
        {
            if (timeSinceLastBulletMS >= timeBetweenBulletsMS)
            {
                if (Ammo > 0)
                {
                    FireRound();
                    timeSinceLastBulletMS = 0f;
                    remainingBullets--;
                }
            }

            timeSinceLastBulletMS += Time.deltaTime * 1000f;
            yield return null;
        }
    }

    IEnumerator AutomaticFire()
    {
        triggerReleasedSinceLastShot = false;
        while (!triggerReleasedSinceLastShot)
        {
            if (msSinceLastFired >= msBetweenShots && Ammo > 0 && !isReloading)
            {
                FireRound();
                msSinceLastFired = 0f;
            }

            yield return null;
        }
    }

    void BeginReload()
    {
        isReloading = true;
        if (hasLasterScope && laserScope != null)
        {
            laserScope.lineRenderer.enabled = false;
        }
    }

    void FinishReload()
    {
        isReloading = false;
        if (hasLasterScope && laserScope != null)
        {
            laserScope.lineRenderer.enabled = true;
        }
    }

    void FireRound()
    {
        for (int i = 0; i < projectileSpawns.Length; i++)
        {
            // Inject inaccuracy.
            Vector3 projectileAngle = projectileSpawns[i].eulerAngles;
            if (randomAngleBound > 0)
            {
                float angleNoiseX = UnityEngine.Random.value * randomAngleBound;
                float angleNoiseY = UnityEngine.Random.value * randomAngleBound;
                projectileAngle = projectileSpawns[i].eulerAngles + new Vector3(angleNoiseX, angleNoiseY, 0);
            }

            Projectile bullet = Instantiate(projectile, projectileSpawns[i].position, Quaternion.Euler(projectileAngle)) as Projectile;
            bullet.SetProjectileSpeed(projectileVelocity);
            Instantiate(shellPrefab.gameObject, shellEjectionPoints[i].position, Quaternion.Euler(Vector3.right * 90));
        }
        muzzleFlash.Activate();
        AudioManager.instance.PlaySound(fireAudioClip, transform.position);
        Recoil();
        Ammo--;
    }

    public void Reload()
    {
        if (isReloading)
            return;

        if (reloadCR != null)
        {
            StopCoroutine(reloadCR);
        }

        switch (reloadMode)
        {
            case ReloadMode.TILT:
                reloadCR = _TiltReload();
                break;

            case ReloadMode.SPIN:
                reloadCR = _SpinReload();
                break;

            case ReloadMode.SHOTGUN:
                reloadCR = _ShotgunReload();
                break;
        }

        StartCoroutine(reloadCR);
        AudioManager.instance.PlaySound(reloadAudioClip, transform.position);
    }

    public void SetFlashColor(Color color)
    {
        if (muzzleFlash != null)
            muzzleFlash.SetColor(color);
    }
}

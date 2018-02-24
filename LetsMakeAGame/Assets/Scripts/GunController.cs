using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GunController : MonoBehaviour {

    public Transform weaponHold;
    public List<Gun> guns;
    public Gun equippedGun;
    public event System.Action OnEquipGun;

    public int remainingAmmo;

    void Awake()
    {
        if (guns.Count > 0)
        {
            EquipGun(guns[WeaponSelect.selection]);
        }
    }

    void Update()
    {
        remainingAmmo = equippedGun.Ammo;
    }

    public void Aim(Vector3 target)
    {
        if (equippedGun != null)
            equippedGun.Aim(target);
    }

    public void Reload()
    {
        if (equippedGun != null)
        {
            equippedGun.Reload();
        }
    }

    public void EquipGun(Gun gunToEquip)
    {
        if (equippedGun != null)
            Destroy(equippedGun.gameObject);

        equippedGun = Instantiate(gunToEquip, weaponHold.position, weaponHold.rotation) as Gun;
        equippedGun.transform.parent = weaponHold;

        if (OnEquipGun != null)
            OnEquipGun();
    }

    public void OnTriggerHold()
    {
        if (equippedGun != null)
        {
            equippedGun.OnTriggerHold();
        }
    }

    public void OnTriggerRelease()
    {
        if (equippedGun != null)
        {
            equippedGun.OnTriggerRelease();
        }
    }
}

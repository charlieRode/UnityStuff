using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class UpgradePickupController : PickupController
{

    protected override void PickUp(Collider c)
    {
        GunController gunCtrl = c.GetComponent<Player>().GunCtrl;
        gunCtrl.EquipGun(gunCtrl.equippedGun.upgradedGun);

        base.PickUp(c);
        c.GetComponent<Player>().recievedWeaponUpgrade = true;
    }
}

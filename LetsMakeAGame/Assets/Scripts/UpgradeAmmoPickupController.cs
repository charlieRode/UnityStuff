using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class UpgradeAmmoPickupController : PickupController {

    public AmmoUpgrade[] upgrades;

    protected override void PickUp(Collider c)
    {
        GunController ctrl = c.GetComponent<Player>().GunCtrl;
        foreach (AmmoUpgrade upgrade in upgrades)
        {
            if (upgrade.name == ctrl.equippedGun.name)
            {
                ctrl.equippedGun.projectile = upgrade.ammoPrefab;
                ctrl.equippedGun.SetFlashColor(upgrade.ammoColor);

                if (ctrl.equippedGun.upgradedGun != null)
                {
                    ctrl.equippedGun.upgradedGun.projectile = upgrade.ammoPrefab;
                    ctrl.equippedGun.upgradedGun.SetFlashColor(upgrade.ammoColor);
                }
            }
        }

        base.PickUp(c);
        c.GetComponent<Player>().recievedAmmoUpgrade = true;
    }

    [System.Serializable]
    public struct AmmoUpgrade
    {
        public string name;
        public Projectile ammoPrefab;
        public Color ammoColor;
    }
}

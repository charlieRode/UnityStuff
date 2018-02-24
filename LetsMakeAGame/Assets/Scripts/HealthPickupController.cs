using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class HealthPickupController : PickupController {

    protected override void PickUp(Collider c)
    {
        c.GetComponent<Player>().ResetHealth();
        base.PickUp(c);
    }
}

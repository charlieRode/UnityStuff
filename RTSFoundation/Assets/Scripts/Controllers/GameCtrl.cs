using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GameCtrl : MonoBehaviour {

    UnitOrdersCtrl unitOrdersCtrl;
    //public List<UnitController> activeUnitsInGame;
    public List<UnitCtrl> activeUnitsInGame;
    public LayerMask groundLayer;

    static GameCtrl _instance;
    public static GameCtrl Instance
    {
        get { return _instance; }
    }

    void Awake()
    {
        if (_instance != null)
        {
            Destroy(gameObject);
        }
        else
        {
            _instance = this;
        }

        //activeUnitsInGame = new List<UnitController>();
        activeUnitsInGame = new List<UnitCtrl>();
        unitOrdersCtrl = GetComponent<UnitOrdersCtrl>();
    }

    /*
    public void RegisterActiveUnit(UnitController unit)
    {
        activeUnitsInGame.Add(unit);
    }
    public void UnregisterActiveUnit(UnitController unit)
    {
        activeUnitsInGame.Remove(unit);
    }
    */

    // Overloads for new controller
    public void RegisterActiveUnit(UnitCtrl unit)
    {
        activeUnitsInGame.Add(unit);
    }
    public void UnregisterActiveUnit(UnitCtrl unit)
    {
        activeUnitsInGame.Remove(unit);
    }

    void Update()
    {
        if (Input.GetMouseButtonDown(1))
        {
            Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
            RaycastHit rHit;
            if (Physics.Raycast(ray, out rHit, 100, groundLayer))
            {
                unitOrdersCtrl.ExecuteMoveOrder(rHit.point);
            }
        }
    }
}

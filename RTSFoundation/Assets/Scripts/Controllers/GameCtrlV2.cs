using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GameCtrlV2 : MonoBehaviour
{

    UnitOrdersCtrlV2 unitOrdersCtrl;
    public List<UnitController> activeUnitsInGame;
    public LayerMask groundLayer;

    static GameCtrlV2 _instance;
    public static GameCtrlV2 Instance
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

        activeUnitsInGame = new List<UnitController>();
        unitOrdersCtrl = GetComponent<UnitOrdersCtrlV2>();
    }


    // Overloads for new controller
    public void RegisterActiveUnit(UnitController unit)
    {
        activeUnitsInGame.Add(unit);
    }
    public void UnregisterActiveUnit(UnitController unit)
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

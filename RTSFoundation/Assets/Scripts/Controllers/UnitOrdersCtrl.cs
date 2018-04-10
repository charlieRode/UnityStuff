using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class UnitOrdersCtrl : MonoBehaviour {

    public event System.Action<Vector3> moveUnitsConvergeOnPoint;
    public event System.Action<Vector3, Vector3> moveUnitsKeepFormation;

    static UnitOrdersCtrl _instance;
    public static UnitOrdersCtrl Instance
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
    }

    public void ExecuteMoveOrder(Vector3 destination)
    {
        Vector3 anchor = UnitSelectionCtrl.Instance.GetAnchorPosition();
        if (UnitSelectionCtrl.Instance.GetBoundingBoxDiagonal() > UnitSelectionCtrl.Instance.SelectedUnitCount || UnitSelectionCtrl.Instance.BoundingBoxContainsPoint(destination))
        {
            //print("converge on point");
            if (moveUnitsConvergeOnPoint != null)
                moveUnitsConvergeOnPoint(destination);
        }
        else
        {
            //print("keep formation");
            if (moveUnitsKeepFormation != null)
                moveUnitsKeepFormation(destination, anchor);
        }
    }
}

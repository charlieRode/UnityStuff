using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class UnitOrdersCtrlV2 : MonoBehaviour
{

    public event System.Action<Vector3> moveUnitsConvergeOnPoint;
    public event System.Action<Vector3, Vector3> moveUnitsKeepFormation;

    static UnitOrdersCtrlV2 _instance;
    public static UnitOrdersCtrlV2 Instance
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
        print(Grid.Instance.maxStepHeight);
        Vector3 anchor = UnitSelectionCtrlV2.Instance.GetAnchorPosition();
        if (UnitSelectionCtrl.Instance.GetBoundingBoxDiagonal() > UnitSelectionCtrlV2.Instance.SelectedUnitCount || UnitSelectionCtrlV2.Instance.BoundingBoxContainsPoint(destination))
        {
            print("converge on point");
            if (moveUnitsConvergeOnPoint != null)
                moveUnitsConvergeOnPoint(destination);
        }
        else
        {
            print("keep formation");
            if (moveUnitsKeepFormation != null)
                moveUnitsKeepFormation(destination, anchor);
        }
    }
}

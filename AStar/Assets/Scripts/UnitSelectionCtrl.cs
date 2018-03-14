using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class UnitSelectionCtrl : MonoBehaviour {

    public List<UnitControllerV2> selectedUnits = new List<UnitControllerV2>();
    HashSet<UnitControllerV2> selectionPool = new HashSet<UnitControllerV2>();
    bool isSelecting = false;
    Vector3 mousePosition1;

    void Update()
    {
        if (Input.GetMouseButtonDown(0))
        {
            isSelecting = true;
            mousePosition1 = Input.mousePosition;
            selectionPool.Clear();
        }
        if (Input.GetMouseButtonUp(0))
        {
            isSelecting = false;
            if (selectionPool.Count > 0)
            {

            }
                //SelectUnits(selectionPool);

            //else if (SingleSelect())
            //    SelectUnits(selectionPool);
        }
        if (isSelecting)
        {
            GameObject[] units = GameObject.FindGameObjectsWithTag("Unit");
            for (int i = 0; i < units.Length; i++)
            {
                //if (IsWithinSelectionBounds(units[i]))
                  //  selectionPool.Add(units[i].GetComponent<UnitControllerV2>());
            }
        }
    }

    private void OnGUI()
    {
        if (isSelecting)
        {
            
        }
    }
}

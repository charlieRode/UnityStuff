using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class UnitSelectionCtrl : MonoBehaviour {

    //List<UnitController> selectedUnits = new List<UnitController>();
    //HashSet<UnitController> selectionPool = new HashSet<UnitController>();
    List<UnitCtrl> selectedUnits = new List<UnitCtrl>();
    HashSet<UnitCtrl> selectionPool = new HashSet<UnitCtrl>();

    bool isSelecting = false;
    Vector3 mousePosition1;

    public int SelectedUnitCount
    {
        get { return selectedUnits.Count; }
    }

    static UnitSelectionCtrl _instance;
    public static UnitSelectionCtrl Instance
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
            CheckSelectionBox();
        }

    }
    void SelectUnits()
    {
        selectedUnits.Clear();
        //foreach(UnitController unit in GameCtrl.Instance.activeUnitsInGame)
        foreach(UnitCtrl unit in GameCtrl.Instance.activeUnitsInGame)
        {
            if (selectionPool.Contains(unit))
            {
                unit.Select();
                selectedUnits.Add(unit);
            }
            else
            {
                unit.Deselect();
            }
        }
    }

    void CheckSelectionBox()
    {
        selectionPool.Clear();
        //foreach(UnitController unit in GameCtrl.Instance.activeUnitsInGame)
        foreach (UnitCtrl unit in GameCtrl.Instance.activeUnitsInGame)
        {
            if (IsWithinSelectionBounds(unit))
            {
                selectionPool.Add(unit);
            }
        }
        if (selectionPool.Count > 0 || SingleSelect())
            SelectUnits();
    }

    void OnGUI()
    {
        if (isSelecting)
        {
            Rect rect = Utils.GetScreenRect(mousePosition1, Input.mousePosition);
            Utils.DrawScreenRect(rect, new Color(0.8f, 0.8f, 0.95f, 0.25f));
            Utils.DrawScreenRectBorder(rect, 2, new Color(0.8f, 0.8f, 0.95f));
        }
    }

    bool IsWithinSelectionBounds(UnitController unit)
    {
        Bounds viewportBounds = Utils.GetViewportBounds(mousePosition1, Input.mousePosition);
        return viewportBounds.Contains(
            Camera.main.WorldToViewportPoint(unit.transform.position)
        );
    }

    bool IsWithinSelectionBounds(UnitCtrl unit)
    {
        Bounds viewportBounds = Utils.GetViewportBounds(mousePosition1, Input.mousePosition);
        return viewportBounds.Contains(
            Camera.main.WorldToViewportPoint(unit.transform.position)
        );
    }

    bool SingleSelect()
    {
        Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
        RaycastHit hit;

        if (Physics.Raycast(ray, out hit))
        {
            GameObject selectedUnit = hit.collider.gameObject;
            if (selectedUnit && selectedUnit.tag == "Unit")
            {
                //selectionPool.Add(selectedUnit.GetComponent<UnitController>());
                selectionPool.Add(selectedUnit.GetComponent<UnitCtrl>());
                return true;
            }
        }

        return false;
    }

    // Get the bounding box of the selected units in world space.
    float[,] GetBoundingBox()
    {
        // WorldSpace has intuitive x and z orientations.
        if (selectedUnits.Count == 0)
            return null;

        float minX, maxX, minZ, maxZ;
        minX = maxX = selectedUnits[0].transform.position.x;
        minZ = maxZ = selectedUnits[0].transform.position.z;

        for (int i = 1; i < selectedUnits.Count; i++)
        {
            if (selectedUnits[i].transform.position.x < minX)
                minX = selectedUnits[i].transform.position.x;
            else if (selectedUnits[i].transform.position.x > maxX)
                maxX = selectedUnits[i].transform.position.x;

            if (selectedUnits[i].transform.position.z < minZ)
                minZ = selectedUnits[i].transform.position.z;
            else if (selectedUnits[i].transform.position.z > maxZ)
                maxZ = selectedUnits[i].transform.position.z;
        }
        print(new float[,] { { minX, maxX }, { minZ, maxZ } });
        return new float[,] { { minX, maxX }, { minZ, maxZ } };
    }

    // Get the center of the bounding box for selected units in world space.
    public Vector3 GetAnchorPosition()
    {
        float[,] bb = GetBoundingBox();
        if (bb == null)
            return Vector3.zero;
        return new Vector3(bb[0,0] + (bb[0,1] - bb[0,0]) * 0.5f, 0f, bb[1,0] + (bb[1,1] - bb[1,0]) * 0.5f);
    }

    // Get the diagonal of the bounding box for the selected units in world space.
    public int GetBoundingBoxDiagonal()
    {
        float[,] bb = GetBoundingBox();
        if (bb == null)
            return -1;

        //int result = (int)Mathf.Sqrt(Mathf.Pow((maxX - minX), 2) + Mathf.Pow((maxZ - minZ), 2));
        return (int)Mathf.Sqrt(Mathf.Pow((bb[0, 1] - bb[0, 0]), 2) + Mathf.Pow((bb[1, 1] - bb[1, 0]), 2));
    }

    public bool BoundingBoxContainsPoint(Vector3 point)
    {
        float[,] bb = GetBoundingBox();
        if (bb == null)
            return false;

        return point.x >= bb[0, 0] && point.x <= bb[0, 1] && point.z >= bb[1, 0] && point.z <= bb[1, 1];
    }
}

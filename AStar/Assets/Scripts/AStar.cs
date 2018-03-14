using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Runtime.InteropServices;

public class AStar : MonoBehaviour {

    public enum HeuristicFormula
    {
        Manhattan = 1,
        MaxDXDY = 2,
        DiagonalShortCut = 3,
        Euclidean = 4,
        EuclideanNoSQR = 5,
        Custom1 = 6
    }

    // Allows the struct to take up fewer bytes than 16. This should be 13.
    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    internal struct AStarNode
    {
        public int F;
        public int G;
        public ushort PX; // Parent
        public ushort PY;
        public byte Status;
    }


    // FIGURE OUT HOW TO CONVERT GRIDCONSTRUCTOR INTO GRID
    byte[,] grid;
    GridConstructor gridConstructor;

    PriorityQueueB<int> openSet;

    // Thought we got rid of the closed set...
    List<AStarNode> closedSet;

    bool stop = false;
    bool stopped = true;

    HeuristicFormula HFormula = HeuristicFormula.Manhattan;
    int hEstimate = 2;

    public bool punishChangeDirection = false;
    // Something to do with punishChangeDirection
    int horiz;

    // I don't understand this.
    // public bool reopenClosedNodes;

    public bool tieBreaker = false;
    public int searchLimit = 2000;
    float completedTime;
    AStarNode[] calcGrid;
    byte openNodeValue = 1;
    byte closedNodeValue = 2;


    int H;
    int G;
    int location;
    int newLocation;
    ushort locationX;
    ushort locationY;
    ushort newLocationX;
    ushort newLocationY;
    int closedNodeCounter = 0;
    ushort gridX;
    ushort gridY;
    ushort gridXMinus1;
    ushort gridYLog2;
    bool found = false;
    sbyte[,] direction = new sbyte[8, 2] { { 0, -1 }, { 1, 0 }, { 0, 1 }, { -1, 0 }, { 1, -1 }, { 1, 1 }, { -1, 1 }, { -1, -1 } };
    int endLocation;

    void Start()
    {
        gridConstructor = GetComponent<GridConstructor>();
        gridX = (ushort)gridConstructor.gridSizeX;
        gridY = (ushort)gridConstructor.gridSizeY;
        gridXMinus1 = (ushort)(gridX - 1);
        gridYLog2 = (ushort)Mathf.Log(gridY, 2);

        if (Mathf.Log(gridX, 2) != (int)Mathf.Log(gridX, 2) || Mathf.Log(gridY, 2) != (int)Mathf.Log(gridY, 2))
            Debug.Log("Invalid Grid, size in X and Y must be power of 2");

        if (calcGrid == null || calcGrid.Length != gridX * gridY)
            calcGrid = new AStarNode[gridX * gridY];

        openSet = new PriorityQueueB<int>(new ComparePFNodeMatrix(calcGrid));
    }

    public void FindPathStop()
    {
        stop = true;
    }

    List<AStarNode> FindPath(Vector3 startPosition, Vector3 endPosition)
    {
        Node start = gridConstructor.NodeFromWorldPoint(startPosition);
        Node end = gridConstructor.NodeFromWorldPoint(endPosition);

        found = false;
        stop = false;
        stopped = false;
        closedNodeCounter = 0;
        openNodeValue += 2;
        closedNodeValue += 2;

        openSet.Clear();
        closedSet.Clear();

        location = (start.gridY << gridYLog2) + start.gridX;
        endLocation = (end.gridY << gridYLog2) + start.gridX;
        calcGrid[location].G = 0;
        calcGrid[location].F = hEstimate;
        calcGrid[location].PX = (ushort)start.gridX;
        calcGrid[location].PY = (ushort)start.gridY;
        calcGrid[location].Status = openNodeValue;

        openSet.Push(location);
        while(openSet.Count > 0 && !stop)
        {
            location = openSet.Pop();

            if (calcGrid[location].Status == closedNodeValue)
                continue;

            locationX = (ushort)(location & gridXMinus1);
            locationY = (ushort)(location >> gridYLog2);

            if (location == endLocation)
            {
                calcGrid[location].Status = openNodeValue;
                found = true;
                break;
            }

            if (closedNodeCounter > searchLimit)
            {
                stopped = true;
                return null;
            }

            if (punishChangeDirection)
                horiz = (locationX - calcGrid[location].PX);

            for (int i = 0; i < 8; i++)
            {
                newLocationX = (ushort)(locationX + direction[i, 0]);
                newLocationY = (ushort)(locationY + direction[i, 1]);
                newLocation = (newLocationY << gridYLog2) + newLocationX;

            }
        }

        return null;

    }










    internal class ComparePFNodeMatrix : IComparer<int>
    {
        #region Variables Declaration
        AStarNode[] mMatrix;
        #endregion

        #region Constructors
        public ComparePFNodeMatrix(AStarNode[] matrix)
        {
            mMatrix = matrix;
        }
        #endregion

        #region IComparer Members
        public int Compare(int a, int b)
        {
            if (mMatrix[a].F > mMatrix[b].F)
                return 1;
            else if (mMatrix[a].F < mMatrix[b].F)
                return -1;
            return 0;
        }
        #endregion
    }
}

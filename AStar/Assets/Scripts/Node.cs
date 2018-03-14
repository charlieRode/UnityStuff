using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Node : IHeapItem<Node>
{
    public bool walkable;
    public int gridX;
    public int gridY;
    public int movementPenalty;
    public Vector3 worldPosition;
    public int HeapIndex { get; set; }
    public Node parent;

    public Node(bool _walkable, Vector3 _worldPos, int x, int y, int _movementPenalty)
    {
        walkable = _walkable;
        worldPosition = _worldPos;
        gridX = x;
        gridY = y;
        movementPenalty = _movementPenalty;
    }

    public int F
    {
        get { return G + H; }
    }

    public int G;
    public int H;

    public int CompareTo(Node other)
    {
        if (other == null || F < other.F)
        {
            return 1;
        }

        else if (F > other.F)
        {
            return -1;
        }

        // F cost tie
        else
        {
            if (H < other.H)
            {
                return 1;
            }

            else if (H > other.H)
            {
                return -1;
            }
        }

        return 0;
    }

    public override string ToString()
    {
        return F.ToString();
    }
}

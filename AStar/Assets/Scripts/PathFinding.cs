using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Diagnostics;

public class PathFinding : MonoBehaviour {

    PathRequestManager requestManager;
    GridConstructor grid;

    void Awake()
    {
        grid = GetComponent<GridConstructor>();
        requestManager = GetComponent<PathRequestManager>();
    }

    public void StartFindPath(Vector3 startPoint, Vector3 endPoint)
    {
        StartCoroutine(FindPath(startPoint, endPoint));
    }

    IEnumerator FindPath(Vector3 startPosition, Vector3 targetPosition)
    {
        Stopwatch sw = new Stopwatch();
        sw.Start();

        Vector3[] waypoints = new Vector3[0];
        bool pathSuccess = false;

        Node startNode = grid.NodeFromWorldPoint(startPosition);
        Node targetNode = grid.NodeFromWorldPoint(targetPosition);

        if (startNode.walkable && targetNode.walkable)
        {
            BinaryHeap<Node> openSet = new BinaryHeap<Node>(grid.MaxSize);
            HashSet<Node> closedSet = new HashSet<Node>();

            openSet.Add(startNode);
            while (openSet.Count > 0)
            {
                Node current = openSet.Pop();
                closedSet.Add(current);
                if (current == targetNode)
                {
                    pathSuccess = true;
                    sw.Stop();
                    break;
                }

                foreach (Node neighbor in grid.GetNeighbors(current))
                {
                    if (!neighbor.walkable || closedSet.Contains(neighbor))
                        continue;

                    int newMoveCostToNeighbor = current.G + GetDistance(current, neighbor) + neighbor.movementPenalty;
                    if (!openSet.Contains(neighbor) || newMoveCostToNeighbor < neighbor.G)
                    {
                        neighbor.G = newMoveCostToNeighbor;
                        neighbor.H = GetDistance(neighbor, targetNode);
                        neighbor.parent = current;
                        if (!openSet.Contains(neighbor))
                            openSet.Add(neighbor);
                        else
                            openSet.UpdateItem(neighbor);
                    }
                }
            }
        }

        yield return null;
        if (pathSuccess)
        {
            waypoints = RetracePath(startNode, targetNode);
        }
        requestManager.FinishedProcessingPath(waypoints, pathSuccess);
        print("path found: " + sw.ElapsedMilliseconds);
    }

    // Does not account for height differences
    /*
    Vector3[] SimplifyPath(List<Node> path)
    {
        List<Vector3> waypoints = new List<Vector3>();
        Vector2 directionOld = Vector2.zero;

        for (int i = 1; i < path.Count - 1; i++)
        {
            Vector2 directionNew = new Vector2(path[i].gridX - path[i - 1].gridX, path[i].gridY - path[i - 1].gridY);
            if (directionNew != directionOld)
            {
                waypoints.Add(path[i].worldPosition);
            }
            directionOld = directionNew;
        }

        waypoints.Add(path[path.Count - 1].worldPosition);
        waypoints.Reverse();
        return waypoints.ToArray();
    }
    */

    // Accounts for height differences.
    Vector3[] SimplifyPath(List<Node> path)
    {
        List<Vector3> waypoints = new List<Vector3>();
        //Vector2 directionOld = Vector2.zero;
        Vector3 directionOld = Vector3.zero;

        for (int i = 1; i < path.Count - 1; i++)
        {
            //Vector2 directionNew = new Vector2(path[i].gridX - path[i - 1].gridX, path[i].gridY - path[i - 1].gridY);
            Vector3 directionNew = new Vector3(path[i].gridX - path[i - 1].gridX, path[i].worldPosition.y - path[i-1].worldPosition.y, path[i].gridY - path[i - 1].gridY);
            if (directionNew != directionOld)
            {
                waypoints.Add(path[i-1].worldPosition);
            }
            directionOld = directionNew;
        }

        waypoints.Add(path[path.Count - 1].worldPosition);
        waypoints.Reverse();
        return waypoints.ToArray();
    }

    Vector3[] RetracePath(Node startNode, Node endNode)
    {
        List<Node> path = new List<Node>();
        Node current = endNode;

        while (current != startNode)
        {
            path.Add(current);
            current = current.parent;
        }
        Vector3[] waypoints = SimplifyPath(path);
        return waypoints;
    }

    int GetDistance(Node nodeA, Node nodeB)
    {
        int distX = Mathf.Abs(nodeA.gridX - nodeB.gridX);
        int distY = Mathf.Abs(nodeA.gridY - nodeB.gridY);
        
        if (distX < distY)
        {
            return distX * 14 + (distY - distX) * 10;
        }
        return distY * 14 + (distX - distY) * 10;
    }
}

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Diagnostics;

public class PathFindingV2 : MonoBehaviour
{
    // For path smoothing. Will need to get these from the units eventually.
    public LayerMask groundLayer;
    public float maxStepHeight;
    public float sphereColliderBuffer;
    public float epsilon;

    static PathFindingV2 _instance;
    public static PathFindingV2 Instance
    {
        get
        {
            return _instance;
        }
    }

    public enum SmoothOptions
    {
        Simplify, Smooth, SimplifyThenSmooth
    }
    public SmoothOptions pathSmoothing = SmoothOptions.Simplify;

    public bool useSmoothPath;
    public LayerMask obsticleLayer;

//------------------------------EVERYTHING BEFORE HERE NEEDS TO BE RE-EVALUATED BEFORE CONFIRMING IT BELONGS IN THIS CLASS.

    public enum HeuristicFormula
    {
        Manhattan, MaxDXDY, DiagonalShortCut, Euclidean, EuclideanNoSQR, Custom1, Custom2
    }
    public HeuristicFormula hFormula = HeuristicFormula.Manhattan;
    public int hEstimate = 1;

    public int lightTurnPenalty;
    public int heavyTurnPenalty;

    PathRequestManager requestManager;
    Grid grid;
    Vector2 previousDirection = Vector2.zero;

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
        grid = GetComponent<Grid>();
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

        MapNode startNode = grid.NodeFromWorldPoint(startPosition);
        MapNode targetNode = grid.NodeFromWorldPoint(targetPosition);

        if (startNode.walkable && targetNode.walkable)
        {
            BinaryHeap<MapNode> openSet = new BinaryHeap<MapNode>(grid.MaxSize);
            HashSet<MapNode> closedSet = new HashSet<MapNode>();

            openSet.Add(startNode);
            while (openSet.Count > 0)
            {
                MapNode current = openSet.Pop();
                if (current.parent != null)
                    previousDirection = new Vector2(current.gridX - current.parent.gridX, current.gridY - current.parent.gridY);

                closedSet.Add(current);
                if (current == targetNode)
                {
                    pathSuccess = true;
                    sw.Stop();
                    break;
                }

                foreach (MapNode neighbor in grid.GetNeighbors(current))
                {
                    if (!neighbor.walkable || closedSet.Contains(neighbor))
                        continue;

                    Vector2 newDirection = new Vector2(neighbor.gridX - current.gridX, neighbor.gridY - current.gridY);

                    int directionChangePenalty;
                    if (newDirection == previousDirection)
                        directionChangePenalty = 0;

                    else if (newDirection.x == previousDirection.x || newDirection.y == previousDirection.y)
                        directionChangePenalty = lightTurnPenalty;

                    else
                        directionChangePenalty = heavyTurnPenalty;

                    int newMoveCostToNeighbor = current.G + GetDistance(current, neighbor) + neighbor.movementPenalty + directionChangePenalty;
                    if (!openSet.Contains(neighbor) || newMoveCostToNeighbor < neighbor.G)
                    {
                        neighbor.G = newMoveCostToNeighbor;
                        neighbor.H = GetHeuristicValue(neighbor, targetNode);
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
        //print("path found: " + sw.ElapsedMilliseconds);
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

    Vector3[] SmoothPath(List<MapNode> path)
    {
        path.Reverse();
        List<Vector3> smoothedPath = new List<Vector3>();
        smoothedPath.Add(path[0].worldPosition);

        float dyOld = 0f;
        for (int i = 1; i < path.Count; i++)
        {
            float dyNew = path[i].worldPosition.y - path[i - 1].worldPosition.y;
            if (dyNew - dyOld >= epsilon || !Walkable(smoothedPath[smoothedPath.Count - 1], path[i].worldPosition, 0.25f))
            {
                smoothedPath.Add(path[i - 1].worldPosition);
            }
            dyOld = dyNew;
        }

        smoothedPath.Add(path[path.Count - 1].worldPosition);
        return smoothedPath.ToArray();
    }

    bool Walkable(Vector3 A, Vector3 B, float checkRadius)
    {
        RaycastHit rHit1, rHit2;
        Vector3 step = (B - A).normalized * checkRadius * 2f; // multiply by 2 to check every point instead of every half point to improve performance.
        Vector3 checkPoint = A + Vector3.up * (checkRadius + sphereColliderBuffer);
        while (Mathf.Abs(checkPoint.x - B.x) > checkRadius || Mathf.Abs(checkPoint.z - B.z) > checkRadius)
        {
            // First check that at each point, the delta between the point on the ground and the starting point on the ground is at most the maxStepHeight
            // Expanding to two check point on either edge of the check sphere. This will account for the unit's width when raycasting down.

            // The two points on the left and right (local) poles of the check sphere are given by the calculations below.
            Vector3 checkPoint1 = checkPoint + new Vector3(step.z, step.y, -step.x).normalized * checkRadius;
            Vector3 checkPoint2 = checkPoint + new Vector3(-step.z, step.y, step.x).normalized * checkRadius;
            if (Physics.Raycast(checkPoint1, Vector3.down, out rHit1, 100, groundLayer) && Physics.Raycast(checkPoint2, Vector3.down, out rHit2, 100, groundLayer))
            {
                if (checkPoint.y - rHit1.point.y > maxStepHeight || checkPoint.y - rHit2.point.y > maxStepHeight)
                    return false;
            }

            // Next we check that the sphere of checkRadius at each checkPoint along the line does not collide with anything.
            if (Physics.CheckSphere(checkPoint, checkRadius, obsticleLayer))
                return false;

            checkPoint += step;
        }

        return true;
    }

    // Accounts for height differences.
    Vector3[] SimplifyPath(List<MapNode> path)
    {
        List<Vector3> waypoints = new List<Vector3>();
        //Vector2 directionOld = Vector2.zero;
        Vector3 directionOld = Vector3.zero;

        for (int i = 1; i < path.Count - 1; i++)
        {
            //Vector2 directionNew = new Vector2(path[i].gridX - path[i - 1].gridX, path[i].gridY - path[i - 1].gridY);
            Vector3 directionNew = new Vector3(path[i].gridX - path[i - 1].gridX, path[i].worldPosition.y - path[i - 1].worldPosition.y, path[i].gridY - path[i - 1].gridY);
            if (directionNew != directionOld)
            {
                waypoints.Add(path[i - 1].worldPosition);
            }
            directionOld = directionNew;
        }

        waypoints.Add(path[path.Count - 1].worldPosition);
        waypoints.Reverse();
        return waypoints.ToArray();
    }


    // The final version won't mash two methods together. I just want to
    // get this to work to see the effects.
    Vector3[] SimplifyThenSmooth(List<MapNode> path)
    {
        List<MapNode> waypoints = new List<MapNode>();
        //Vector2 directionOld = Vector2.zero;
        Vector3 directionOld = Vector3.zero;

        for (int i = 1; i < path.Count - 1; i++)
        {
            //Vector2 directionNew = new Vector2(path[i].gridX - path[i - 1].gridX, path[i].gridY - path[i - 1].gridY);
            Vector3 directionNew = new Vector3(path[i].gridX - path[i - 1].gridX, path[i].worldPosition.y - path[i - 1].worldPosition.y, path[i].gridY - path[i - 1].gridY);
            if (directionNew != directionOld)
            {
                waypoints.Add(path[i - 1]);
            }
            directionOld = directionNew;
        }

        waypoints.Add(path[path.Count - 1]);

        return SmoothPath(waypoints);
    }

    Vector3[] RetracePath(MapNode startNode, MapNode endNode)
    {
        List<MapNode> path = new List<MapNode>();
        MapNode current = endNode;

        while (current != startNode)
        {
            path.Add(current);
            current = current.parent;
        }
        path.Add(startNode);

        if (debugMode)
        {
            StartCoroutine(DebugSmoothPath(path));
            return new Vector3[0];
        }

        Vector3[] waypoints;
        switch (pathSmoothing)
        {
            case SmoothOptions.Simplify:
                waypoints = SimplifyPath(path);
                break;
            case SmoothOptions.Smooth:
                waypoints = SmoothPath(path);
                break;
            case SmoothOptions.SimplifyThenSmooth:
                waypoints = SimplifyThenSmooth(path);
                break;
            default:
                waypoints = new Vector3[0];
                break;
        }
        return waypoints;
    }

    int GetDistance(MapNode nodeA, MapNode nodeB)
    {
        int distX = Mathf.Abs(nodeA.gridX - nodeB.gridX);
        int distY = Mathf.Abs(nodeA.gridY - nodeB.gridY);

        if (distX < distY)
        {
            return distX * 14 + (distY - distX) * 10;
        }
        return distY * 14 + (distX - distY) * 10;
    }

    int GetHeuristicValue(MapNode current, MapNode target)
    {
        switch (hFormula)
        {
            case HeuristicFormula.Manhattan:
                return Mathf.RoundToInt((Mathf.Abs(current.gridX - target.gridX) + Mathf.Abs(current.gridY - target.gridY)) * hEstimate);

            case HeuristicFormula.MaxDXDY:
                return Mathf.RoundToInt(Mathf.Max(Mathf.Abs(current.gridX - target.gridX), Mathf.Abs(current.gridY - target.gridY)) * hEstimate);

            case HeuristicFormula.DiagonalShortCut:
                int h_diagonal = Mathf.Min(Mathf.Abs(current.gridX - target.gridX), Mathf.Abs(current.gridY - target.gridY));
                int h_straight = Mathf.Abs(current.gridX - target.gridX) + Mathf.Abs(current.gridY - target.gridY);
                return (hEstimate * 2) * h_diagonal + hEstimate * (h_straight - 2 * h_diagonal);

            case HeuristicFormula.Euclidean:
                return Mathf.RoundToInt(Mathf.Sqrt(Mathf.Pow(current.gridX - target.gridX, 2) + Mathf.Pow(current.gridY - target.gridY, 2)) * hEstimate);

            case HeuristicFormula.EuclideanNoSQR:
                return Mathf.RoundToInt(Mathf.Pow(current.gridX - target.gridX, 2) + Mathf.Pow(current.gridY - target.gridY, 2) * hEstimate);

            case HeuristicFormula.Custom1:
                Vector2 dxdy = new Vector2(Mathf.Abs(current.gridX - target.gridX), Mathf.Abs(current.gridY - target.gridY));
                int orthogonal = (int)Mathf.Abs(dxdy.x - dxdy.y);
                int diagonal = (int)(Mathf.Abs(dxdy.x + dxdy.y - orthogonal) / 2f);
                return Mathf.RoundToInt((diagonal + orthogonal + dxdy.x + dxdy.y) * hEstimate);

            case HeuristicFormula.Custom2:
                return hEstimate * GetDistance(current, target);

            default:
                return 0;
        }

    }


    // DEBUG
    public bool debugMode;
    public float debugSpeed;
    bool _valid;
    List<Vector3> validWalkPoints = new List<Vector3>();
    List<Vector3> invalidWalkPoints = new List<Vector3>();
    List<Vector3> startingPath = new List<Vector3>();
    List<Vector3> _smoothPath = new List<Vector3>();

    IEnumerator DebugSmoothPath(List<MapNode> path)
    {
        path.Reverse();

        startingPath = new List<Vector3>();
        foreach (MapNode node in path)
            startingPath.Add(node.worldPosition);

        List<Vector3> smoothedPath = new List<Vector3>();
        //smoothedPath.Add(path[0].worldPosition);
        _smoothPath.Add(path[0].worldPosition);

        print("path length: " + path.Count);

        float dyOld = 0f;
        for (int i = 1; i < path.Count; i++)
        {
            print("i: " + i);
            float dyNew = path[i].worldPosition.y - path[i-1].worldPosition.y;
            yield return StartCoroutine(DebugWalkable(_smoothPath[_smoothPath.Count - 1], path[i].worldPosition, 0.25f));
            if (dyNew - dyOld >= epsilon || !_valid)
            {
                print("valid: " + _valid);
                _smoothPath.Add(path[i - 1].worldPosition);
                //dyOld = dyNew;
            }
            dyOld = dyNew;
        }

        yield return null;
        
    }

    IEnumerator DebugWalkable(Vector3 A, Vector3 B, float checkRadius)
    {
        validWalkPoints.Clear();
        invalidWalkPoints.Clear();
        RaycastHit rHit1, rHit2;
        Vector3 step = (B - A).normalized * checkRadius * 2f;
        Vector3 checkPoint = A + Vector3.up * (checkRadius + sphereColliderBuffer);
        _valid = true;
        bool enteredWhileLoop = false;

        while (Mathf.Abs(checkPoint.x - B.x) > checkRadius || Mathf.Abs(checkPoint.z - B.z) > checkRadius)
        {
            enteredWhileLoop = true;
            // First check that at each point, the delta between the point on the ground and the starting point on the ground is at most the maxStepHeight
            // Expanding to two check point on either edge of the check sphere. This will account for the unit's width when raycasting down.

            // The two points on the left and right (local) poles of the check sphere are given by the calculations below.
            Vector3 checkPoint1 = checkPoint + new Vector3(step.z, step.y, -step.x).normalized * checkRadius;
            Vector3 checkPoint2 = checkPoint + new Vector3(-step.z, step.y, step.x).normalized * checkRadius;
            //UnityEngine.Debug.DrawLine(checkPoint1, checkPoint2, Color.red);

            if (Physics.Raycast(checkPoint1, Vector3.down, out rHit1, 100, groundLayer) && Physics.Raycast(checkPoint2, Vector3.down, out rHit2, 100, groundLayer))
            {
                // Should I be comparing checkPoint1.y, checkPoint2.y?
                if (checkPoint.y - rHit1.point.y > maxStepHeight || checkPoint.y - rHit2.point.y > maxStepHeight)
                {
                    _valid = false;
                }
            }     

            // Next we check that the sphere of checkRadius at each checkPoint along the line does not collide with anything.
            if (Physics.CheckSphere(checkPoint, checkRadius, obsticleLayer))
                _valid = false;

            if (_valid)
            {
                validWalkPoints.Add(checkPoint);
            }
            else
            {
                invalidWalkPoints.Add(checkPoint);
                //break;
            }

            checkPoint += step;
            yield return new WaitForSeconds(debugSpeed);
        }
        print("enteredWhileLoop: " + enteredWhileLoop);
        if (!enteredWhileLoop)
        {
            print("checkPoint == A + Vector3.up * (checkRadius + sphereColliderBuffer): " + (checkPoint == (A + Vector3.up * (checkRadius + sphereColliderBuffer))));
            print("checkPoint: " + checkPoint);
            print("B: " + B);
        }


    }

    public Color validPointColor;
    public Color invalidPointColor;
    public Color smoothPathColor;
    private void OnDrawGizmos()
    {
        if (debugMode)
        {
            Gizmos.color = validPointColor;
            foreach (Vector3 point in validWalkPoints)
            {
                Gizmos.DrawSphere(point, 0.25f);
            }
            Gizmos.color = invalidPointColor;
            foreach (Vector3 point in invalidWalkPoints)
            {
                Gizmos.DrawSphere(point, 0.25f);
            }
            Gizmos.color = Color.gray;
            foreach (Vector3 point in startingPath)
            {
                Gizmos.DrawCube(point, Vector3.one / 4f);
            }
            Gizmos.color = smoothPathColor;
            foreach (Vector3 point in _smoothPath)
            {
                Gizmos.DrawCube(point, Vector3.one / 2f);
            }
        }
    }

}

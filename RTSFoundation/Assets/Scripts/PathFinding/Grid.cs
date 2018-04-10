using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Grid : MonoBehaviour
{

    public bool displayGridGizmos;
    public bool blurWeights;
    public int blurIntensity;
    public int obsticleProximityPenalty = 10;

    public LayerMask unwalkable;
    public LayerMask groundLayer;
    //public LayerMask walkable;
    public LayerMask obsticleLayer;

    public Vector2 gridWorldSize;
    public float nodeRadius;

    public float gridProjectorHeight;

    public TerrainType[] walkableRegions;
    Dictionary<int, int> walkableRegionsDict = new Dictionary<int, int>();

    MapNode[,] grid;

    public int penaltyMax;
    public int penaltyMin;

    float nodeDiameter;
    public int gridSizeX;
    public int gridSizeY;
    public int MaxSize
    {
        get { return gridSizeX * gridSizeY; }
    }

    static Grid _instance;
    public static Grid Instance
    {
        get
        {
            return _instance;
        }
    }

    // public float maxStepHeight;
    public float maxInclineAngle;
    public float maxStepHeight
    {
        get
        {
            return nodeDiameter * Mathf.Tan(maxInclineAngle * Mathf.Deg2Rad);
        }
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
        nodeDiameter = nodeRadius * 2;
        gridSizeX = Mathf.RoundToInt(gridWorldSize.x / nodeDiameter);
        gridSizeY = Mathf.RoundToInt(gridWorldSize.y / nodeDiameter);

        foreach (TerrainType region in walkableRegions)
        {
            walkableRegionsDict.Add((int)Mathf.Log(region.terrainMask.value, 2), region.terrainPenalty);
        }

        CreateGrid();
    }

    public List<MapNode> GetNeighbors(MapNode node)
    {
        List<MapNode> neighbors = new List<MapNode>();
        for (int i = -1; i <= 1; i++)
        {
            for (int j = -1; j <= 1; j++)
            {
                if (i == 0 && j == 0)
                    continue;

                if (node.gridX + i > 0 && node.gridX + i < gridSizeX && node.gridY + j > 0 && node.gridY + j < gridSizeY)
                {
                    if (Mathf.Abs(node.worldPosition.y - grid[node.gridX + i, node.gridY + j].worldPosition.y) <= maxStepHeight) // Check step height
                    {
                        neighbors.Add(grid[node.gridX + i, node.gridY + j]);
                    }
                }
            }
        }

        return neighbors;
    }

    void CreateGrid()
    {
        grid = new MapNode[gridSizeX, gridSizeY];
        Vector3 worldBottomLeft = new Vector3(transform.position.x - gridWorldSize.x / 2, gridProjectorHeight, transform.position.z - gridWorldSize.y / 2);

        for (int x = 0; x < gridSizeX; x++)
        {
            for (int y = 0; y < gridSizeY; y++)
            {
                Vector3 projectorPoint = worldBottomLeft + Vector3.right * (x * nodeDiameter + nodeRadius) + Vector3.forward * (y * nodeDiameter + nodeRadius);

                Ray ray = new Ray(projectorPoint, Vector3.down);
                RaycastHit hit;
                int nodeWeight = 0;
                if (Physics.Raycast(ray, out hit, gridProjectorHeight * 2, groundLayer))
                {
                    bool walkable = !Physics.CheckSphere(hit.point + hit.normal * (nodeRadius + 0.005f), nodeRadius - 0.2f, obsticleLayer);
                    walkableRegionsDict.TryGetValue(hit.collider.gameObject.layer, out nodeWeight);
                    if (!walkable)
                        nodeWeight += obsticleProximityPenalty;

                    grid[x, y] = new MapNode(walkable, hit.point, x, y, nodeWeight);
                }
                else
                {
                    grid[x, y] = new MapNode(false, Vector3.zero, -1, -1, nodeWeight);
                }
            }
        }

        if (blurWeights)
            BlurPenaltyMap(blurIntensity);
    }

    void BlurPenaltyMap(int blurSize)
    {
        int kernelSize = blurSize * 2 + 1;
        int kernelExtents = (kernelSize - 1) / 2;

        int[,] penaltiesHorizontalPass = new int[gridSizeX, gridSizeY];
        int[,] penaltiesVerticalPass = new int[gridSizeX, gridSizeY];

        for (int y = 0; y < gridSizeY; y++)
        {
            for (int x = -kernelExtents; x <= kernelExtents; x++)
            {
                int sampleX = Mathf.Clamp(x, 0, kernelExtents);
                penaltiesHorizontalPass[0, y] += grid[sampleX, y].movementPenalty;
            }

            for (int x = 1; x < gridSizeX; x++)
            {
                int removeIndex = Mathf.Clamp(x - kernelExtents - 1, 0, gridSizeX);
                int addIndex = Mathf.Clamp(x + kernelExtents, 0, gridSizeX - 1);

                penaltiesHorizontalPass[x, y] = penaltiesHorizontalPass[x - 1, y] - grid[removeIndex, y].movementPenalty + grid[addIndex, y].movementPenalty;
            }
        }

        for (int x = 0; x < gridSizeX; x++)
        {
            for (int y = -kernelExtents; y <= kernelExtents; y++)
            {
                int sampleY = Mathf.Clamp(y, 0, kernelExtents);
                penaltiesVerticalPass[x, 0] += penaltiesHorizontalPass[x, sampleY];
            }

            int blurredPenalty = Mathf.RoundToInt((float)penaltiesVerticalPass[x, 0] / (kernelSize * kernelSize));
            grid[x, 0].movementPenalty = blurredPenalty;

            for (int y = 1; y < gridSizeY; y++)
            {
                int removeIndex = Mathf.Clamp(y - kernelExtents - 1, 0, gridSizeY);
                int addIndex = Mathf.Clamp(y + kernelExtents, 0, gridSizeY - 1);

                penaltiesVerticalPass[x, y] = penaltiesVerticalPass[x, y - 1] - penaltiesHorizontalPass[x, removeIndex] + penaltiesHorizontalPass[x, addIndex];
                blurredPenalty = Mathf.RoundToInt((float)penaltiesVerticalPass[x, y] / (kernelSize * kernelSize));
                grid[x, y].movementPenalty = blurredPenalty;

                if (blurredPenalty > penaltyMax)
                {
                    penaltyMax = blurredPenalty;
                }
                if (blurredPenalty < penaltyMin)
                {
                    penaltyMin = blurredPenalty;
                }
            }
        }

    }

    public MapNode NodeFromWorldPoint(Vector3 worldPoint)
    {
        float pctX = Mathf.Clamp01((gridWorldSize.x / 2 + worldPoint.x) / gridWorldSize.x);
        float pctY = Mathf.Clamp01((gridWorldSize.y / 2 + worldPoint.z) / gridWorldSize.y);
        int xIdx = (int)Mathf.RoundToInt(pctX * (gridSizeX - 1));
        int yIdx = (int)Mathf.RoundToInt(pctY * (gridSizeY - 1));

        return grid[xIdx, yIdx];
    }

    void OnDrawGizmos()
    {
        Gizmos.DrawWireCube(transform.position, new Vector3(gridWorldSize.x, 1, gridWorldSize.y));

        if (grid != null && displayGridGizmos)
        {
            foreach (MapNode n in grid)
            {
                if (n != null)
                {
                    Gizmos.color = Color.Lerp(Color.white, Color.black, Mathf.InverseLerp(0, 10, n.movementPenalty));

                    if (!n.walkable)
                        Gizmos.color = Color.red;

                    Gizmos.DrawCube(n.worldPosition, Vector3.one * (nodeDiameter - 0.1f));
                    //Gizmos.DrawCube(n.worldPosition, Vector3.one * (nodeDiameter));

                }
            }
        }
    }

    [System.Serializable]
    public class TerrainType
    {
        public LayerMask terrainMask;
        public int terrainPenalty;
    }

}

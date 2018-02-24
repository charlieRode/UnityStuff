using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

public class MapGenerator : MonoBehaviour {

    public Map[] maps;
    public int mapIndex;

    public Transform tilePrefab;
    public Transform obsticlePrefab;
    public Transform navmeshFloor;
    public Transform floorBackground;
    public Transform navmeshMaskPrefab;
    public Vector2 maxMapSize;

    Map currentMap;

    List<Vector3> visitedTiles = new List<Vector3>();

    public float navmeshMaskBuffer = 1f;
    public float tileSize = 1;
    [Range(0, 1)]
    public float outLinePercent;

    List<Coord> allTileCoords;
    Queue<Coord> shuffledTileCoords;
    Queue<Coord> shuffledOpenTileCoords;
    Transform[,] tileMap;

    bool[,] obsticleMap;

    void Awake()
    {
        FindObjectOfType<Spawner>().OnNextWave += OnNewWave;
    }

    public Coord GetRandomCoord()
    {
        Coord coord = shuffledTileCoords.Dequeue();
        shuffledTileCoords.Enqueue(coord);

        return coord;
    }

    public Transform GetRandomOpenTile()
    {
        Coord coord = shuffledOpenTileCoords.Dequeue();
        shuffledOpenTileCoords.Enqueue(coord);

        return tileMap[coord.x, coord.y];
    }

    void OnNewWave(int waveNumber)
    {
        if (mapIndex < maps.Length)
            mapIndex = waveNumber - 1;
        GenerateMap();
    }

    public void GenerateMap()
    {
        currentMap = maps[mapIndex];
        currentMap.foregroundColor = ColorManager.instance.RandomColor();
        currentMap.backgroundColor = ColorManager.instance.RandomColor();
        while (currentMap.backgroundColor == currentMap.foregroundColor)
        {
            currentMap.backgroundColor = ColorManager.instance.RandomColor();
        }
        if (!currentMap.isStatic)
        {
            currentMap.seed = MathUtils.RandomSeed();
        }
        System.Random randGen = new System.Random(currentMap.seed);
        tileMap = new Transform[currentMap.mapSize.x, currentMap.mapSize.y];

        // Generating Coords
        allTileCoords = new List<Coord>();
        for (int i = 0; i < currentMap.mapSize.x; i++)
        {
            for (int j = 0; j < currentMap.mapSize.y; j++)
            {
                allTileCoords.Add(new Coord(i, j));
            }
        }
        shuffledTileCoords = new Queue<Coord>(MathUtils.FisherYates(allTileCoords.ToArray(), currentMap.seed));

        // Create mapHolder GameObject
        string holderName = "Generated Map";
        if (transform.Find(holderName))
        {
            DestroyImmediate(transform.Find(holderName).gameObject);
        }
        Transform mapHolder = new GameObject(holderName).transform;
        mapHolder.parent = transform;

        // Spawning Tiles
        for (int i = 0; i < currentMap.mapSize.x; i++)
        {
            for (int j = 0; j < currentMap.mapSize.y; j++)
            {
                Vector3 tilePosition = CoordToPosition(i, j);
                Transform newTile = Instantiate(tilePrefab, tilePosition, Quaternion.Euler(Vector3.right * 90)) as Transform;
                newTile.localScale = Vector3.one * tileSize * (1 - outLinePercent);
                newTile.parent = mapHolder;
                tileMap[i, j] = newTile;
            }
        }

        // Spawning Obsticles
        bool[,] obsticleMap = new bool[currentMap.mapSize.x, currentMap.mapSize.y];
        int obsticleCount = (int)(currentMap.obsticlePercent * currentMap.mapSize.x * currentMap.mapSize.y);
        int currentObsticleCount = 0;

        List<Coord> allOpenCoords = new List<Coord>(allTileCoords);

        for (int i = 0; i < obsticleCount; i++)
        {
            Coord randomCoord = GetRandomCoord();
            allOpenCoords.Remove(randomCoord);
            obsticleMap[randomCoord.x, randomCoord.y] = true;
            currentObsticleCount++;
            if (randomCoord != currentMap.mapCenter && MapIsFullyAccessible(obsticleMap, currentObsticleCount))
            {
                Vector3 prefabScale = Vector3.one * tileSize * (1 - outLinePercent);
                float obsticleHeight = Mathf.Lerp(currentMap.minObsticleHeight, currentMap.maxObsticleHeight, (float)randGen.NextDouble());
                Vector3 obsticlePosition = CoordToPosition(randomCoord.x, randomCoord.y);
                Transform newObsticle = Instantiate(obsticlePrefab, obsticlePosition + (Vector3.up * obsticleHeight / 2), Quaternion.identity) as Transform;
                newObsticle.localScale = new Vector3(prefabScale.x, obsticleHeight, prefabScale.z);
                newObsticle.parent = mapHolder;

                Renderer obsticleRenderer = newObsticle.GetComponent<Renderer>();
                Material obsticleMaterial = new Material(obsticleRenderer.sharedMaterial);
                float colorPct = (float)randomCoord.y / currentMap.mapSize.y;
                obsticleMaterial.color = Color.Lerp(currentMap.foregroundColor, currentMap.backgroundColor, colorPct);
                obsticleRenderer.sharedMaterial = obsticleMaterial;
            }
            else
            {
                obsticleMap[randomCoord.x, randomCoord.y] = false;
                currentObsticleCount--;
            }
        }

        shuffledOpenTileCoords = new Queue<Coord>(MathUtils.FisherYates(allOpenCoords.ToArray(), currentMap.seed));

        navmeshFloor.localScale = new Vector3(maxMapSize.x, maxMapSize.y, 1) * tileSize;
        floorBackground.localScale = new Vector3(currentMap.mapSize.x, currentMap.mapSize.y, 1) * tileSize;

        // Creating Navmesh Mask
        Transform leftMask = Instantiate(navmeshMaskPrefab, Vector3.left * (((currentMap.mapSize.x + maxMapSize.x) / 4f * tileSize) + navmeshMaskBuffer), Quaternion.identity) as Transform;
        leftMask.parent = mapHolder;
        leftMask.localScale = new Vector3((maxMapSize.x - currentMap.mapSize.x) / 2f, 1, currentMap.mapSize.y) * tileSize;

        Transform rightMask = Instantiate(navmeshMaskPrefab, Vector3.right * (((currentMap.mapSize.x + maxMapSize.x) / 4f * tileSize) + navmeshMaskBuffer), Quaternion.identity) as Transform;
        rightMask.parent = mapHolder;
        rightMask.localScale = new Vector3((maxMapSize.x - currentMap.mapSize.x) / 2f, 1, currentMap.mapSize.y) * tileSize;

        Transform topMask = Instantiate(navmeshMaskPrefab, Vector3.forward * (((currentMap.mapSize.y + maxMapSize.y) / 4f * tileSize) + navmeshMaskBuffer), Quaternion.identity) as Transform;
        topMask.parent = mapHolder;
        topMask.localScale = new Vector3(maxMapSize.x, 1, (maxMapSize.y - currentMap.mapSize.y) / 2f) * tileSize;

        Transform bottomMask = Instantiate(navmeshMaskPrefab, Vector3.back * (((currentMap.mapSize.y + maxMapSize.y) / 4f * tileSize) + navmeshMaskBuffer), Quaternion.identity) as Transform;
        bottomMask.parent = mapHolder;
        bottomMask.localScale = new Vector3(maxMapSize.x, 1, (maxMapSize.y - currentMap.mapSize.y) / 2f) * tileSize;
    }

    bool MapIsFullyAccessible(bool[,] obsticleMap, int currentObsticleCount)
    {
        int nonObsticleTileCount = 0;
        Queue<Coord> coordinates = new Queue<Coord>();
        bool[,] mapFlags = new bool[obsticleMap.GetLength(0), obsticleMap.GetLength(1)];
        mapFlags[currentMap.mapCenter.x, currentMap.mapCenter.y] = true;
        coordinates.Enqueue(currentMap.mapCenter);
        while (coordinates.Count > 0)
        {
            Coord current = coordinates.Dequeue(); 
            nonObsticleTileCount++;
            List<Coord> neighbors = GetAdjacentCoords(current, obsticleMap);
            foreach (Coord n in neighbors)
            {
                if (!mapFlags[n.x, n.y])
                {
                    mapFlags[n.x, n.y] = true;
                    coordinates.Enqueue(n);
                }
            }
        }
        return nonObsticleTileCount == (currentMap.mapSize.x * currentMap.mapSize.y - currentObsticleCount);
    }

    List<Coord> GetAdjacentCoords(Coord current, bool[,] obsticleMap)
    {
        List<Coord> neighbors = new List<Coord>();
        for (int a = -1; a <= 1; a++)
        {
            for (int b = -1; b <= 1; b++)
            {
                // if the coordinate falls outside of the map bounds
                if ((current.x + a < 0 || current.x + a >= obsticleMap.GetLength(0)) || (current.y + b < 0 || current.y + b >= obsticleMap.GetLength(1)))
                    continue;

                // if the coordinate is a center or diagonal coordinate
                if ((a == 0 && b == 0) || (a != 0 && b != 0))
                    continue;

                // if the adjacent coordinate is not an obsticle
                if (!obsticleMap[current.x + a, current.y + b])
                    neighbors.Add(new Coord(current.x + a, current.y + b));
            }
        }

        return neighbors;
    }

    public Vector3 GetCenterTilePosition()
    {
        return CoordToPosition(currentMap.mapCenter);
    }

    Vector3 CoordToPosition(int x, int y)
    {
        return new Vector3(-currentMap.mapSize.x / 2f + 0.5f + x, 0, -currentMap.mapSize.y / 2f + 0.5f + y) * tileSize;
    }

    Vector3 CoordToPosition(Coord c)
    {
        return CoordToPosition(c.x, c.y);
    }

    public Transform GetTileFromPosition(Vector3 position)
    {
        int x = Mathf.RoundToInt(position.x / tileSize + (currentMap.mapSize.x - 1f) / 2f);
        int y = Mathf.RoundToInt(position.z / tileSize + (currentMap.mapSize.y - 1f) / 2f);
        x = Mathf.Clamp(x, 0, tileMap.GetLength(0) - 1);
        y = Mathf.Clamp(y, 0, tileMap.GetLength(1) - 1);

        return tileMap[x, y];
    }

    [System.Serializable]
    public struct Coord
    {
        public int x;
        public int y;

        public Coord(int _x, int _y)
        {
            x = _x;
            y = _y;
        }

        public static bool operator == (Coord a, Coord b)
        {
            return (a.x == b.x && a.y == b.y);
        }
        public static bool operator != (Coord a, Coord b)
        {
            return !(a == b);
        }

        public override string ToString()
        {
            return "(" + x + ", " + y + ")";
        }
    }

    [System.Serializable]
    public class Map
    {
        public bool isStatic;
        public Coord mapSize;
        [Range(0, 1)]
        public float obsticlePercent;
        public int seed;
        public float minObsticleHeight;
        public float maxObsticleHeight;
        public Color foregroundColor;
        public Color backgroundColor;

        public Coord mapCenter
        {
            get
            {
                return new Coord(mapSize.x / 2, mapSize.y / 2);
            }
        }
    }
}

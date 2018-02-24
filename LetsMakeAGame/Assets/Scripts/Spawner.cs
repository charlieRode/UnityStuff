using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;
using System;

public class Spawner : MonoBehaviour {

    public bool devMode;
    public Wave[] waves;
    public Enemy enemy;
    public Boss bossMan;
    public Transform healthPickup;
    public Transform upgradePickup;
    public Transform upgradeAmmoPickup;
    public float timeBetweenWaves = 3f;
    Player player;
    Transform playerTransform;

    public event Action<int> OnNextWave;
    public event Action OnWinGame;

    MapGenerator map;

    Wave currentWave;
    public int currentWaveIdx;
    int currentWaveNumber
    {
        get
        {
            return currentWaveIdx + 1;
        }
    }

    int remainingEnemiesAlive;
    int remainingEnemiesToSpawn;
    float timeToNextSpawn;
    float timeInLevel = 0f;

    float timeBetweenCampingChecks = 2f;
    float timeSinceLastCampingCheck = 0f;
    float campThresholdDistance = 1.5f;
    Vector3 campPositionOld;
    bool isCamping;
    bool waveActive;
    bool isDisabled;

    bool spawnedBoss;
    bool bossIsDead;
    bool spawnedHealth;
    bool spawnedUpgrade;
    bool spawnedAmmoUpgrade;

    void Start()
    {
        player = FindObjectOfType<Player>();
        player.OnDeath += OnPlayerDeath;
        playerTransform = player.transform;
        campPositionOld = playerTransform.position;

        map = FindObjectOfType<MapGenerator>();

        currentWave = waves[currentWaveIdx];

        StartCoroutine(SpawnNextWave());
    }

    void Update()
    {
        if (isDisabled)
            return;

        if (waveActive)
            timeSinceLastCampingCheck += Time.deltaTime;
            if (timeSinceLastCampingCheck >= timeBetweenCampingChecks)
            {
                timeSinceLastCampingCheck = 0f;
                isCamping = Vector3.Distance(playerTransform.position, campPositionOld) < campThresholdDistance;
                campPositionOld = playerTransform.position;
            }

        if ((remainingEnemiesToSpawn > 0 || currentWave.infinite || currentWave.hasBoss) && timeToNextSpawn <= 0)
        {
            timeToNextSpawn = currentWave.timeBetweenSpawns;
            if (currentWave.hasBoss && !spawnedBoss)
            {
                StartCoroutine(SpawnBoss());
                spawnedBoss = true;
            }
            else if (currentWave.hasBoss)
            {
                if (!bossIsDead)
                {
                    StartCoroutine("SpawnEnemy");
                }
                else
                {
                    if (remainingEnemiesAlive < 0)
                        StartCoroutine(FindRemainingEnemies());

                    else if (remainingEnemiesAlive == 0)
                        OnWinGame();
                }
            }
            else
            {
                StartCoroutine("SpawnEnemy");
                remainingEnemiesToSpawn--;
            }

        }
   
        if (currentWave.spawnsHealth && !spawnedHealth && timeInLevel >= currentWave.timeToSpawnHealth)
        {
            StartCoroutine("SpawnPickup", healthPickup);
            spawnedHealth = true;
        }
        if (currentWave.spawnsUpgrade && !spawnedUpgrade && timeInLevel >= currentWave.timeToSpawnUpgrade && !player.recievedWeaponUpgrade)
        {
            StartCoroutine("SpawnPickup", upgradePickup);
            spawnedUpgrade = true;
        }
        if (currentWave.spawnsAmmoUpgrade && !spawnedAmmoUpgrade && timeInLevel >= currentWave.timeToSpawnAmmoUpgrade && !player.recievedAmmoUpgrade)
        {
            StartCoroutine("SpawnPickup", upgradeAmmoPickup);
            spawnedAmmoUpgrade = true;
        }

        if (devMode)
        {
            if (Input.GetKeyDown(KeyCode.Return))
            {
                StopCoroutine("SpawnEnemy");
                foreach(Enemy enemy in FindObjectsOfType<Enemy>())
                {
                    Destroy(enemy);
                }
                NextWave();
            }
        }

        timeToNextSpawn -= Time.deltaTime;
        timeInLevel += Time.deltaTime;
    }

    IEnumerator FindRemainingEnemies()
    {
        yield return new WaitForSeconds(1);
        remainingEnemiesAlive = FindObjectsOfType<Enemy>().Length;

    }

    IEnumerator SpawnEnemy()
    {
        float spawnDelay = 1f;
        float tileFlashSpeed = 4f;
        Transform spawnTile = map.GetRandomOpenTile();
        if (isCamping)
            spawnTile = map.GetTileFromPosition(playerTransform.position);
        Material tileMaterial = spawnTile.GetComponent<Renderer>().material;
        Color initialColor = Color.white;
        Color flashColor = Color.red;

        float spawnTimer = 0f;
        while (spawnTimer < spawnDelay)
        {
            tileMaterial.color = Color.Lerp(initialColor, flashColor, Mathf.PingPong(spawnTimer * tileFlashSpeed, 1));
            spawnTimer += Time.deltaTime;
            yield return null;
        }

        Enemy spawnedEnemy = Instantiate(enemy, spawnTile.position + Vector3.up, Quaternion.identity) as Enemy;
        spawnedEnemy.OnDeath += OnEnemyKill;
        // workaround for NavMeshAgent bug that I don't understand.
        yield return null;
        spawnedEnemy.Activate();
        spawnedEnemy.SetCharacteristics(currentWave.moveSpeed, currentWave.damage, currentWave.enemyHealth, currentWave.skinColor);
    }

    IEnumerator SpawnPickup(Transform pickup)
    {
        float spawnDelay = 1f;
        float tileFlashSpeed = 4f;
        Transform spawnTile = map.GetRandomOpenTile();
        Material tileMaterial = spawnTile.GetComponent<Renderer>().material;
        Color initialColor = Color.white;
        Color flashColor = Color.green;

        float spawnTimer = 0f;
        while (spawnTimer < spawnDelay)
        {
            tileMaterial.color = Color.Lerp(initialColor, flashColor, Mathf.PingPong(spawnTimer * tileFlashSpeed, 1));
            spawnTimer += Time.deltaTime;
            yield return null;
        }

        Instantiate(pickup, spawnTile.position + Vector3.up * 2, Quaternion.Euler(Vector3.right * 90));
    }

    IEnumerator SpawnBoss()
    {
        Transform spawnTile = map.GetRandomOpenTile();
        Boss boss = Instantiate(bossMan, spawnTile.position + Vector3.up * bossMan.transform.localScale.y, Quaternion.identity) as Boss;
        Color skinColor = boss.GetComponent<Renderer>().material.color;
        boss.deathEffect.GetComponent<Renderer>().sharedMaterial.color = skinColor;
        boss.OnDeath += OnBossDeath;
        yield return null;
        boss.Activate();
    }

    void ResetPlayerPosition()
    {
        playerTransform.position = map.GetCenterTilePosition() + Vector3.up * 1.5f;
    }

    void NextWave()
    {
        Enemy[] enemies = FindObjectsOfType<Enemy>();
        foreach(Enemy enemy in enemies)
        {
            Destroy(enemy.gameObject);
        }
        PickupController[] pickups = FindObjectsOfType<PickupController>();
        foreach(var item in pickups)
        {
            Destroy(item.gameObject);
        }
        waveActive = true;
        isCamping = false;
        spawnedHealth = false;
        spawnedBoss = false;
        spawnedAmmoUpgrade = false;
        spawnedUpgrade = false;

        if (OnNextWave != null)
            OnNextWave(currentWaveNumber);
        if (currentWaveIdx < waves.Length)
            currentWave = waves[currentWaveIdx++];

        currentWave.skinColor = ColorManager.instance.RandomColor();
        remainingEnemiesToSpawn = currentWave.enemyCount;
        remainingEnemiesAlive = currentWave.enemyCount;
        timeInLevel = 0f;
        ResetPlayerPosition();
    }

    void OnEnemyKill()
    {
        remainingEnemiesAlive--;
        if (remainingEnemiesAlive == 0)
        {
            if (currentWaveIdx < waves.Length)
                StartCoroutine(SpawnNextWave());
        }
    }

    void OnBossDeath()
    {
        bossIsDead = true;
    }

    void OnPlayerDeath()
    {
        Cursor.visible = true;
        isDisabled = true;
    }

    IEnumerator SpawnNextWave()
    {
        //AudioManager.instance.PlaySound2D("LevelComplete");
        waveActive = false;
        yield return currentWaveIdx > 0 ? new WaitForSeconds(timeBetweenWaves) : null;

        NextWave();
    }

    [System.Serializable]
	public class Wave
    {
        public bool infinite;
        public bool hasBoss;
        public int enemyCount;
        public float timeBetweenSpawns;

        public float moveSpeed;
        public int damage;
        public int enemyHealth;
        public Color skinColor;

        public bool spawnsHealth;
        public float timeToSpawnHealth;

        public bool spawnsUpgrade;
        public float timeToSpawnUpgrade;

        public bool spawnsAmmoUpgrade;
        public float timeToSpawnAmmoUpgrade;
    }
}

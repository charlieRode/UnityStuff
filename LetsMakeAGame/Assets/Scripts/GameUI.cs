using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.SceneManagement;

public class GameUI : MonoBehaviour {

    public Image fadePlane;
    public GameObject gameOverUI;
    public GameObject winUI;
    public Text gameOverText;
    public Transform ammoDisplay;
    public Transform bulletIcon;
    public Transform healthDisplay;
    public Transform healthFullIcon;
    public Transform healthEmptyIcon;
    public Text WaveNumber;
    public Text EnemyCount;

    Player player;
    Spawner spawner;

    public RectTransform newWaveBanner;
    public float bannerScrollSpeed = 10f;
    public float bannerPauseTime = 1f;

    Vector2 displayBottomRight;
    List<Transform> bulletIcons = new List<Transform>();
    int totalAmmo;
    int remainingAmmo;
    float iconWidth;

    Transform[] healthIcons;
    int totalHealth;
    float healthIconWidth;

    void Awake()
    {
        fadePlane.color = Color.clear;
        gameOverUI.SetActive(false);
        winUI.SetActive(false);
    }

    void Start ()
    {
        player = FindObjectOfType<Player>();
        player.OnDeath += OnGameOver;
        player.GunCtrl.OnEquipGun += this.OnEquipGun;
        player.OnUpdateHealth += UpdateHealthDisplay;

        OnEquipGun();

        spawner = FindObjectOfType<Spawner>();
        spawner.OnNextWave += DisplayWaveBanner;
        spawner.OnWinGame += OnWinGame;

        totalAmmo = player.GunCtrl.equippedGun.magazineSize;
        remainingAmmo = player.GunCtrl.equippedGun.Ammo;
        
        totalHealth = (int)player.startingHealth;
        healthIcons = new Transform[totalHealth];

        SetIconSpacing();
        UpdateAmmoDisplay();
        UpdateHealthDisplay();
    }

    void OnEquipGun()
    {
        player.GunCtrl.equippedGun.OnAmmoChange += UpdateAmmoDisplay;
        UpdateAmmoDisplay();
    }

    void SetIconSpacing()
    {
        int widthFactor = (PlayerPrefs.GetInt("ScreenResolutionIdx") + 1);
        iconWidth = totalAmmo < 10 ? 30 : 15;
        iconWidth += widthFactor * 5f;
        healthIconWidth = 45 + widthFactor * 30;
    }

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.Escape))
        {
            if (SceneManager.GetActiveScene().name == "Game")
            {
                Cursor.visible = true;
                SceneManager.LoadScene("Menu");
            }
        }
    }

    void OnWinGame()
    {
        winUI.SetActive(true);
        player.gameObject.SetActive(false);
        Cursor.visible = true;
    }

    void OnGameOver()
    {
        StartCoroutine(Fade(1f));
        gameOverUI.SetActive(true);
        Cursor.visible = true;
    }

    IEnumerator Fade(float fadeTime)
    {
        float speed = 1 / fadeTime;
        float percent = 0;

        while (percent < 1)
        {
            percent += Time.deltaTime * speed;
            fadePlane.color = Color.Lerp(Color.clear, Color.black, percent);
            gameOverText.color = Color.Lerp(Color.clear, Color.gray, percent);
            yield return null;
        }
    }

    // UI stuff
    public void StartNewGame()
    {
        SceneManager.LoadScene("Customization");
    }

    public void BackToMenu()
    {
        SceneManager.LoadScene("Menu");
    }

    void DisplayWaveBanner(int waveNumber)
    {
        string msg;

        if (spawner.waves[waveNumber - 1].hasBoss)
        {
            msg = "Boss Level";
        }
        else if (spawner.waves[waveNumber - 1].infinite)
        {
            msg = "Infinite Wave";
        }
        else
        {
            msg = string.Format("Enemies: {0}", spawner.waves[spawner.currentWaveIdx].enemyCount);
        }

        WaveNumber.text = string.Format("- Wave {0}-", waveNumber);
        EnemyCount.text = msg;

        StopCoroutine("DisplayNewWaveBanner");
        StartCoroutine("DisplayNewWaveBanner");
    }

    IEnumerator DisplayNewWaveBanner()
    {
        // y = -230 => 65
        float percent = 0f;
        int dir = 1;
        float endDelayTime = Time.time + 1 / bannerScrollSpeed + bannerPauseTime;

        while (percent >= 0)
        {
            percent += Time.deltaTime * bannerScrollSpeed * dir;
            if (percent >= 1)
            {
                percent = 1;
                if (Time.time >= endDelayTime)
                {
                    dir = -1;
                }
            }
            newWaveBanner.anchoredPosition = Vector2.up * Mathf.Lerp(-430, -290, percent);
            yield return null;
        }
    }

    void UpdateAmmoDisplay()
    {
        remainingAmmo = player.GunCtrl.equippedGun.Ammo;

        foreach (Transform t in bulletIcons)
        {
            Destroy(t.gameObject);
        }
        bulletIcons.Clear();

        for (int i = 0; i < remainingAmmo; i++)
        {
            Vector2 _position = ammoDisplay.position + Vector3.left * iconWidth * i;
            Transform icon = Instantiate(bulletIcon, _position, Quaternion.identity, ammoDisplay) as Transform;
            bulletIcons.Add(icon);
        }
    }

    
    void UpdateHealthDisplay()
    {
        int remainingHealth = (int)player.Health;

        foreach(Transform t in healthIcons)
        {
            if (t != null)
            {
                Destroy(t.gameObject);
            }
        }

        for (int i = 0; i < totalHealth; i++)
        {
            Vector2 _position = healthDisplay.position + Vector3.right * healthIconWidth * i;
            Transform icon = i >= player.Health ? healthEmptyIcon : healthFullIcon;
            healthIcons[i] = Instantiate(icon, _position, Quaternion.identity, healthDisplay) as Transform;
        }
    }
}

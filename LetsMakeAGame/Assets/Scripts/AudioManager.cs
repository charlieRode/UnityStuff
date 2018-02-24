using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;

public class AudioManager : MonoBehaviour
{
    public enum AudioChannel { MASTER, SFX, MUSIC };

    public float masterVolumePercent { get; private set; }
    public float sfxVolumePercent { get; private set; }
    public float musicVolumePercent { get; private set; }
    float musicToSfxFactor = 0.5f;

    Transform audioListener;
    Transform playerT;
    SoundLibrary library;

    float MusicVolume
    {
        get
        {
            return musicVolumePercent * masterVolumePercent * musicToSfxFactor;
        }
    }

    float SfxVolume
    {
        get
        {
            return sfxVolumePercent * masterVolumePercent;
        }
    }

    float totalSfxVolume;
    float totalMusicVolume;
    bool crossFading;

    int activeMusicSourceIdx;

    AudioSource sfx2DSource;
    AudioSource[] musicSources;

    // Singleton
    public static AudioManager instance;

    void Awake()
    {
        if (instance != null)
        {
            Destroy(gameObject);
        }
        else
        {
            instance = this;
            DontDestroyOnLoad(gameObject);
            SceneManager.sceneLoaded += OnSceneLoad;

            musicSources = new AudioSource[2];
            for (int i = 0; i < 2; i++)
            {
                GameObject newMusicSource = new GameObject("Music Source " + (i + 1));
                musicSources[i] = newMusicSource.AddComponent<AudioSource>();
                newMusicSource.transform.parent = transform;
            }
            GameObject newSfx2DSource = new GameObject("2D SFX Source");
            sfx2DSource = newSfx2DSource.AddComponent<AudioSource>();
            sfx2DSource.transform.parent = transform;

            audioListener = FindObjectOfType<AudioListener>().transform;
            AssignPlayerT();
            library = GetComponent<SoundLibrary>();

            masterVolumePercent = PlayerPrefs.GetFloat("Master Volume", 1);
            musicVolumePercent = PlayerPrefs.GetFloat("Music Volume", 1);
            sfxVolumePercent = PlayerPrefs.GetFloat("SFX Volume", 1);
        }
    }
    
    void OnSceneLoad(Scene scene, LoadSceneMode mode)
    {
        if (playerT == null)
        {
            AssignPlayerT();
        }
    }

    void AssignPlayerT()
    {
        if (FindObjectOfType<Player>() != null)
        {
            playerT = FindObjectOfType<Player>().transform;
        }
    }

    void Update()
    {
        if (!crossFading)
        {
            musicSources[activeMusicSourceIdx].volume = MusicVolume;
        }
        if (playerT != null)
        {
            audioListener.position = playerT.position;
        }
    }

    public void PlayMusic(AudioClip audioClip, float fadeDuration = 1)
    {
        if (audioClip != null)
        {
            activeMusicSourceIdx = 1 - activeMusicSourceIdx;
            musicSources[activeMusicSourceIdx].clip = audioClip;
            musicSources[activeMusicSourceIdx].Play();

            StartCoroutine(AnimateMusicCrossFade(fadeDuration));
        }
    }

    public void SetVolume(float volumePercent, AudioChannel channel)
    {
        switch (channel)
        {
            case AudioChannel.MASTER:
                masterVolumePercent = volumePercent;
                break;

            case AudioChannel.MUSIC:
                musicVolumePercent = volumePercent;
                break;

            case AudioChannel.SFX:
                sfxVolumePercent = volumePercent;
                break;

            default:
                break;
        }

        PlayerPrefs.SetFloat("Master Volume", masterVolumePercent);
        PlayerPrefs.SetFloat("Music Volume", musicVolumePercent);
        PlayerPrefs.SetFloat("SFX Volume",sfxVolumePercent);

    }

    public void PlaySound2D(string soundName)
    {
        sfx2DSource.PlayOneShot(library.GetClipFromName(soundName), SfxVolume);
    }

    public void PlaySound(string clipName, Vector3 location)
    {
        PlaySound(library.GetClipFromName(clipName), location);
    }

    public void PlaySound(AudioClip audioClip, Vector3 location)
    {
        if (audioClip != null)
            AudioSource.PlayClipAtPoint(audioClip, location, SfxVolume);
    }

    IEnumerator AnimateMusicCrossFade(float duration)
    {
        crossFading = true;
        float pct = 0f;
        while (pct < 1)
        {
            pct += Time.deltaTime * 1 / duration;
            musicSources[activeMusicSourceIdx].volume = Mathf.Lerp(0, MusicVolume, pct);
            musicSources[1 - activeMusicSourceIdx].volume = Mathf.Lerp(0, MusicVolume, 1-pct);
            yield return null;
        }
        crossFading = false;
    }
}

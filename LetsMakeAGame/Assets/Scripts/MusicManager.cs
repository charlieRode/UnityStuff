using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;

public class MusicManager : MonoBehaviour
{

    public AudioClip[] tracks;
    public AudioClip menuTheme;
    int currentTrackIdx;
    bool inGame;

    int randSeed;

    float currentTrackSecondsPlayed;

    void Awake()
    {
        SceneManager.sceneLoaded += OnLoadScene;
    }
    
    void Start()
    {
        randSeed = PlayerPrefs.GetInt("RandomSeed", 0);
        tracks = MathUtils.FisherYates(tracks, randSeed);
        PlayerPrefs.SetInt("RandomSeed", ++randSeed);
    }

    void Update()
    {
        if (inGame)
        {
            if (Input.GetKeyDown(KeyCode.Space) || currentTrackSecondsPlayed >= tracks[currentTrackIdx].length)
            {
                currentTrackIdx = (currentTrackIdx + 1) % tracks.Length;
                AudioManager.instance.PlayMusic(tracks[currentTrackIdx], 3);
                currentTrackSecondsPlayed = 0f;
            }

            currentTrackSecondsPlayed += Time.deltaTime;
        }
    }

    string TrackInfo()
    {
        AudioClip track = tracks[currentTrackIdx];
        return "(AudioClip) " + track.name + ": " + track.length + " seconds";
    }

    void OnLoadScene(Scene scene, LoadSceneMode mode)
    {
        if (scene.name == "Menu")
        {
            AudioManager.instance.PlayMusic(menuTheme, 2);
        }

        else if (scene.name == "Game")
        {
            AudioManager.instance.PlayMusic(tracks[currentTrackIdx], 3);
            currentTrackSecondsPlayed = 0f;
            inGame = true;
        }
    }

}

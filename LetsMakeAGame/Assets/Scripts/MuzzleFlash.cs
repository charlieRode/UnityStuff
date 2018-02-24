using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MuzzleFlash : MonoBehaviour {

    public GameObject flashEffect;
    public Sprite[] spriteOptions;
    public SpriteRenderer[] spriteRenderers;

    public float flashSpeed;

    void Start()
    {
        flashEffect.SetActive(false);
    }

    public void Activate()
    {
        flashEffect.SetActive(true);
        int randomIdx = Random.Range(0, spriteOptions.Length);
        foreach(SpriteRenderer sr in spriteRenderers)
        {
            sr.sprite = spriteOptions[randomIdx];
        }

        Invoke("Deactivate", flashSpeed);
    }

    void Deactivate()
    {
        flashEffect.SetActive(false);
    }

    public void SetColor(Color color)
    {
        flashEffect.GetComponentInChildren<Light>().color = color;
    }
}

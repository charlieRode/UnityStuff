using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ColorManager : MonoBehaviour
{

    public Color[] colors;
    System.Random randGen;

    public static ColorManager instance;

    void Awake()
    {
        if (instance != null)
        {
            Destroy(gameObject);
        }
        else
        {
            instance = this;
        }
        randGen = new System.Random();
    }

    public Color RandomColor()
    {
        return colors[randGen.Next(0, colors.Length)];
    }
}

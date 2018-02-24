using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public static class MathUtils
{
    static bool initialized = false;
    
    static System.Random randGen;
    static System.Random RandGen
    {
        get
        {
            if (randGen == null)
            {
                randGen = new System.Random();
            }
            return randGen;
        }
    }

    public static float Gaussian(float x, float a, float b, float c)
    {
        // Returns the value of the Gaussian distribution function f(x, a, b, c)
        // Google to see what a, b, c are.
        float exp = -Mathf.Pow(x - b, 2) / (2 * Mathf.Pow(c, 2));
        return a * Mathf.Pow((float)Math.E, exp);
    }

    public static float Gaussian01(float x)
    {
        // Returns a the f(x) value of the Gaussian curve with x and y bound between [0, 1]
        if (x < 0.005 || x > 0.995)
            return 0;

        return Gaussian(x, 1f, 0.5f, 0.15f);
    }

    public static T[] FisherYates<T>(T[] array, int seed)
    {
        System.Random randomGen = new System.Random(seed);
        for (int i = 0; i < array.Length - 1; i++)
        {
            int idx = randomGen.Next(i, array.Length);
            T temp = array[i];
            array[i] = array[idx];
            array[idx] = temp;
        }

        return array;
    }

    //public static Color RandomColor()
    //{
    //    int idx = RandGen.Next(0, colorOptions.Count);
    //    return colorOptions[idx];
    //}

    public static Color RandomColor()
    {
        return new Color((float)RandGen.NextDouble(), (float)RandGen.NextDouble(), (float)RandGen.NextDouble());
    }

    public static int RandomSeed()
    {
        return RandGen.Next(10000);
    }

    public static float RotateN(float n, float min, float max)
    {
        if (n < min)
        {
            float d = min - n;
            return max - (d % (max - min));
        }
        else if (n > max)
        {
            float d = n - max;
            return min + (d % (max - min));
        }

        return n;
    }

    public static Vector3 RotateN(Vector3 current, float min = 0, float max = 360)
    {
        return new Vector3(RotateN(current.x, min, max), RotateN(current.y, min, max), RotateN(current.z, min, max));
    }
}

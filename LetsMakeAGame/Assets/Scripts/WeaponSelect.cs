using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.UI;

public class WeaponSelect : MonoBehaviour
{

    public Transform[] weaponOptions;
    public float rotationSpeed;
    public float zoomDistance;
    public Text UIText;

    Vector3[] originalPositions;
    public static int selection;

    void Start()
    {
        originalPositions = new Vector3[weaponOptions.Length];
        for (int i = 0; i < weaponOptions.Length; i++)
        {
            originalPositions[i] = weaponOptions[i].position;
        }
    }

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.RightArrow) || Input.GetKeyDown(KeyCode.D))
        {
            if (selection < weaponOptions.Length - 1)
            {
                selection++;
                for (int i = 0; i < weaponOptions.Length; i++)
                {
                    if (i != selection)
                    {
                        weaponOptions[i].rotation = Quaternion.Euler(Vector3.up * 90);
                        weaponOptions[i].position = originalPositions[i];
                    }
                }
            }
        }
        if (Input.GetKeyDown(KeyCode.LeftArrow) || Input.GetKeyDown(KeyCode.A))
        {
            if (selection > 0)
            {
                selection--;
                for (int i = 0; i < weaponOptions.Length; i++)
                {
                    if (i != selection)
                    {
                        weaponOptions[i].rotation = Quaternion.Euler(Vector3.up * 90);
                        weaponOptions[i].position = originalPositions[i];
                    }
                        
                }
            }
        }
        
        SelectionAnimation(weaponOptions[selection]);
    }

    void SelectionAnimation(Transform weaponIcon)
    {
        weaponIcon.Rotate(Vector3.down * rotationSpeed * Time.deltaTime);
        weaponIcon.position = originalPositions[selection] + Vector3.forward * zoomDistance * -1;
        
        switch(selection)
        {
            case 0:
                UIText.text = "AUTO";
                break;
            case 1:
                UIText.text = "6 SHOOTER";
                break;
            case 2:
                UIText.text = "D.B.";
                break;
            default:
                UIText.text = "";
                break;
        }
    }

    public void Play()
    {
        SceneManager.LoadScene("Game");
    }
}

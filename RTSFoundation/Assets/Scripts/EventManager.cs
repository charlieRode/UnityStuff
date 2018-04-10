using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Events;

[System.Serializable]
public class MoveUnitEvent : UnityEvent<Vector3, Vector3> { }

public class EventManager : MonoBehaviour {

    Dictionary<string, UnityEvent<Vector3, Vector3>> registeredEvents;
    static EventManager _instance;
    public static EventManager Instance
    {
        get
        {
            if (!_instance)
            {
                _instance = FindObjectOfType(typeof(EventManager)) as EventManager;

                if (!_instance)
                {
                    Debug.Log("There needs to be one active EventManager script on a GameObject in your scene.");
                }
                else
                {
                    _instance.Invoke();
                }
            }

            return _instance;
        }
    }

    void Invoke()
    {
        if (registeredEvents == null)
            registeredEvents = new Dictionary<string, UnityEvent<Vector3, Vector3>>();
    }

    public static void StartListening(string eventName, UnityAction<Vector3, Vector3> listener)
    {

    }

    public static void StopListening(string eventName, UnityAction<Vector3, Vector3> listener)
    {
        if (Instance == null)
            return;

        UnityEvent<Vector3, Vector3> _event;
        if (Instance.registeredEvents.TryGetValue(eventName, out _event))
        {
            _event.RemoveListener(listener);
        }
    }
	
}

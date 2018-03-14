using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PathRequestManager : MonoBehaviour
{
    public bool useV2;

    Queue<PathRequest> pathRequestQueue = new Queue<PathRequest>();
    PathRequest currentPathRequest;
    PathFinding pathFinding;
    PathFindingV2 pathFindingV2;

    bool isProcessingPath;

    static PathRequestManager instance;

    void Awake()
    {
        instance = this;
        pathFinding = GetComponent<PathFinding>();
        pathFindingV2 = GetComponent<PathFindingV2>();
    }

    public static void RequestPath(Vector3 startPosition, Vector3 endPosition, Action<Vector3[], bool> callback)
    {
        PathRequest newRequest = new PathRequest(startPosition, endPosition, callback);
        instance.pathRequestQueue.Enqueue(newRequest);
        instance.TryProcessNext();
    }

    struct PathRequest
    {
        public Vector3 pathStart;
        public Vector3 pathEnd;
        public Action<Vector3[], bool> callback;

        public PathRequest(Vector3 _start, Vector3 _end, Action<Vector3[], bool> _callback)
        {
            pathStart = _start;
            pathEnd = _end;
            callback = _callback;
        }
    }

    void TryProcessNext()
    {
        if (!isProcessingPath && pathRequestQueue.Count > 0)
        {
            currentPathRequest = pathRequestQueue.Dequeue();
            isProcessingPath = true;
            if (useV2)
                pathFindingV2.StartFindPath(currentPathRequest.pathStart, currentPathRequest.pathEnd);
            else
                pathFinding.StartFindPath(currentPathRequest.pathStart, currentPathRequest.pathEnd);
        }
    }

    public void FinishedProcessingPath(Vector3[] path, bool success)
    {
        currentPathRequest.callback(path, success);
        isProcessingPath = false;
        TryProcessNext();
    }
}
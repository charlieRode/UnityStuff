using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

public class BinaryHeap<T> where T : IHeapItem<T>
{
    T[] items;
    int currentItemCount;
    public int Count
    {
        get { return currentItemCount; }
    }

    public BinaryHeap(int maxHeapSize)
    {
        items = new T[maxHeapSize];
    }

    public void Add(T item)
    {
        item.HeapIndex = currentItemCount;
        items[currentItemCount++] = item;
        SortUp(item);
    }

    public bool Contains(T item)
    {
        if (item.HeapIndex == -1)
            return false;

        return Equals(item, items[item.HeapIndex]);
    }

    public void UpdateItem(T item)
    {
        SortUp(item);
    }

    void SortUp(T item)
    {
        if (item == null)
            return;

        int parentIdx = (item.HeapIndex - 1) / 2;
        if (parentIdx > -1)
        {
            T parent = items[parentIdx];
            if (item.CompareTo(parent) > 0)
            {
                Swap(item, parent);
                SortUp(item);
            }
        }
    }

    public T Pop()
    {
        T poppedItem = items[0];
        currentItemCount--;
        items[0] = items[currentItemCount];
        items[0].HeapIndex = 0;
        SortDown(items[0]);

        return poppedItem;
    }

    void SortDown(T item)
    {
        int childIndexLeft = item.HeapIndex * 2 + 1;
        int childIndexRight = item.HeapIndex * 2 + 2;
        int swapIndex = 0;      

        if (childIndexLeft < currentItemCount)
        {
            swapIndex = childIndexLeft;

            if (childIndexRight < currentItemCount)
            {
                if (items[childIndexLeft].CompareTo(items[childIndexRight]) < 0)
                {
                    swapIndex = childIndexRight;
                }
            }

            if (item.CompareTo(items[swapIndex]) < 0)
            {
                Swap(item, items[swapIndex]);
                SortDown(item);
            }
        }
    }

    void Swap(T itemA, T itemB)
    {
        int idxA = itemA.HeapIndex;
        int idxB = itemB.HeapIndex;
        items[idxA] = itemB;
        items[idxB] = itemA;
        itemA.HeapIndex = idxB;
        itemB.HeapIndex = idxA;
    }

    void DisplayArray()
    {
        string arrString = "";
        for (int i = 0; i < items.Length; i++)
        {
            arrString += items[i] == null ? "null, " : items[i] + ", ";
        }
        Debug.Log(arrString);
        Debug.Log("item count: " + currentItemCount);
        
    }
}

public interface IHeapItem<T> : IComparable<T>
{
    int HeapIndex { get; set; }
}
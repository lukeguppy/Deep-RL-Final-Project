using System.Collections;
using System.Collections.Generic;
using UnityEngine;

// Class for defining target points in the environment
public class Target : MonoBehaviour
{
    // Public variables for coordinates and state of the target point
    public float x;
    public float y;
    public float z;
    public bool stop = false;
    public bool slow = false;
    public bool stopPoint = false;
    public bool finishedPoint = false;
    public int active = 0;
    public int waiting = 0;
    public int carLimit = 2;

    // Start is called before the first frame update
    void Start()
    {
        // Set coordinates from transform position
        SetCoordinatesFromTransform();
    }

    // Method for force starting target
    public void ForceStart()
    {
        Start();
    }

    // Method for getting coordinates of the target
    public Vector3 GetCoords()
    {
        return new Vector3(x, y, z);
    }

    // Method for setting coordinates from transform position
    public void SetCoordinatesFromTransform()
    {
        x = transform.position.x;
        y = transform.position.y;
        z = transform.position.z;
    }
}
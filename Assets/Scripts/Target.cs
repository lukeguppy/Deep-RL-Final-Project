using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Target : MonoBehaviour
{
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

    void Start()
    {
        // Set initial values based on the sphere's transform
        SetCoordinatesFromTransform();
    }

    // Update is called once per frame
    void Update()
    {
        // You can add additional logic here if needed
    }

    public void ForceStart()
    {
        // Set initial values based on the sphere's transform
        Start();
    }

    public Vector3 GetCoords() 
    {  
        return new Vector3(x, y, z);
    }

    // Set the coordinates based on the sphere's transform
    public void SetCoordinatesFromTransform()
    {
        x = transform.position.x;
        y = transform.position.y;
        z = transform.position.z;
    }
}
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Unity.VisualScripting;
using UnityEngine;

// Class for controlling junction behavior
public class JunctionController : MonoBehaviour
{
    // Serialized fields for stop points, active times, and materials
    [SerializeField]
    private Target[] stopPoints = null;
    [SerializeField]
    private double[] activeTimes = null;
    [SerializeField]
    private Material goMaterial, slowMaterial, stopMaterial;
    [SerializeField]
    private bool showStopPoints = true;

    // Variables for tracking active status and time
    public int active = 0;
    private float time;
    private int currentGo;
    private int totalActiveOrWaiting = 0;
    private bool started = false;


    // Start is called before the first frame update
    void Start()
    {
        time = 0;
        currentGo = 0;

        // Show or hide lights
        for (int i = 0; i < stopPoints.Length; i++)
        {
            stopPoints[i].GetComponent<MeshRenderer>().enabled = showStopPoints;
        }
    }

    // Update is called once per frame
    void Update()
    {
        // Use the desired traffic light behaviour

        //AlwaysGo();
        TimedLights();
    }

    private void AlwaysGo()
    {
        // Set all stop points to go state (For agent training without lights)
        if (!started)
        {
            for (int i = 0; i < stopPoints.Length; i++)
            {
                stopPoints[i].stop = false;
                MeshRenderer meshRenderer = stopPoints[i].GetComponent<MeshRenderer>();
                meshRenderer.material = goMaterial;
            }
            started = true;
        }

    }

    private void TimedLights()
    {
        if (!started)
        {
            // Initialise the first light as green and the rest as red
            stopPoints[0].stop = false;
            for (int i = 1; i < stopPoints.Length; i++) stopPoints[i].stop = true;
            started = true;
        }

        // Update the traffic light colours and sum the active counts
        for (int i = 0; i < stopPoints.Length; i++)
        {
            Target current = stopPoints[i];
            if (showStopPoints)
            {
                MeshRenderer meshRenderer = current.GetComponent<MeshRenderer>();
                meshRenderer.enabled = true;
                meshRenderer.material = current.stop ? stopMaterial : current.slow ? slowMaterial : goMaterial;
            }
            active += current.active;
        }

        // Update the current traffic light and time
        Target currentStop = stopPoints[currentGo];
        time += Time.deltaTime;

        // If the time exceeds the designated active time change to amber
        if (time > activeTimes[currentGo])
        {
            currentStop.slow = true;
        }

        // If the time exceeds the amber threshold change to red and switch to the next light
        //if (currentStop.waiting == 0 || time > activeTimes[currentGo] + 1.5f)
        if (time > activeTimes[currentGo] + 1.5f)
        {
            currentStop.slow = false;
            currentStop.stop = true;
            if (active == 0)
            {
                time = 0f;
                currentGo = (currentGo + 1) % stopPoints.Length;
                stopPoints[currentGo].stop = false;
            }
        }

        active = 0;

    }
}

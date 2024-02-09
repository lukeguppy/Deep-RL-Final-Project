using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Unity.VisualScripting;
using UnityEngine;

public class JunctionController : MonoBehaviour
{
    [SerializeField]
    private Target[] stopPoints = null;
    [SerializeField]
    private double[] activeTimes = null;
    [SerializeField]
    private Material goMaterial, slowMaterial, stopMaterial;
    [SerializeField]
    private bool showStopPoints = true;

    public int active = 0;

    private float time;
    private int currentGo;
    private bool started = false;


    // Start is called before the first frame update
    void Start()
    {
        time = 0;
        currentGo = 0;

        for (int i = 0; i < stopPoints.Length; i++)
        {
            stopPoints[i].GetComponent<MeshRenderer>().enabled = showStopPoints;
        }
    }

    // Update is called once per frame
    void Update()
    {
        if (!started)
        {
            stopPoints[0].stop = false;
            for (int i = 1; i < stopPoints.Length; i++) stopPoints[i].stop = true;
            started = true;
        }

        for (int i = 0; i < stopPoints.Length; i++)
        {
            Target current = stopPoints[i];
            if (showStopPoints)
            {
                MeshRenderer meshRenderer = current.GetComponent<MeshRenderer>();
                meshRenderer.material = current.stop ? stopMaterial : current.slow ? slowMaterial : goMaterial;
                //meshRenderer.enabled = current.waiting + current.active > 0;
            }
            active += current.active;
        }

        Target currentStop = stopPoints[currentGo];
        time += Time.deltaTime;

        if (time > activeTimes[currentGo])
        {
            currentStop.slow = true;
        }
        if (currentStop.waiting == 0 || time > activeTimes[currentGo] + 1.5f)
        {
            currentStop.slow = false;
            currentStop.stop = true;
            if (active == 0) {
                time = 0f;
                currentGo = (currentGo + 1) % stopPoints.Length;
                stopPoints[currentGo].stop = false;
            }
        }

        active = 0;
    }
}

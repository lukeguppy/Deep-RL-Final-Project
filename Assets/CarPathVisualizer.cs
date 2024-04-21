using UnityEngine;
using UnityStandardAssets.Vehicles.Car;

public class CarPathVisualizer : MonoBehaviour
{
    public Transform carTransform;
    public float distanceThreshold = 0.1f; // Distance threshold between two points
    public Color minColor = Color.yellow; // Minimum color for high speed
    public Color maxColor = Color.red; // Maximum color for low speed
    public float maxSpeed = 10f; // Maximum speed of the car

    private GameObject currentSphere;
    private Vector3 lastPosition;

    void Start()
    {
        carTransform = GetComponent<Transform>();
        maxSpeed = gameObject.GetComponent<CarController>().MaxSpeed;
        lastPosition = carTransform.position;
    }

    void Update()
    {
        if (Vector3.Distance(lastPosition, carTransform.position) > distanceThreshold)
        {
            CreatePathSphere();
            lastPosition = carTransform.position;
        }
    }


    // Create a sphere to visualize the car's path
    void CreatePathSphere()
    {
        // Calculate the speed ratio based on the car's current speed
        float speedRatio = (0.15f * gameObject.GetComponent<CarController>().MaxSpeed + 2.5f * gameObject.GetComponent<CarController>().CurrentSpeed) / gameObject.GetComponent<CarController>().MaxSpeed;

        // Create a new sphere
        if (currentSphere == null)
        {
            currentSphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            currentSphere.transform.position = carTransform.position;
            currentSphere.transform.localScale = new Vector3(speedRatio, speedRatio, speedRatio);
            currentSphere.GetComponent<Collider>().enabled = false; // Disable the collider
        }
        else
        {
            currentSphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            currentSphere.transform.position = carTransform.position;
            currentSphere.transform.localScale = new Vector3(speedRatio, speedRatio, speedRatio);
            Destroy(currentSphere.GetComponent<Collider>()); // Remove the collider if not needed
        }

        // Get the color based on the car's speed
        Color color = GetColorFromSpeed(gameObject.GetComponent<CarController>().CurrentSpeed);
        SetSphereColor(currentSphere, color);
    }

    Color GetColorFromSpeed(float speed)
    {
        // Interpolate color between minColor and maxColor based on speed
        float t = Mathf.Clamp01(speed / maxSpeed);
        return Color.Lerp(maxColor, minColor, t);
    }

    void SetSphereColor(GameObject sphere, Color color)
    {
        Renderer renderer = sphere.GetComponent<Renderer>();
        if (renderer != null)
        {
            renderer.material.color = color;
        }
    }
}

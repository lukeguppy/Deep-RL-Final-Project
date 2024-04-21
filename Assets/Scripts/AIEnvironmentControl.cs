using System;
using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;
using UnityStandardAssets.Vehicles.Car;

public class AIEnvironmentControl : MonoBehaviour
{
    [SerializeField] private Rigidbody rb; // Reference to the Rigidbody component
    [SerializeField] private float viewAngle = 45f; // Field of view angle for detecting other cars
    [SerializeField] private float xOffset, zOffset; // Offset values for positioning the car
    [SerializeField] CarController carController; // Reference to the CarController component
    [SerializeField] private List<Target> path = null; // List of targets representing the path
    [SerializeField] private float arriveDistance = 1.5f; // Distance threshold for arriving at a target
    [SerializeField] bool showCenterLine; // Flag to toggle visualization of center line
    [SerializeField] bool showTarget = false; // Flag to toggle visualization of target points
    [SerializeField] private int startIndex = 0; // Index of the start target
    [SerializeField] private bool showNextStop, showNextFinish, showCarDetection; // Flags to toggle visualization of next stop and finish points

    private GameObject[] otherCars; // Array to store references to other cars
    public int index = 0; // Index of the current target
    private Target activeStop; // Reference to active stop point
    private Target activeFinish; // Reference to active finish point
    private bool started = false; // Flag to track initialization
    private bool reachedTarget = false; // Flag to indicate if the car has reached the target

    public bool finished = false; // Flag to indicate if the car has finished the path
    public float currentSpeed = 0; // Current speed of the car
    public Target currentTarget; // Reference to the current target point
    public Target nextTarget; // Reference to the next target point
    public Target nextStop; // Reference to the next stop point

    public LineRenderer lineRenderer; // Reference to the LineRenderer component


    private void Start()
    {
        lineRenderer = gameObject.AddComponent<LineRenderer>();
    }

    private void Setup()
    {
        // Destroy the object if the path is empty
        if (path.Count == 0)
        {
            Destroy(gameObject);
            return;
        }

        finished = false;
        index = startIndex;

        rb.velocity = rb.angularVelocity = Vector3.zero;

        // Start the car at initial target location
        transform.position = path[index].GetCoords() + new Vector3(xOffset, 0.5f, zOffset);

        // Calculate the direction to the next target
        Vector3 direction = path[index + 1].GetCoords() - path[index].GetCoords();

        // Calculate the rotation angle to the next target
        float targetAngle = Mathf.Atan2(direction.x, direction.z) * Mathf.Rad2Deg;

        // Apply the rotation around the Y-axis to face the next target
        transform.rotation = Quaternion.Euler(0f, targetAngle, 0f);

        // Set current and next target points
        currentTarget = path[index];
        nextTarget = path[(index + 1) % path.Count];
        currentTarget.GetComponent<MeshRenderer>().enabled = showTarget;

        // Update next stop point (traffic light)
        if (nextStop != null) nextStop.waiting = (int)MathF.Max(0, nextStop.waiting - 1);
        nextStop = GetNextStop();
        nextStop.waiting++;

        // Update active finish point
        activeFinish = GetActiveFinish();
        activeFinish.active++;
    }

    private void Update()
    {
        if (!started)
        {
            // Force initialisation of the targets before using their coordinates otherwise they are 0
            foreach (var target in path) target.ForceStart();
            Setup();
            GameObject[] potentialCars = GameObject.FindGameObjectsWithTag("Car");
            otherCars = new GameObject[potentialCars.Length - 1];

            int j = 0;

            for (int i = 0; i < potentialCars.Length; i++)
            {
                if (potentialCars[i].GameObject() != gameObject)
                {
                    otherCars[j++] = potentialCars[i];
                }
            }

            started = true;
        }

        // Visualise target direction if enabled
        if (showTarget && currentTarget != null)
        {
            Vector3 targetRelativeDirection = currentTarget.GetCoords() - transform.position;

            Vector3 currentTargetPosition = transform.InverseTransformPoint(currentTarget.GetCoords());
            float distanceToTarget = currentTargetPosition.magnitude;

            Debug.DrawRay(transform.position + new Vector3(0, 0.5f, 0), targetRelativeDirection.normalized * distanceToTarget, new Color(0, 0, 1f));
        }

        // Check if the car has arrived at the target
        CheckIfArrived();
    }

    public void ResetCars()
    {
        if (!started) Update();
        foreach (var car in otherCars)
        {
            AutoCarControl carControl = car.GetComponent<AutoCarControl>();
            
            carControl.RestartCar();
        }
    }

    private void ResetTargets()
    {
        // Reset active stop and finish points
        if (activeStop != null)
        {
            activeStop.active = (int)MathF.Max(0, activeStop.active - 1);
        }

        if (activeFinish != null)
        {
            activeFinish.active = (int)MathF.Max(0, activeFinish.active - 1);
        }

    }

    public void Restart()
    {
        if (currentTarget != null) currentTarget.GetComponent<MeshRenderer>().enabled = false;
        ResetTargets();
        Setup();
    }

    public float DistanceBetweenTargets()
    {
        int previousIndex = index == 0 ? (path.Count + index - 2) % path.Count : (path.Count + index - 1) % path.Count;
        return Vector3.Distance(currentTarget.transform.position, path[previousIndex].transform.position);
    }


    public Vector2 ClosestPointOnCenterLine(Vector3 v3Position)
    {
        Vector2 v2Position = new(v3Position.x, v3Position.z);

        Target previousTarget = path[index == 0 ? (path.Count + index - 2) % path.Count : (path.Count + index - 1) % path.Count];
        Vector2 previous = new(previousTarget.transform.position.x, previousTarget.transform.position.z);
        Vector2 current = new(currentTarget.transform.position.x, currentTarget.transform.position.z);

        Vector2 AB = current - previous;
        Vector2 AC = v2Position - previous;

        // Calculate the projection of AC onto AB
        float t = Vector2.Dot(AC, AB) / Vector2.Dot(AB, AB);

        // Calculate the closest point on the line AB to point C
        Vector2 closestPoint = previous + t * AB;

        // Update the line renderer positions
        lineRenderer.SetPosition(0, v3Position); // Start position is the center of the car
        lineRenderer.SetPosition(1, new(closestPoint.x, v3Position.y, closestPoint.y)); // End position is the closest point on the line AB
        lineRenderer.enabled = showCenterLine;

        return closestPoint;
    }

    private void CheckIfArrived()
    {
        Target nextFinish = GetNextFinish();
        if (!currentTarget.stop
            && Vector3.Distance(currentTarget.GetCoords(), transform.position) < arriveDistance)
        {
            // Arrived at a target within the threshold and the target isn't a red light
            if (currentTarget.stopPoint)
            {
                // Handle arriving at a traffic light
                activeStop = currentTarget;
                activeStop.waiting = (int)MathF.Max(0, activeStop.waiting - 1);
                activeStop.active++;
                activeFinish.active = (int)MathF.Max(0, activeFinish.active - 1);
                activeFinish = GetNextFinish();
                activeFinish.active++;
            }

            if (currentTarget.finishedPoint && activeStop != null)
            {
                // Handle arriving at a finished point
                activeStop.active = (int)MathF.Max(0, activeStop.active - 1);
                activeStop = null;
                nextStop.waiting++;
            }

            // Move to the next target
            currentTarget.GetComponent<MeshRenderer>().enabled = false;
            reachedTarget = true;
            SetNextTargetIndex();
            nextStop = GetNextStop();
            currentTarget.GetComponent<MeshRenderer>().enabled = showTarget;
        }
    }

    public bool CheckReachedTarget()
    {
        CheckIfArrived();
        if (reachedTarget)
        {
            reachedTarget = false;
            return true;
        }
        else
        {
            return false;
        }
    }

    public float CalculateRelativeSpeedTowards(Rigidbody body)
    {
        // Check if the bodies are moving towards each other
        return Mathf.Max(0, rb.velocity.magnitude - body.velocity.magnitude) * 2.23693629f;
    }

    public Target GetNextStop()
    {
        // Gets the next traffic light point cycling through the targets in the path
        for (int i = index; i < path.Count + index + 1; i++)
        {
            int current = i % path.Count;
            if (path[current].stopPoint) return path[current];
        }
        return null;
    }
      
    private Target GetNextFinish()
    {
        // Gets the next finish point cycling through the targets in the path
        for (int i = index; i < path.Count + index; i++)
        {
            int current = i % path.Count;
            if (path[current].finishedPoint) return path[current];
        }
        return null;
    }

    private Target GetActiveFinish()
    {
        // Gets the current finish point cycling through the targets in the path
        for (int i = 0; i < path.Count; i++)
        {
            // Cycle through the targets backwards:
            // - if a finish point is reached it is used
            // - if a traffic light is reached before a finish point we use the next finish point (car is active in a junction)
            int current = (path.Count + index - 1 - i) % path.Count;
            if (path[current].finishedPoint) return path[current];
            if (path[current].stopPoint) return GetNextFinish();
        }
        return null;
    }

    public GameObject FindNearestVehicleInfront()
    {
        if (!started) Update();

        GameObject nearestVehicle = null;
        float nearestDistance = Mathf.Infinity;

        // Get the position and forward direction of this GameObject
        Vector3 currentPosition = transform.position;
        Vector3 currentForward = transform.forward;

        foreach (GameObject vehicle in otherCars)
        {
            // Make sure the current vehicle is not the same as the one calling the function
            if (vehicle != gameObject)
            {
                // Get the position and forward direction of the current vehicle
                Vector3 vehiclePosition = vehicle.transform.position;
                Vector3 vehicleForward = vehicle.transform.forward;

                // Calculate the direction from the current vehicle to the other vehicle
                Vector3 directionToVehicle = (vehiclePosition - currentPosition).normalized;

                // Check if the other vehicle is within 45 degrees in front
                float angle = Mathf.Abs(Vector3.Angle(currentForward, directionToVehicle));
                if (angle <= viewAngle)
                {
                    // Check if they are traveling in roughly the same direction
                    float forwardAngle = Mathf.Abs(Vector3.Angle(currentForward, vehicleForward));
                    if (forwardAngle <= 85f)
                    {
                        // Calculate the distance between this object and the vehicle
                        float distance = Vector3.Distance(currentPosition, vehiclePosition);

                        // Check if this vehicle is closer than the previous nearest vehicle
                        if (distance < nearestDistance)
                        {
                            nearestDistance = distance;
                            nearestVehicle = vehicle;
                        }
                    }
                }
            }
        }

        return nearestVehicle;
    }

    public GameObject FindNearestVehicleBehind()
    {
        GameObject nearestVehicle = null;
        float nearestDistance = Mathf.Infinity;

        // Get the position and forward direction of this GameObject
        Vector3 currentPosition = transform.position;
        Vector3 currentForward = transform.forward;

        foreach (GameObject vehicle in otherCars)
        {
            // Make sure the current vehicle is not the same as the one calling the function
            if (vehicle != gameObject)
            {
                // Get the position and forward direction of the current vehicle
                Vector3 vehiclePosition = vehicle.transform.position;
                Vector3 vehicleForward = vehicle.transform.forward;

                // Calculate the direction from the current vehicle to the other vehicle
                Vector3 directionToVehicle = (vehiclePosition - currentPosition).normalized;

                // Check if the other vehicle is within 45 degrees in front
                float angle = Mathf.Abs(Vector3.Angle(currentForward, directionToVehicle));
                if (angle >= 180 - viewAngle)
                {
                    // Check if they are traveling in roughly the same direction
                    float forwardAngle = Mathf.Abs(Vector3.Angle(currentForward, vehicleForward));
                    if (forwardAngle >= 95f)
                    {
                        // Calculate the distance between this object and the vehicle
                        float distance = Vector3.Distance(currentPosition, vehiclePosition);

                        // Check if this vehicle is closer than the previous nearest vehicle
                        if (distance < nearestDistance)
                        {
                            nearestDistance = distance;
                            nearestVehicle = vehicle;
                        }
                    }
                }
            }
        }

        return nearestVehicle;
    }

    private void SetNextTargetIndex()
    {
        index++;
        if (index == path.Count)
        {
            finished = true;
            index = 0;
        }
        currentTarget = path[index];
        nextTarget = path[(index + 1) % path.Count];
    }

}

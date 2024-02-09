using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityStandardAssets.Vehicles.Car;

public class AIEnvironmentControl : MonoBehaviour
{
    [SerializeField]
    private Rigidbody rb;
    [SerializeField]
    private float viewAngle = 45f;
    [SerializeField]
    CarController carController;
    [SerializeField]
    private List<Target> path = null;
    [SerializeField]
    private float arriveDistance = 1.5f;
    [SerializeField]
    private int startIndex = 0;
    [SerializeField]
    private bool showNextStop, showNextFinish, showTarget, showCarDetection;

    private GameObject[] otherCars;

    private int index = 0;
    private Target activeStop;
    private Target activeFinish;
    private Target nextStop;
    private bool started = false;
    private bool reachedTarget = false;

    public bool finished = false;
    public float currentSpeed = 0;
    public Target currentTarget;
    public Target nextTarget;
    public float steer = 0, accelerate = 0, brake = 0;


    private void Start()
    {

    }

    private void Setup()
    {
        if (path.Count == 0)
        {
            Destroy(gameObject);
            return;
        }

        finished = false;
        index = startIndex;

        rb.velocity = rb.angularVelocity = Vector3.zero;

        // Start the car at a random target location
        transform.position = path[index].GetCoords() + new Vector3(0, 0.5f, 0);

        // Calculate the direction to the next target
        Vector3 direction = path[index + 1].GetCoords() - path[index].GetCoords();

        // Calculate the rotation angle to the next target
        float targetAngle = Mathf.Atan2(direction.x, direction.z) * Mathf.Rad2Deg;

        // Apply the rotation around the Y-axis to face the next target
        transform.rotation = Quaternion.Euler(0f, targetAngle, 0f);

        currentTarget = path[index];
        nextTarget = path[(index + 1) % path.Count];
        currentTarget.GetComponent<MeshRenderer>().enabled = true;

        if (nextStop != null) nextStop.waiting = (int)MathF.Max(0, nextStop.waiting - 1);
        nextStop = NextStop();

        nextStop.waiting++;

        activeFinish = ActiveFinish();
        activeFinish.active++;
    }

    private void Update()
    {
        if (!started)
        {
            // Force initialisation of the targets before using their coordinates otherwise they are 0
            foreach (var target in path) target.ForceStart();
            Setup();
            otherCars = GameObject.FindGameObjectsWithTag("Car");

            started = true;
        }

        if (showTarget && currentTarget != null)
        {
            Vector3 targetRelativeDirection = currentTarget.GetCoords() - transform.position;

            Vector3 currentTargetPosition = transform.InverseTransformPoint(currentTarget.GetCoords());
            float distanceToTarget = currentTargetPosition.magnitude;

            Debug.DrawRay(transform.position + new Vector3(0, 0.5f, 0), targetRelativeDirection.normalized * distanceToTarget, new Color(0, 0, 1f));
        }

        CheckIfArrived();
    }

    private void FixedUpdate()
    {
        carController.Move(steer, accelerate, accelerate, brake);
    }

    private void ResetTargets()
    {
        if (activeStop != null)
        {
            activeStop.active = (int)MathF.Max(0, activeStop.active - 1);
        }

        if (activeFinish != null)
        {
            activeFinish.active = (int)MathF.Max(0, activeFinish.active - 1);
        }

    }

    private void OnTriggerStay(Collider other)
    {
        /*if (other.CompareTag("Wall"))
        {
            Restart();
        }*/

    }

    private void OnCollisionStay(Collision collision)
    {
       /* if (collision.gameObject.CompareTag("Car"))
        {
            Restart();
        }*/
    }

    public void Restart()
    {
        if (currentTarget != null) currentTarget.GetComponent<MeshRenderer>().enabled = false;
        ResetTargets();
        Setup();

    }

    private void CheckIfArrived()
    {
        Target nextFinish = NextFinish();
        if (!currentTarget.stop
            && (!currentTarget.stopPoint || nextFinish.active < nextFinish.carLimit)
            && Vector3.Distance(currentTarget.GetCoords(), transform.position) < arriveDistance)
        {
            if (currentTarget.stopPoint)
            {
                activeStop = currentTarget;
                activeStop.waiting = (int)MathF.Max(0, activeStop.waiting - 1);
                activeStop.active++;
                activeFinish.active = (int)MathF.Max(0, activeFinish.active - 1);
                activeFinish = NextFinish();
                activeFinish.active++;
            }

            if (currentTarget.finishedPoint && activeStop != null)
            {
                activeStop.active = (int)MathF.Max(0, activeStop.active - 1);
                activeStop = null;
                nextStop = NextStop();
                nextStop.waiting++;
            }

            currentTarget.GetComponent<MeshRenderer>().enabled = false;
            reachedTarget = true;
            SetNextTargetIndex();
            currentTarget.GetComponent<MeshRenderer>().enabled = true;
        }
    }

    public bool CheckReachedTarget()
    {
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

    private float CalculateRelativeSpeedTowards(Rigidbody body)
    {
        // Check if the bodies are moving towards each other
        return Mathf.Max(0, rb.velocity.magnitude - body.velocity.magnitude) * 2.23693629f;
    }

    private Target NextStop()
    {
        for (int i = index; i < path.Count + index; i++)
        {
            int current = i % path.Count;
            if (path[current].stopPoint) return path[current];
        }
        return null;
    }
      
    private Target NextFinish()
    {
        for (int i = index; i < path.Count + index; i++)
        {
            int current = i % path.Count;
            if (path[current].finishedPoint) return path[current];
        }
        return null;
    }

    private Target ActiveFinish()
    {
        for (int i = 0; i < path.Count; i++)
        {
            int current = (path.Count + index - 1 - i) % path.Count;
            if (path[current].finishedPoint) return path[current];
            if (path[current].stopPoint) return NextFinish();
        }
        return null;
    }

    GameObject FindNearestVehicleInfront()
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

    GameObject FindNearestVehicleBehind()
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
        index = index + 1;
        if (index == path.Count)
        {
            finished = true;
            index = 0;
        }
        currentTarget = path[index];
        nextTarget = path[(index + 1) % path.Count];
    }

}

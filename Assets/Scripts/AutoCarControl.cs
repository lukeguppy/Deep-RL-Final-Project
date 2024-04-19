using System;
using System.Collections.Generic;
using UnityEngine;
using UnityStandardAssets.Vehicles.Car;
using Random = UnityEngine.Random;

public class AutoCarControl : MonoBehaviour
{
    [SerializeField]
    private Rigidbody rb;
    [SerializeField]
    private float stopDistance = 10f;
    [SerializeField]
    private float viewAngle = 45f;
    [SerializeField]
    CarController carController;
    [SerializeField]
    private List<Target> path = null;
    [SerializeField]
    private List<int> validSpawns = null;
    [SerializeField]
    private float arriveDistance = 1.5f;
    [SerializeField]
    private Target currentTarget;
    [SerializeField]
    private int startIndex = -1;
    [SerializeField]
    private bool showNextStop, showNextFinish, showTarget, showCarDetection;

    private GameObject[] otherCars;

    private int index = 0;
    private Target activeStop;
    private Target activeFinish;
    private Target nextStop;
    private bool started = false;
    [SerializeField]
    public float steer = 0, accelerate = 0, brake = 0;

    public float currentSpeed = 0;
    

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

        index = (startIndex >= 0) ? validSpawns[startIndex] : validSpawns[Random.Range(0, validSpawns.Count)];

        rb.velocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;

        // Start the car at a random target location
        transform.position = path[index].GetCoords() + new Vector3(0,0.5f,0);

        // Calculate the direction to the next target
        Vector3 direction = path[(index + 1)%path.Count].GetCoords() - path[index].GetCoords();

        // Calculate the rotation angle to the next target
        float targetAngle = Mathf.Atan2(direction.x, direction.z) * Mathf.Rad2Deg;

        // Apply the rotation around the Y-axis to face the next target
        transform.rotation = Quaternion.Euler(0f, targetAngle, 0f);

        currentTarget = path[index];

        if(nextStop != null) nextStop.waiting = (int)MathF.Max(0, nextStop.waiting - 1);
        nextStop = GetNextStop();

        nextStop.waiting++;

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

        Drive();
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
        if (other.CompareTag("Wall"))
        {
            RestartCar();
        }

    }

    private void OnCollisionStay(Collision collision)
    {
        if (collision.gameObject.CompareTag("Car"))
        {
            RestartCar();
        }
    }

    public void RestartCar()
    {
        ResetTargets();
        Setup();
    }

    private void CheckIfArrived()
    {
        Target nextFinish = GetNextFinish();
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
                activeFinish = GetNextFinish();
                activeFinish.active++;
            }

            if (currentTarget.finishedPoint && activeStop != null)
            {
                activeStop.active = (int)MathF.Max(0, activeStop.active - 1);
                activeStop = null;
                nextStop = GetNextStop();
                nextStop.waiting++;
            }

            SetNextTargetIndex();
        }
    }

    private float CalculateRelativeSpeedTowards(Rigidbody body)
    {
        // Check if the bodies are moving towards each other
        return Mathf.Max(0, rb.velocity.magnitude - body.velocity.magnitude) * 2.23693629f;
    }

    private void Drive()
    {
        accelerate = 1;
        brake = 0;
        steer = 0;

        currentSpeed = carController.CurrentSpeed;

        Target nextStop = GetNextStop();
        Target nextFinish = GetNextFinish();

        Vector3 targetLocalPosition = transform.InverseTransformPoint(currentTarget.GetCoords());
        Vector3 stopLocalPosition = transform.InverseTransformPoint(nextStop.GetCoords());
        Vector3 finishLocalPosition = transform.InverseTransformPoint(nextFinish.GetCoords());
        Vector3 targetRelativeDirection = nextStop.GetCoords() - transform.position;

        GameObject nearest = FindNearestVehicleInfront();
        // Calculate the distance to the nearest vehicle
        float distanceToNearestVehicle = (nearest != null) ? Vector3.Distance(transform.position, nearest.transform.position) : Mathf.Infinity;

        float distanceToStop = stopLocalPosition.magnitude;
        float distanceToFinish = finishLocalPosition.magnitude;
        float brakeDistance = currentSpeed + currentSpeed * currentSpeed/carController.MaxSpeed;

        if ((distanceToStop > carController.MaxSpeed || nextStop.slow || nextStop.stop || (nextStop.stopPoint && nextFinish.active >= nextFinish.carLimit)) 
            && distanceToStop < distanceToNearestVehicle 
            && distanceToNearestVehicle > stopDistance
            || distanceToStop < 15 && currentSpeed > 12.5)
        {
            if (distanceToStop <= 2.5f)
            {
                brake = 1;
                accelerate = 0;
                if (showNextStop) Debug.DrawRay(transform.position + new Vector3(0, 0.5f, 0), targetRelativeDirection.normalized * distanceToStop, new(1f, 0, 0));
            }
            else if (distanceToStop <= brakeDistance && currentSpeed > 0)
            {
                //accelerate = (currentSpeed < distanceToNearestVehicle) ? 0 : -1 + (brakeDistance / currentSpeed);
                accelerate = -1 + (distanceToStop / brakeDistance);
                if (showNextStop) Debug.DrawRay(transform.position + new Vector3(0, 0.5f, 0), targetRelativeDirection.normalized * distanceToStop, new(0, 1f, 0));
            }
        }
        else if (nearest != null)
        {
            Rigidbody nearestBody = nearest.GetComponent<Rigidbody>();
            float speedTowards = CalculateRelativeSpeedTowards(nearestBody);
            Vector3 directionToNearestVehicle = (nearest.transform.position - transform.position).normalized;

            if (distanceToNearestVehicle <= stopDistance)
            {
                brake = 1;
                accelerate = 0;
                if(showCarDetection) Debug.DrawRay(transform.position + new Vector3(0, 0.5f, 0), directionToNearestVehicle * distanceToNearestVehicle, new(1, 0, 0));
            }
            else if (distanceToNearestVehicle <= brakeDistance && speedTowards > 0)
            {
                accelerate = -1 + (distanceToNearestVehicle / brakeDistance);
                if (showCarDetection) Debug.DrawRay(transform.position + new Vector3(0, 0.5f, 0), directionToNearestVehicle * distanceToNearestVehicle, new(1, 1, 0));
            }

        }

        float angle = Mathf.Atan2(targetLocalPosition.x, targetLocalPosition.z) * Mathf.Rad2Deg;
        float finishAngle = Mathf.Atan2(finishLocalPosition.x, finishLocalPosition.z) * Mathf.Rad2Deg;

        float ratio = 0.35f;
        if (angle > 0) steer = (angle > 90f) ? 1 : ratio + (1-ratio) * (angle / 90f);
        if (angle < 0) steer = (angle < -90f) ? 1 : -ratio + (1 - ratio) * (angle / 90f);

        if (showNextFinish) Debug.DrawRay(transform.position + new Vector3(0, 0.5f, 0), (nextFinish.transform.position - transform.position).normalized * Vector3.Distance(transform.position, nextFinish.transform.position), new(1, 0, 1));

        if (MathF.Abs(angle) > 35 && currentSpeed > 12.5 || currentSpeed > 8 && MathF.Abs(finishAngle) > 35 && distanceToFinish < 15)
        {
            brake = 0;
            accelerate = -1;
        } 

        //steer = (angle > turningAngleOffset) ? 1 : ((angle < -turningAngleOffset) ? -1 : 0);
        //steer *= (angle > hardSteerLimit) ? hardRotateAmount : rotateAmount;
    }

    private Target GetNextStop()
    {
        for (int i = index; i < path.Count + index ; i++)
        {
            int current = i % path.Count;
            if (path[current].stopPoint) return path[current];
        }
        return null;
    }

    private Target GetNextFinish()
    {
        for (int i = index; i < path.Count + index; i++)
        {
            int current = i % path.Count;
            if (path[current].finishedPoint) return path[current];
        }
        return null;
    }
    
    private Target GetActiveFinish()
    {
        for (int i = 0; i < path.Count; i++)
        {
            int current = (path.Count + index - 1 - i) % path.Count;
            if (path[current].finishedPoint) return path[current];
            if (path[current].stopPoint) return GetNextFinish();
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

    private void SetNextTargetIndex()
    {
        index = (index + 1) % path.Count;
        currentTarget = path[index];
    }

}

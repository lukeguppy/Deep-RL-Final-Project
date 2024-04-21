using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using UnityStandardAssets.Vehicles.Car;
using Unity.MLAgents.Actuators;
using System.Collections.Generic;
using UnityStandardAssets.CrossPlatformInput;
using static UnityEngine.GraphicsBuffer;
using System.Collections;
using UnityEditor.Rendering;
using System;
using System.Transactions;
using Unity.VisualScripting;
using UnityEditor.ShaderGraph.Internal;
using UnityEngine.UIElements;
using TMPro;

public class CarAgent : Agent
{
    // References to the car controller and environment controller
    [SerializeField] private CarController carController;
    [SerializeField] private AIEnvironmentControl environmentController;

    // Flags for collision and out-of-bounds detection
    private bool collided = false;
    private bool oob = false;

    // Variables for current and next target positions
    private Target currentTarget;
    private Target nextTarget;
    private Target nextStop;

    // Direction vectors and angles to targets
    [SerializeField] private Vector2 directionToTarget;
    private Vector2 directionToNextTarget;
    [SerializeField] private Vector2 directionToNextStop;
    private float targetAngle;
    private float nextTargetAngle;
    private float nextStopAngle;

    // Distances to targets and forward velocity
    private float targetDistance;
    private float nextTargetDistance;
    private float nextStopDistance;
    private float forwardVelocity;

    // Relative velocity of the car in-front
    private float carInfrontRelativeVelocity;

    // Steering, acceleration, and braking values
    private float steer, accelerate, brake = 0;

    // Rewards for reaching target and crashing
    private float targetReward = 25f;
    private float crashReward = -50f;

    // Time of last target reached
    public float timeOfLastTarget = 0;

    // Previous distance to target
    private float previousDistance = 0;

    // Flag for reaching target
    private bool reachedTarget = true;

    // Lane center angle and distance
    private float centerAngle;
    private float centerDistance;

    // Nearest car in front and related variables
    private GameObject carInfront;
    private float carInfrontDistance;

    public override void Initialize() {
        carController = GetComponent<CarController>();
    }

    public override void OnEpisodeBegin()
    {
        // Restart the car, intitialise the time and initialise the target information
        environmentController.Restart();
        timeOfLastTarget = Time.time;

        collided = oob = false;
        currentTarget = environmentController.currentTarget;
        nextTarget = environmentController.nextTarget;

        directionToTarget = currentTarget.transform.position - transform.position;
        directionToNextTarget = nextTarget.transform.position - transform.position;

        targetAngle = Vector3.SignedAngle(transform.forward, directionToTarget, Vector3.up);
        nextTargetAngle = Vector3.SignedAngle(transform.forward, directionToNextTarget, Vector3.up);

        targetAngle = Mathf.Repeat(targetAngle + 180f, 360f) - 180f;
        nextTargetAngle = Mathf.Repeat(nextTargetAngle + 180f, 360f) - 180f;

    }

    // Implement custom reward calculation logic here
    private float CalculateReward()
    {
        // CALCULATE REWARDS
        
        float stateReward = 0f;

        // Use the current reward function based on the stage in the curriculum

        //stateReward = Reward1();
        //stateReward = Reward2();
        //stateReward = Reward3();
        stateReward = Reward4();

        // Update the previous distance for finding the change in distance
        previousDistance = targetDistance;

        return stateReward;
    }

    private float Reward1()
    {
        float reward = 0f;

        // The angle ratio to scale the reward 
        float angleRatio = (90f - Math.Abs(targetAngle)) / 90f;

        // The change in distance clamped to avoid potential edge cases leading to large values
        // Change in distance not used if the agent just reached a target
        float distanceChange = Math.Clamp((!reachedTarget) ? previousDistance - targetDistance : 0, -1, 1);
        reachedTarget = false;

        // Reward function
        reward += (distanceChange < 0 || angleRatio < 0) ? (-1f * Math.Abs(angleRatio * distanceChange)) : (angleRatio * distanceChange);

        // Punish reversing
        if (forwardVelocity < 0) reward += 0.01f * forwardVelocity;

        return reward;
    }

    private float Reward2()
    {
        float reward = 0f;

        // Angle ratio from reward 1
        float angleRatio = (90f - Math.Abs(targetAngle)) / 90f;

        // Change in distance from reward 1
        float distanceChange = Math.Clamp((!reachedTarget) ? previousDistance - targetDistance : 0, -1, 1);
        reachedTarget = false;
        //Debug.Log(distanceChange);

        // Find the direction to the closest point on the center line
        Vector2 center = environmentController.ClosestPointOnCenterLine(transform.position);
        Vector2 directionToCenter = new(center.x - transform.position.x, center.y - transform.position.z);

        // Find the angle and distance to the closest point
        centerAngle = Vector2.SignedAngle(new(transform.forward.x, transform.forward.z), directionToCenter);
        centerAngle = Mathf.Repeat(centerAngle + 180f, 360f) - 180f;
        float centerDist = Vector2.Distance(center, new(transform.position.x, transform.position.z));
        float centerRatio = (1.6f - centerDist) / 1.6f;

        reward += (centerRatio < 0 || distanceChange < 0 || angleRatio < 0) ? (-1f * Math.Abs(centerRatio * angleRatio * distanceChange)) : (centerRatio * angleRatio * distanceChange);

        if (forwardVelocity < 0) reward += 0.025f * forwardVelocity;

        return reward;
    }

    private float Reward3()
    {
        // REWARD 3 (+ lights):

        float reward = 0f;

        // Angle ratio from reward 1
        float angleRatio = (90f - Math.Abs(targetAngle)) / 90f;

        // Find the distance to the next traffic light
        float stopDistance = Vector2.Distance(new(nextStop.x, nextStop.z), new(transform.position.x, transform.position.z));

        // set to 0.035 if it is at a stop point to substitute for encouraging moving distance
        float distanceChange = ((nextStop.stop || nextStop.slow) && ((stopDistance - 5f) < carController.MaxSpeed)) ? 0.035f : Math.Clamp((!reachedTarget) ? previousDistance - targetDistance : 0, -1, 1);
        //distanceChange = (!currentTarget.stop && !currentTarget.slow && forwardVelocity <= 0) ? -0.05f : distanceChange;
        reachedTarget = false;

        // Find center ratio from reward 2
        Vector2 center = environmentController.ClosestPointOnCenterLine(transform.position);
        Vector2 directionToCenter = new(center.x - transform.position.x, center.y - transform.position.z);
        centerAngle = Mathf.Repeat(Vector2.SignedAngle(new(transform.forward.x, transform.forward.z), directionToCenter) + 180f, 360f) - 180f;
        float centerDist = Vector2.Distance(center, new(transform.position.x, transform.position.z));
        float centerRatio = (1.6f - centerDist) / 1.6f;

        // Dot product to signal if the next traffic light is ahead or behind
        float nextStopDotProduct = (Vector3.Dot(transform.forward, (nextStop.transform.position - transform.position).normalized) > 0) ? 1 : -1;
        float scaledStopDistance = nextStopDotProduct * stopDistance;

        // Calculate the scalar for the traffic lights
        float lightRatio = 1;

        // Change from 1 if the next light is red or amber
        if (nextStop.stop || nextStop.slow)
        {
            // Apply the traffic light reward function
            if (scaledStopDistance >= 0 && scaledStopDistance <= 5f) lightRatio = 1f - 0.1f * Math.Max(0, forwardVelocity);
            else if (forwardVelocity > scaledStopDistance - 5 && scaledStopDistance > 0 && (scaledStopDistance - 5f) < carController.MaxSpeed) lightRatio = (scaledStopDistance - 5f) / Math.Max(scaledStopDistance - 5f, forwardVelocity);

            if (nextStop == currentTarget && nextStopDotProduct < 0) lightRatio = scaledStopDistance / 5f;
        }

        reward += (lightRatio < 0 || centerRatio < 0 || distanceChange < 0 || angleRatio < 0) ? (-1f * Math.Abs(lightRatio * centerRatio * angleRatio * distanceChange)) : (lightRatio * centerRatio * angleRatio * distanceChange);

        return reward;
    }

    private float Reward4()
    {
        // REWARD 4 (+ cars):

        float reward = 0f;
        float angleRatio = (90f - Math.Abs(targetAngle)) / 90f;

        float stopDistance = Vector2.Distance(new(nextStop.x, nextStop.z), new(transform.position.x, transform.position.z));

        // set to 0.035 if it is at a red or amber light
        float distanceChange = ((nextStop.stop || nextStop.slow) && ((stopDistance - 5f) < carController.MaxSpeed)) ? 0.035f : Math.Clamp((!reachedTarget) ? previousDistance - targetDistance : 0, -1, 1);
        reachedTarget = false;

        // Calculate center reward from reward 2
        Vector2 center = environmentController.ClosestPointOnCenterLine(transform.position);
        Vector2 directionToCenter = new(center.x - transform.position.x, center.y - transform.position.z);

        centerAngle = Mathf.Repeat(Vector2.SignedAngle(new(transform.forward.x, transform.forward.z), directionToCenter) + 180f, 360f) - 180f;
        centerDistance = Vector2.Distance(new(transform.position.x, transform.position.z), center);

        float centerDist = Vector2.Distance(center, new(transform.position.x, transform.position.z));
        float centerRatio = (1.6f - centerDist) / 1.6f;

        float nextStopDotProduct = (Vector3.Dot(transform.forward, (nextStop.transform.position - transform.position).normalized) > 0) ? 1 : -1;
        float scaledStopDistance = nextStopDotProduct * stopDistance;

        // Calculate the traffic light ratio from reward 3
        float lightRatio = 1;

        if (nextStop.stop || nextStop.slow)
        {
            if (scaledStopDistance >= 0 && scaledStopDistance <= 5f)
            {
                lightRatio = 1f - 0.1f * Math.Max(0, forwardVelocity);
            }
            else if (forwardVelocity > scaledStopDistance - 5 && scaledStopDistance > 0 && (scaledStopDistance - 5f) < carController.MaxSpeed)
            {
                lightRatio = (scaledStopDistance - 5f) / Math.Max(scaledStopDistance - 5f, forwardVelocity);
            }
            if (nextStop == currentTarget && nextStopDotProduct < 0)
            {
                lightRatio = scaledStopDistance / 5f;

            }
        }

        // Reward scalar to slow for other cars
        float carRatio; 

        // Apply the environment car reward function
        if (carInfrontDistance >= 0 && carInfrontDistance <= 7.5f)
        {
            carRatio = 1f - 0.5f * Math.Max(0, carInfrontRelativeVelocity);
        }
        else
        {
            carRatio = (carInfrontDistance - 7.5f) / Math.Max(carInfrontDistance - 7.5f, carInfrontRelativeVelocity);
        }

        reward += (carRatio < 0 || lightRatio < 0 || centerRatio < 0 || distanceChange < 0 || angleRatio < 0) ? (-1f * Math.Abs(carRatio * lightRatio * centerRatio * angleRatio * distanceChange)) : (carRatio * lightRatio * centerRatio * angleRatio * distanceChange);

        return reward;
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        // Store the actions given by the neural network
        steer = actions.ContinuousActions[0];
        accelerate = actions.ContinuousActions[1];
        // Only accept braking above 0.5 to avoid small handbrake values to hinder progress
        brake = actions.ContinuousActions[2] >= 0.5f ? actions.ContinuousActions[2] : 0;

        // Drive
        carController.Move(steer, accelerate, accelerate, brake);

        // Update target values
        currentTarget = environmentController.currentTarget;

        directionToTarget = new(currentTarget.x - transform.position.x, currentTarget.z - transform.position.z);
        targetAngle = Vector2.SignedAngle(new(transform.forward.x, transform.forward.z), directionToTarget);
        targetAngle = Mathf.Repeat(targetAngle + 180f, 360f) - 180f;
        targetDistance = Vector2.Distance(new(transform.position.x, transform.position.z), new(currentTarget.x, currentTarget.z));

        forwardVelocity = carController.CurrentSpeed * (Vector3.Dot(transform.forward, carController.CurrentVelocity) >= 0 ? 1 : -1);

        float currentReward = 0f;

        // Episode reset conditions disabled when applying the model
/*        if (collided)
        {
            AddReward(crashReward);
            Debug.Log(StepCount + ":  " + GetCumulativeReward());
            EndEpisode();
            environmentController.ResetCars();
            collided = false;
            return;
        }

        if (oob)
        {
            AddReward(crashReward);
            Debug.Log(StepCount + ":  " + GetCumulativeReward());
            EndEpisode();
            environmentController.ResetCars();
            oob = false;
        }

        if (Time.time - timeOfLastTarget > 100)
        {
            AddReward(crashReward);
            Debug.Log(StepCount + ":  " + GetCumulativeReward());
            EndEpisode();
            environmentController.ResetCars();
        }
*/

        // If the agent rewached a target add the target reward and reset the time
        if (environmentController.CheckReachedTarget())
        {
            reachedTarget = true;
            float inverseTurnSpeed = 1 - (Math.Max(forwardVelocity - 15f, 0) / (carController.MaxSpeed - 15f));

            currentReward += targetReward * inverseTurnSpeed;

            timeOfLastTarget = Time.time;
        }

        currentReward += CalculateReward();
        
        // Add the reward
        // Ignore first 10 steps where the car is reseting in the environment
        if (StepCount > 10) AddReward(currentReward);

        // If the agent reaches the end add a big reward and end the episode then reset the environment cars
        if (environmentController.finished)
        {
            AddReward(50f);
            Debug.Log(StepCount + ":  " + GetCumulativeReward());
            EndEpisode();
            environmentController.ResetCars();
        }

    }

    // Collect observations from the environment
    public override void CollectObservations(VectorSensor sensor)
    {
        // Get the next target position and calculate direction and angle to it
        nextTarget = environmentController.nextTarget;
        directionToNextTarget = new(nextTarget.x - transform.position.x, nextTarget.z - transform.position.z);
        nextTargetAngle = Vector2.SignedAngle(new(transform.forward.x, transform.forward.z), directionToNextTarget);
        nextTargetAngle = Mathf.Repeat(nextTargetAngle + 180f, 360f) - 180f;
        nextTargetDistance = Vector2.Distance(new(transform.position.x, transform.position.z), new(nextTarget.x, nextTarget.z));

        // Get the next stop position and calculate direction and angle to it
        nextStop = environmentController.nextStop;
        directionToNextStop = new(nextStop.x - transform.position.x, nextStop.z - transform.position.z);
        nextStopAngle = Vector2.SignedAngle(new(transform.forward.x, transform.forward.z), directionToNextStop);
        nextStopAngle = Mathf.Repeat(nextStopAngle + 180f, 360f) - 180f;
        nextStopDistance = Vector2.Distance(new(transform.position.x, transform.position.z), new(nextStop.x, nextStop.z));

        // Create vectors to store target information
        Vector2 target1Info = new(targetAngle, targetDistance);
        Vector2 target2Info = new(nextTargetAngle, nextTargetDistance);
        Vector2 nextStopInfo = new(nextStopAngle, nextStopDistance);

        // Find the nearest vehicle in front
        carInfront = environmentController.FindNearestVehicleInfront();
        float carInfrontAngle = 0f;
        carInfrontDistance = 100f;
        carInfrontRelativeVelocity = 0f;

        // Calculate angle and distance to the nearest vehicle in front
        if (carInfront != null)
        {
            Vector2 carInfrontDirection = new(carInfront.transform.position.x - transform.position.x, carInfront.transform.position.z - transform.position.z);
            carInfrontAngle = Vector2.SignedAngle(new(transform.forward.x, transform.forward.z), carInfrontDirection);
            carInfrontAngle = Mathf.Repeat(carInfrontAngle + 180f, 360f) - 180f;

            carInfrontDistance = Vector3.Distance(transform.position, carInfront.transform.position);
            carInfrontRelativeVelocity = environmentController.CalculateRelativeSpeedTowards(carInfront.GetComponent<Rigidbody>());
        }

        // Add observations to the sensor
        sensor.AddObservation(target1Info);
        sensor.AddObservation(target2Info);
        sensor.AddObservation(nextStopInfo);
        sensor.AddObservation(nextStop.stop);
        sensor.AddObservation(nextStop.slow);
        sensor.AddObservation(forwardVelocity);
        sensor.AddObservation(centerAngle);
        sensor.AddObservation(centerDistance);
        sensor.AddObservation(carInfrontAngle);
        sensor.AddObservation(carInfrontDistance);
        sensor.AddObservation(carInfrontRelativeVelocity);
    }

    private void OnTriggerStay(Collider other) {
        if (other.CompareTag("Wall")) collided = true;
        if (other.CompareTag("Car")) collided = true;
        if (other.CompareTag("OOB")) oob = true;
    }

    private void OnTriggerExit(Collider other)
    {
        if (other.CompareTag("Wall")) collided = false;
    }


    public override void Heuristic(in ActionBuffers actionsOut) {
        // This function is used to test the agent manually in the Unity Editor.
        ActionSegment<float> continuousActions = actionsOut.ContinuousActions;

        continuousActions[0] = Input.GetAxis("Horizontal");
        continuousActions[1] = Input.GetAxis("Vertical");
        continuousActions[2] = Input.GetKey(KeyCode.Space) ? 1 : 0;
    }

}

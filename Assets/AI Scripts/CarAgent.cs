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

public class CarAgent : Agent
{
    [SerializeField]
    private CarController carController;
    [SerializeField]
    private AIEnvironmentControl environmentController;

    private bool collided = false;
    private bool oob = false;
    private Target currentTarget;
    private Target nextTarget;

    private Vector2 directionToTarget;
    private Vector2 directionToNextTarget;

    private float targetAngle;
    private float nextTargetAngle;

    private float targetDistance;
    private float nextTargetDistance;

    private float forwardVelocity;

    private float steer, accelerate, brake = 0;

    private float currentReward = 0;
    private float previousStateReward = 0;
    private float statesSinceAction = 0;

    public override void Initialize() {
        carController = GetComponent<CarController>();
    }

    public override void OnEpisodeBegin() {
        environmentController.Restart();
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
        float stateReward = 0f;
        float angleWeight = 3;
        float distanceWeight = 1;
        float speedWeight = 0;
        float weightSum = angleWeight + distanceWeight + speedWeight;

        //stateReward -= targetAngle == 0 ? 0 : angleWeight/weightSum * (float)Math.Pow(Math.Abs(targetAngle / 180f), 1.5f);

        stateReward -= angleWeight / weightSum * Math.Abs(targetAngle / 180f);

        stateReward -= distanceWeight / weightSum * (targetDistance / environmentController.DistanceBetweenTargets());

        stateReward += speedWeight / weightSum * forwardVelocity / carController.MaxSpeed;

        return stateReward + 0.2f;
    }

    private void FixedUpdate()
    {


    }

    public override void OnActionReceived(ActionBuffers actions) {
        steer = actions.ContinuousActions[0];
        accelerate = actions.ContinuousActions[1];
        brake = actions.ContinuousActions[2] >= 0.5f ? actions.ContinuousActions[2] : 0;

        carController.Move(steer, accelerate, accelerate, brake);

        currentReward = CalculateReward();

        if (collided)
        {
            currentReward -= 10f;
            SetReward(-10f);
            EndEpisode();
            collided = false;
            return;
        }

        if (oob)
        {
            SetReward(-100f);
            Debug.Log(GetCumulativeReward());
            EndEpisode();
            oob = false;
        }

        if (StepCount > 50000)
        {
            SetReward(-50f);
            EndEpisode();
        }

        //Debug.Log($"{steering,14}" + " " + $"{acceleration,14}" + " " + $"{environmentController.brake,10}" + "         Reward: " + $"{reward,10}" + "    Speed: "+ $"{forwardSpeed,10}");
        //Debug.Log("Speed: " + forwardSpeed);

        if (environmentController.CheckReachedTarget())
        {
            currentReward += 100;
        }

        //AddReward(100*(currentReward - previousStateReward));
        //Debug.Log(StepCount + ":   " + 100 * (currentReward - previousStateReward));
        AddReward(currentReward);
        Debug.Log(StepCount + ":   " + currentReward);

        if (environmentController.finished)
        {
            EndEpisode();
        }

        previousStateReward = currentReward;
    }

    public override void CollectObservations(VectorSensor sensor) {
        forwardVelocity = carController.CurrentSpeed * (Vector3.Dot(transform.forward, carController.CurrentVelocity) >= 0 ? 1 : -1);

        currentTarget = environmentController.currentTarget;
        nextTarget = environmentController.nextTarget;

        directionToTarget = new(currentTarget.transform.position.x - transform.position.x, currentTarget.transform.position.z - transform.position.z);
        directionToNextTarget = new(nextTarget.transform.position.x - transform.position.x, nextTarget.transform.position.z - transform.position.z);

        targetAngle = Vector2.SignedAngle(transform.forward, directionToTarget);
        nextTargetAngle = Vector2.SignedAngle(transform.forward, directionToNextTarget);

        targetAngle = Mathf.Repeat(targetAngle + 180f, 360f) - 180f;
        nextTargetAngle = Mathf.Repeat(nextTargetAngle + 180f, 360f) - 180f;

        targetDistance = Vector2.Distance(transform.position, currentTarget.transform.position);
        nextTargetDistance = Vector2.Distance(transform.position, nextTarget.transform.position);

        Vector2 target1Info = new(targetAngle, targetDistance);
        Vector2 target2Info = new(nextTargetAngle, nextTargetDistance);

        sensor.AddObservation(target1Info);
        sensor.AddObservation(target2Info);
        sensor.AddObservation(currentTarget.stop);
        sensor.AddObservation(currentTarget.slow);
        sensor.AddObservation(forwardVelocity);
        sensor.AddObservation(collided);

    }

    private void OnTriggerStay(Collider other) {
        if (other.CompareTag("Wall")) collided = true;
        if (other.CompareTag("OOB")) oob = true;
    }

    private void OnTriggerExit(Collider other)
    {
        if (other.CompareTag("Wall")) collided = false;
    }


    public override void Heuristic(in ActionBuffers actionsOut) {
        // This function is used to test the agent manually in the Unity Editor.
        ActionSegment<float> continuousActions = actionsOut.ContinuousActions;
        //continuousActions[0] = CrossPlatformInputManager.GetAxis("Horizontal");
        //continuousActions[1] = CrossPlatformInputManager.GetAxis("Vertical");
        //continuousActions[2] = CrossPlatformInputManager.GetAxis("Jump");

        continuousActions[0] = Input.GetAxis("Horizontal");
        continuousActions[1] = Input.GetAxis("Vertical");
        continuousActions[2] = Input.GetKey(KeyCode.Space) ? 1 : 0;
    }

}

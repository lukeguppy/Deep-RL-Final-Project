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

    public float forwardVelocity;
    public float d;

    private float steer, accelerate, brake = 0;

    private float targetReward = 30f;
    private float crashReward = -50f;

    private float currentReward = 0;

    public float timeOfLastTarget = 0;
    private float previousNormalisedDistance = 0;

    public override void Initialize() {
        carController = GetComponent<CarController>();
    }

    public override void OnEpisodeBegin() {
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

        float angleWeight = 1;
        float distanceWeight = 1;
        float speedWeight = 0;
        float weightSum = angleWeight + distanceWeight + speedWeight;

        float normalisedDistance = targetDistance / environmentController.DistanceBetweenTargets();

        //stateReward -= targetAngle == 0 ? 0 : angleWeight/weightSum * (float)Math.Pow(Math.Abs(targetAngle / 180f), 1.5f);

        //stateReward += (angleWeight / weightSum) * 2.5f / 180f; // to make the angle reward positive in the range of 2.5 degrees
        //stateReward += (angleWeight / weightSum) * ((90f-Math.Abs(targetAngle)) / 90f);

        //stateReward += distanceWeight*0.5f;
        //d = previousNormalisedDistance - normalisedDistance;
        //stateReward += (distanceWeight / weightSum) * (previousNormalisedDistance - normalisedDistance);
        //Debug.Log(previousNormalisedDistance + " - " + normalisedDistance);

        //stateReward += (distanceWeight / weightSum) * Math.Min(1, 50f*(previousNormalisedDistance - normalisedDistance));
        //stateReward += (distanceWeight / weightSum) * (1-normalisedDistance);

        //float x = Math.Min(1, 50f * (previousNormalisedDistance - normalisedDistance));

        //Debug.Log(x + "       " + (1 - Math.Abs(targetAngle / 90f)));
        //Debug.Log((((90f - Math.Abs(targetAngle)) / 90f) * (distanceWeight / weightSum) * Math.Min(1, 50f * (previousNormalisedDistance - normalisedDistance))));

        //stateReward += (speedWeight / weightSum) * (forwardVelocity / carController.MaxSpeed);

        //Debug.Log((((90f - Math.Abs(targetAngle)) / 90f) * Math.Min(1, 50f * (previousNormalisedDistance - normalisedDistance))));
        stateReward += (((90f - Math.Abs(targetAngle)) / 90f) * Math.Min(1, 50f * (previousNormalisedDistance - normalisedDistance)));

        if (forwardVelocity < 0) stateReward -= 0.1f;

        previousNormalisedDistance = normalisedDistance;

        return stateReward;
    }


    public override void OnActionReceived(ActionBuffers actions)
    {
        steer = actions.ContinuousActions[0];
        accelerate = actions.ContinuousActions[1];
        brake = actions.ContinuousActions[2] >= 0.5f ? actions.ContinuousActions[2] : 0;

        // Drive
        carController.Move(steer, accelerate, accelerate, brake);
        //Debug.Log("Moved: " + steer + "   " + accelerate + "   " + brake);

        // UPDATE VALUES
        currentTarget = environmentController.currentTarget;
        nextTarget = environmentController.nextTarget;

        directionToTarget = new(currentTarget.transform.position.x - transform.position.x, currentTarget.transform.position.z - transform.position.z);
        directionToNextTarget = new(nextTarget.transform.position.x - transform.position.x, nextTarget.transform.position.z - transform.position.z);

        targetAngle = Vector2.SignedAngle(new(transform.forward.x, transform.forward.z), directionToTarget);
        nextTargetAngle = Vector2.SignedAngle(new(transform.forward.x, transform.forward.z), directionToNextTarget);

        targetAngle = Mathf.Repeat(targetAngle + 180f, 360f) - 180f;
        nextTargetAngle = Mathf.Repeat(nextTargetAngle + 180f, 360f) - 180f;

        targetDistance = Vector2.Distance(new(transform.position.x, transform.position.z), new(currentTarget.transform.position.x, currentTarget.transform.position.z));
        nextTargetDistance = Vector2.Distance(new(transform.position.x, transform.position.z), new(nextTarget.transform.position.x, nextTarget.transform.position.z));

        forwardVelocity = carController.CurrentSpeed * (Vector3.Dot(transform.forward, carController.CurrentVelocity) >= 0 ? 1 : -1);

        // Get reward

        currentReward = CalculateReward();

        if (collided)
        {
            AddReward(crashReward);
            Debug.Log(GetCumulativeReward());
            EndEpisode();
            collided = false;
            return;
        }

        if (oob)
        {
            AddReward(crashReward);
            Debug.Log(GetCumulativeReward());
            EndEpisode();
            oob = false;
        }

        if (Time.time - timeOfLastTarget > 20)
        {
            AddReward(crashReward);
            Debug.Log(GetCumulativeReward());
            EndEpisode();
        }

        if (environmentController.CheckReachedTarget())
        {
            float inverseTurnSpeed = 1 - (Math.Max(forwardVelocity - 15f, 0) / (carController.MaxSpeed - 15f));
            //float inverseNextAngle = 1 - (Math.Min(Math.Abs(nextTargetAngle), 90f) / 90f);

            //currentReward += 150 * inverseTurnSpeed * inverseNextAngle;
            currentReward += targetReward;
            //currentReward += 100;
            timeOfLastTarget = Time.time;
        }

        //if (currentTarget.stop && targetDistance <= carController.MaxSpeed && forwardVelocity >)

        //AddReward(100*(currentReward - previousStateReward));
        //Debug.Log(StepCount + ":   " + 100 * (currentReward - previousStateReward));
        AddReward(currentReward);
        //Debug.Log(StepCount + ":   " + currentReward);

        if (environmentController.finished)
        {
            Debug.Log(GetCumulativeReward());
            EndEpisode();
        }

    }

    public override void CollectObservations(VectorSensor sensor) {

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

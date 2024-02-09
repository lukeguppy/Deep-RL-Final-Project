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
    private Target currentTarget;
    private Target nextTarget;

    private Vector3 directionToTarget;
    private Vector3 directionToNextTarget;

    private float targetAngle;
    private float nextTargetAngle;

    private float targetDistance;
    private float nextTargetDistance;

    private float forwardSpeed;

    public override void Initialize() {
        carController = GetComponent<CarController>();
    }

    public override void OnEpisodeBegin() {
        environmentController.Restart();
        collided = false;
        currentTarget = environmentController.currentTarget;
        nextTarget = environmentController.nextTarget;

        directionToTarget = currentTarget.transform.position - transform.position;
        directionToNextTarget = nextTarget.transform.position - transform.position;

        targetAngle = Vector3.SignedAngle(transform.forward, directionToTarget, Vector3.up);
        nextTargetAngle = Vector3.SignedAngle(transform.forward, directionToNextTarget, Vector3.up);

        targetAngle = Mathf.Repeat(targetAngle + 180f, 360f) - 180f;
        nextTargetAngle = Mathf.Repeat(nextTargetAngle + 180f, 360f) - 180f;
    } 

    public override void OnActionReceived(ActionBuffers actions) {
        float steering = actions.ContinuousActions[0];
        float acceleration = actions.ContinuousActions[1];
        float brake = actions.ContinuousActions[2];

        // Control the car using the received actions
        environmentController.accelerate = acceleration;
        environmentController.brake = brake >= 0.25 ? brake : 0;
        environmentController.steer = steering;

        float reward = CalculateReward();

        if (collided)
        {
            reward = -10f;
            Debug.Log(GetCumulativeReward());
            EndEpisode();
            GetCumulativeReward();
            collided = false;
        }

        //Debug.Log($"{steering,14}" + " " + $"{acceleration,14}" + " " + $"{environmentController.brake,10}" + "         Reward: " + $"{reward,10}" + "    Speed: "+ $"{forwardSpeed,10}");
        //Debug.Log("Speed: " + forwardSpeed);

        AddReward(reward);

        if (environmentController.finished)
        {
            Debug.Log(GetCumulativeReward());
            EndEpisode();
        }
    }

    public override void CollectObservations(VectorSensor sensor) {
        forwardSpeed = carController.CurrentSpeed * (Vector3.Dot(transform.forward, carController.CurrentVelocity) >= 0 ? 1 : -1);

        currentTarget = environmentController.currentTarget;
        nextTarget = environmentController.nextTarget;

        directionToTarget = currentTarget.transform.position - transform.position;
        directionToNextTarget = nextTarget.transform.position - transform.position;

        targetAngle = Vector3.SignedAngle(transform.forward, directionToTarget, Vector3.up);
        nextTargetAngle = Vector3.SignedAngle(transform.forward, directionToNextTarget, Vector3.up);

        targetAngle = Mathf.Repeat(targetAngle + 180f, 360f) - 180f;
        nextTargetAngle = Mathf.Repeat(nextTargetAngle + 180f, 360f) - 180f;

        targetDistance = Vector3.Distance(transform.position, currentTarget.transform.position);
        nextTargetDistance = Vector3.Distance(transform.position, nextTarget.transform.position);

        Vector2 target1Vector = new(targetAngle, targetDistance);
        Vector2 target2Vector = new(nextTargetAngle, nextTargetDistance);

        //Debug.Log(targetAngle);

        //if (Math.Abs(targetAngle) > 160) EndEpisode();

        /*sensor.AddObservation(targetAngle);
        sensor.AddObservation(Vector3.Distance(transform.position, currentTarget.transform.position));
        sensor.AddObservation(nextTargetAngle);
        sensor.AddObservation(Vector3.Distance(transform.position, nextTarget.transform.position));*/
        sensor.AddObservation(target1Vector);
        sensor.AddObservation(target2Vector);
        sensor.AddObservation(currentTarget.stop);
        sensor.AddObservation(currentTarget.slow);
        sensor.AddObservation(forwardSpeed); // current speed (neg if reversing)

    }

    private void OnTriggerStay(Collider other) {
        if (other.CompareTag("Wall")) collided = true;
    }

    // Implement custom reward calculation logic here
    private float CalculateReward() {
        float reward = -0.1f;

        reward += Math.Min(forwardSpeed / 100f, 0.05f); 

        reward += 1 / (targetDistance + 2.5f); 
        
        //reward = collided ? -1f : reward;

        //if (forwardSpeed > 1f) reward += 0.1f;
        if (forwardSpeed < 0f) reward -= 0.5f;
        reward -= 0.5f * Math.Abs(targetAngle / 180f);

        reward += environmentController.CheckReachedTarget() ? 25 : reward;

        return reward;
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

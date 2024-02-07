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

public class CarAgent : Agent
{
    [SerializeField]
    private CarController carController;
    [SerializeField]
    private AIEnvironmentControl environmentController;

    private bool collided = false;

    public override void Initialize() {
        carController = GetComponent<CarController>();
    }

    public override void OnEpisodeBegin() {
        environmentController.Restart();
        collided = false;
    } 

    public override void OnActionReceived(ActionBuffers actions) {
        float steering = actions.ContinuousActions[0];
        float acceleration = actions.ContinuousActions[1];
        float brake = actions.ContinuousActions[2];

        // Control the car using the received actions
        environmentController.accelerate = acceleration;
        environmentController.brake = 0;
        environmentController.steer = steering;

        float reward = CalculateReward();
        AddReward(reward);
        Debug.Log(steering + " " + acceleration + " " + brake + "   " + reward);

        if (collided) {
            SetReward(-1);
            EndEpisode();
            collided = false;         
        }

        if (environmentController.finished)
        {
            EndEpisode();
        }
    }

    public override void CollectObservations(VectorSensor sensor) {
        Target currentTarget = environmentController.currentTarget;
        Target nextTarget = environmentController.nextTarget;

        Vector3 directionToTarget = currentTarget.transform.position - transform.position;
        Vector3 directionToNextTarget = currentTarget.transform.position - transform.position;
        float targetAngle = Vector3.SignedAngle(transform.forward, directionToTarget, Vector3.up);
        float nextTargetAngle = Vector3.SignedAngle(transform.forward, directionToNextTarget, Vector3.up);
        targetAngle = Mathf.Repeat(targetAngle + 180f, 360f) - 180f;
        nextTargetAngle = Mathf.Repeat(nextTargetAngle + 180f, 360f) - 180f;

        sensor.AddObservation(targetAngle);
        sensor.AddObservation(Vector3.Distance(transform.position, currentTarget.transform.position));
        sensor.AddObservation(nextTargetAngle);
        sensor.AddObservation(Vector3.Distance(transform.position, nextTarget.transform.position));
        sensor.AddObservation(currentTarget.stop);
        sensor.AddObservation(currentTarget.slow);
        sensor.AddObservation(carController.CurrentSpeed * (Vector3.Dot(transform.forward, carController.CurrentVelocity) >= 1 ? 1 : -1)); // current speed (neg if reversing)

    }

    private void OnTriggerStay(Collider other) {
        if (other.CompareTag("Wall")) collided = true;
    }

    // Implement custom reward calculation logic here
    private float CalculateReward() {
        float reward = -0.1f;
        float forwardSpeed = carController.CurrentSpeed * (Vector3.Dot(transform.forward, carController.CurrentVelocity) >= 1 ? 1 : -1);
        Target currentTarget = environmentController.currentTarget;
        Vector3 directionToTarget = currentTarget.transform.position - transform.position;
        float targetAngle = Vector3.SignedAngle(transform.forward, directionToTarget, Vector3.up);
        targetAngle = Mathf.Repeat(targetAngle + 180f, 360f) - 180f;

        if (forwardSpeed > 0f) reward += 0.05f;
        if (forwardSpeed < 0f) reward -= 0.1f;
        reward -= 0.25f * Math.Abs(targetAngle / 180f);

        reward = environmentController.CheckReachedTarget() ? 10 : reward;

        return reward;
    }

    public override void Heuristic(in ActionBuffers actionsOut) {
        // This function is used to test the agent manually in the Unity Editor.
        ActionSegment<float> continuousActions = actionsOut.ContinuousActions;

        continuousActions[0] = Input.GetAxis("Horizontal");
        continuousActions[1] = Input.GetAxis("Vertical");
        continuousActions[2] = Input.GetKey(KeyCode.Space) ? 1 : 0;
    }

}

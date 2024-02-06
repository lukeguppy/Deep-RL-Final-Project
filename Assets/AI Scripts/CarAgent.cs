using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using UnityStandardAssets.Vehicles.Car;
using Unity.MLAgents.Actuators;
using System.Collections.Generic;
using UnityStandardAssets.CrossPlatformInput;

public class CarAgent : Agent
{
    [SerializeField]
    private CarController carController;
    [SerializeField]
    private AIEnvironmentControl environmentController;

    private bool collided;

    public override void Initialize() {
        carController = GetComponent<CarController>();
        collided = false;
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
        carController.Move(steering, acceleration, acceleration, brake);       

        float reward = CalculateReward();

        reward += environmentController.CheckReachedTarget() ? 50 : 0;

        if (collided) {
            reward -= 500;
            SetReward(reward);
            // EndEpisode();
            collided = false;         
        }

        SetReward(reward);
    }

    public override void CollectObservations(VectorSensor sensor) {
        sensor.AddObservation(carController.CurrentSpeed);
        sensor.AddObservation(transform.position);
    }

    private void OnTriggerStay(Collider other) {
        if (other.CompareTag("Wall")) collided = true;
    }

    // Implement custom reward calculation logic here
    private float CalculateReward() {
        float reward = 0f;
        float forwardSpeed = Vector3.Dot(transform.forward, carController.CurrentVelocity);

        if (forwardSpeed > 10f) reward += 1f;
        if (forwardSpeed < 2f) reward -= 2f;

        return reward;
    }

    public override void Heuristic(in ActionBuffers actionsOut) {
        // This function is used to test the agent manually in the Unity Editor.
        ActionSegment<float> continuousActions = actionsOut.ContinuousActions;

        continuousActions[0] = Input.GetAxis("Horizontal");
        continuousActions[1] = Input.GetAxis("Vertical");
    }

}

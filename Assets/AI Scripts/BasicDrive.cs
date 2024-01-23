using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using UnityStandardAssets.Vehicles.Car;
using Unity.MLAgents.Actuators;
using System.Collections.Generic;
using UnityStandardAssets.CrossPlatformInput;

public class CarAgent : Agent
{
    public CarController carController;
    public List<Checkpoint> Checkpoints;
    private int cpID;
    private Vector3 startingPosition;
    private bool collided;
    private bool gotCP;
    private int step = 0;

    public override void Initialize()
    {
        carController = GetComponent<CarController>();
        startingPosition = transform.localPosition;
        cpID = 0;
        gotCP = false;
        collided = false;
    }

    public override void OnEpisodeBegin()
    {
        // Reset the car's position and other parameters here.
        // For example, you can teleport the car to a starting position.
        transform.localPosition = startingPosition;
        transform.localRotation = Quaternion.Euler(0, 0, 0);

        cpID = 0;
        gotCP = false;
        collided = false;
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        //carController.Move(0, 0.95f, 0.95f, 0f);
        step++;

        float steering = actions.ContinuousActions[0];
        float acceleration = actions.ContinuousActions[1];
        float brake = actions.ContinuousActions[2];

        // Control the car using the received actions
        carController.Move(steering, acceleration, brake, 0f);

        // Check for collisions with a wall and restart the episode
       

        float reward = CalculateReward();

        if (gotCP)
        {
            reward += 500;
            cpID++;
            //Debug.Log("reward:" + reward);
            gotCP = false;
        }

        if (collided)
        {
            reward -= 500;
            SetReward(reward);
            //Debug.Log("reward:" + reward);
            //EndEpisode();
        }

        // Provide a reward based on the agent's performance (custom logic)
        //if (step % 25 == 0) Debug.Log("reward:" + reward);
        SetReward(reward);

    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // You can add observations here to provide information to the agent.
        // For example, you can observe the car's current speed, position, etc.
        sensor.AddObservation(carController.CurrentSpeed);
        sensor.AddObservation(transform.position);
    }

    private void OnTriggerStay(Collider other)
    {
        if (other.CompareTag("Wall"))
        {
            // Check if the collided object is a Wall
            collided = true;
            return;
        }

        if (other.CompareTag("Checkpoint"))
        {
            // Check if the collided object is a Checkpoint
            Checkpoint checkpoint = other.GetComponent<Checkpoint>();

            if (checkpoint.id == cpID)
            {
                gotCP = true;
                // log got id
                Debug.Log("CHECKPOINT");
            }
            return;
        }
    }

    // Implement custom reward calculation logic here
    private float CalculateReward()
    {
        float reward = 0f;

        // Calculate the distance from the starting position
        /*float distanceFromStart = Vector3.Distance(startingPosition, transform.localPosition);

        reward += distanceFromStart / 100f;
        Debug.Log("distance reward:" + distanceFromStart / 100f);*/

        //if (carController.CurrentSpeed > 10f) reward += 10f;

        // Calculate the forward speed
        float forwardSpeed = Vector3.Dot(transform.forward, carController.CurrentVelocity);

        if (forwardSpeed > 10f) reward += 1f;
        if (forwardSpeed < 2f) reward -= 2f;

        return reward;
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        // This function is used to test the agent manually in the Unity Editor.
        ActionSegment<float> continuousActions = actionsOut.ContinuousActions;

        continuousActions[0] = Input.GetAxis("Horizontal");
        continuousActions[1] = Input.GetAxis("Vertical");
    }

}

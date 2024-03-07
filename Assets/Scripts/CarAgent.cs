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
    [SerializeField]
    private CarController carController;
    [SerializeField]
    private AIEnvironmentControl environmentController;

    private bool collided = false;
    private bool oob = false;
    private Target currentTarget;
    private Target nextTarget;
    private Target nextStop;

    [SerializeField]
    private Vector2 directionToTarget;
    private Vector2 directionToNextTarget;
    [SerializeField]
    private Vector2 directionToNextStop;

    private float targetAngle;
    private float nextTargetAngle;
    private float nextStopAngle;

    private float targetDistance;
    private float nextTargetDistance;
    private float nextStopDistance;

    private float forwardVelocity;
    private float carInfrontRelativeVelocity;

    private float steer, accelerate, brake = 0;

    private float targetReward = 25f;
    private float crashReward = -50f;

    public float timeOfLastTarget = 0;
    private float previousDistance = 0;

    private bool reachedTarget = true;

    private float centerAngle;
    private float centerDistance;

    private GameObject carInfront;
    private float carInfrontDistance;

    public override void Initialize() {
        carController = GetComponent<CarController>();
    }

    public override void OnEpisodeBegin()
    {
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


        //stateReward = Reward1();
        //stateReward = Reward2();
        //stateReward = Reward3();
        stateReward = Reward4();

        previousDistance = targetDistance;

        return stateReward;
    }

    private float Reward1()
    {
        float reward = 0f;
        // REWARD 1 (car infront + angle + distance):

        float angleRatio = (90f - Math.Abs(targetAngle)) / 90f;

        float distanceChange = Math.Clamp((!reachedTarget) ? previousDistance - targetDistance : 0, -1, 1);
        reachedTarget = false;

        //Debug.Log(distanceChange);

        reward += (distanceChange < 0 || angleRatio < 0) ? (-1f * Math.Abs(angleRatio * distanceChange)) : (angleRatio * distanceChange);

        if (forwardVelocity < 0) reward += 0.01f * forwardVelocity;

        return reward;
    }

    private float Reward2()
    {
        // REWARD 2 (+ lanes):

        float reward = 0f;

        float angleRatio = (90f - Math.Abs(targetAngle)) / 90f;

        float distanceChange = Math.Clamp((!reachedTarget) ? previousDistance - targetDistance : 0, -1, 1);
        reachedTarget = false;
        //Debug.Log(distanceChange);

        Vector2 center = environmentController.ClosestPointOnCenterLine(transform.position);
        Vector2 directionToCenter = new(center.x - transform.position.x, center.y - transform.position.z);

        centerAngle = Vector2.SignedAngle(new(transform.forward.x, transform.forward.z), directionToCenter);
        centerAngle = Mathf.Repeat(centerAngle + 180f, 360f) - 180f;
        centerDistance = Vector2.Distance(new(transform.position.x, transform.position.z), center);

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
        float angleRatio = (90f - Math.Abs(targetAngle)) / 90f;

        float stopDistance = Vector2.Distance(new(nextStop.x, nextStop.z), new(transform.position.x, transform.position.z));

        // set to 0.1 if it is at a stop point to substitute for encouraging moving distance
        float distanceChange = ((nextStop.stop || nextStop.slow) && ((stopDistance - 5f) < carController.MaxSpeed)) ? 0.035f : Math.Clamp((!reachedTarget) ? previousDistance - targetDistance : 0, -1, 1);
        //distanceChange = (!currentTarget.stop && !currentTarget.slow && forwardVelocity <= 0) ? -0.05f : distanceChange;
        reachedTarget = false;

        Vector2 center = environmentController.ClosestPointOnCenterLine(transform.position);
        Vector2 directionToCenter = new(center.x - transform.position.x, center.y - transform.position.z);

        centerAngle = Mathf.Repeat(Vector2.SignedAngle(new(transform.forward.x, transform.forward.z), directionToCenter) + 180f, 360f) - 180f;
        centerDistance = Vector2.Distance(new(transform.position.x, transform.position.z), center);

        float centerDist = Vector2.Distance(center, new(transform.position.x, transform.position.z));
        float centerRatio = (1.6f - centerDist) / 1.6f;

        float nextStopDotProduct = (Vector3.Dot(transform.forward, (nextStop.transform.position - transform.position).normalized) > 0) ? 1 : -1;
        float scaledStopDistance = nextStopDotProduct * stopDistance;

        float lightRatio = 1;

        if (nextStop.stop || nextStop.slow)
        {
            if (scaledStopDistance >= 0 && scaledStopDistance <= 5f) lightRatio = 1f - 0.1f * Math.Max(0, forwardVelocity);
            else if (forwardVelocity > scaledStopDistance - 5 && scaledStopDistance > 0 && (scaledStopDistance - 5f) < carController.MaxSpeed) lightRatio = (scaledStopDistance - 5f) / Math.Max(scaledStopDistance - 5f, forwardVelocity);

            if (nextStop == currentTarget && nextStopDotProduct < 0) lightRatio = scaledStopDistance / 5f;
        }

        reward += (lightRatio < 0 || centerRatio < 0 || distanceChange < 0 || angleRatio < 0) ? (-1f * Math.Abs(lightRatio * centerRatio * angleRatio * distanceChange)) : (lightRatio * centerRatio * angleRatio * distanceChange);

        //if (forwardVelocity < 0) reward += 0.1f * forwardVelocity;

        //if (dotProduct < 0) reward -= 0.25f;

        if (!((currentTarget.stop || currentTarget.slow) && (carInfrontDistance < carController.MaxSpeed || targetDistance < carController.MaxSpeed)))
        {
            reward -= 0.05f;
        }

        return reward;
    }

    private float Reward4()
    {
        // REWARD 4 (+ cars):

        float reward = 0f;
        float angleRatio = (90f - Math.Abs(targetAngle)) / 90f;

        float stopDistance = Vector2.Distance(new(nextStop.x, nextStop.z), new(transform.position.x, transform.position.z));

        // set to 0.1 if it is at a stop point to substitute for encouraging moving distance
        float distanceChange = ((nextStop.stop || nextStop.slow) && ((stopDistance - 5f) < carController.MaxSpeed)) ? 0.035f : Math.Clamp((!reachedTarget) ? previousDistance - targetDistance : 0, -1, 1);
        //distanceChange = (!currentTarget.stop && !currentTarget.slow && forwardVelocity <= 0) ? -0.05f : distanceChange;
        reachedTarget = false;

        Vector2 center = environmentController.ClosestPointOnCenterLine(transform.position);
        Vector2 directionToCenter = new(center.x - transform.position.x, center.y - transform.position.z);

        centerAngle = Mathf.Repeat(Vector2.SignedAngle(new(transform.forward.x, transform.forward.z), directionToCenter) + 180f, 360f) - 180f;
        centerDistance = Vector2.Distance(new(transform.position.x, transform.position.z), center);

        float centerDist = Vector2.Distance(center, new(transform.position.x, transform.position.z));
        float centerRatio = (1.6f - centerDist) / 1.6f;

        float nextStopDotProduct = (Vector3.Dot(transform.forward, (nextStop.transform.position - transform.position).normalized) > 0) ? 1 : -1;
        float scaledStopDistance = nextStopDotProduct * stopDistance;

        float lightRatio = 1;

        if (nextStop.stop || nextStop.slow)
        {
            if (scaledStopDistance >= 0 && scaledStopDistance <= 5f) lightRatio = 1f - 0.1f * Math.Max(0, forwardVelocity);
            else if (forwardVelocity > scaledStopDistance - 5 && scaledStopDistance > 0 && (scaledStopDistance - 5f) < carController.MaxSpeed) lightRatio = (scaledStopDistance - 5f) / Math.Max(scaledStopDistance - 5f, forwardVelocity);

            if (nextStop == currentTarget && nextStopDotProduct < 0) lightRatio = scaledStopDistance / 5f;
        }

        float carRatio; // Add reward for slowing for cars;

        if (carInfrontDistance >= 0 && carInfrontDistance <= 7.5f) carRatio = 1f - 0.5f *  Math.Max(0, carInfrontRelativeVelocity);
        else carRatio = (carInfrontDistance - 7.5f) / Math.Max(carInfrontDistance - 7.5f, carInfrontRelativeVelocity);

        reward += (carRatio < 0 || lightRatio < 0 || centerRatio < 0 || distanceChange < 0 || angleRatio < 0) ? (-1f * Math.Abs(carRatio*lightRatio * centerRatio * angleRatio * distanceChange)) : (carRatio*lightRatio * centerRatio * angleRatio * distanceChange);

        if (forwardVelocity < 0) reward += 0.1f * forwardVelocity;

        if (currentTarget == nextStop && nextStopDotProduct < 0) reward -= 0.25f;

        if (!((currentTarget.stop || currentTarget.slow) && (carInfrontDistance < carController.MaxSpeed || targetDistance < carController.MaxSpeed)))
        {
            reward -= 0.05f;
        }

        return reward;
    }

    private float RewardFail()
    {
        // REWARD 3 (+ lights):

        float reward = 0f;

        float angleRatio = (90f - Math.Abs(targetAngle)) / 90f;

        // set to 0.1 if it is at a stop point to substitute for encouraging moving distance
        float distanceChange = ((currentTarget.stop || currentTarget.slow) && (carInfrontDistance < carController.MaxSpeed || targetDistance < carController.MaxSpeed)) ? 0.035f : (!reachedTarget) ? previousDistance - targetDistance : 0;
        //distanceChange = (!currentTarget.stop && !currentTarget.slow && forwardVelocity <= 0) ? -0.05f : distanceChange;
        reachedTarget = false;

        Vector2 center = environmentController.ClosestPointOnCenterLine(transform.position);
        Vector2 directionToCenter = new(center.x - transform.position.x, center.y - transform.position.z);

        centerAngle = Mathf.Repeat(Vector2.SignedAngle(new(transform.forward.x, transform.forward.z), directionToCenter) + 180f, 360f) - 180f;
        centerDistance = Vector2.Distance(new(transform.position.x, transform.position.z), center);

        float centerDist = Vector2.Distance(center, new(transform.position.x, transform.position.z));
        float centerRatio = (1.6f - centerDist) / 1.6f;

        float carInfrontRelativeVelocity = (carInfront != null) ? environmentController.CalculateRelativeSpeedTowards(carInfront.GetComponent<Rigidbody>()) : 0f;

        float dotProduct = (Vector3.Dot(transform.forward, (currentTarget.transform.position - transform.position).normalized) > 0) ? 1 : -1;
        float scaledTargetDistance = dotProduct * targetDistance;

        float stopRatio = FindStopRatio(carInfrontRelativeVelocity, dotProduct);

        /*float carRatio = Math.Max(-1f, (carInfrontRelativeVelocity <= 0) ? 1 : Math.Min((carInfrontDistance - 7.5f) / carInfrontRelativeVelocity, 1));
        float lightRatio;

        if (currentTarget.stop || currentTarget.slow)
        {
            lightRatio = (scaledTargetDistance >= 0f && scaledTargetDistance <= 2.5f) ? 1 : Math.Max(-1,Math.Min((scaledTargetDistance - 2.5f) / forwardVelocity, 1));
        }
        else
        {
            lightRatio = 1f;

        }*/

        //float stopDistance = Math.Min(targetDistance - 2.5f, Math.Max(0,carInfrontDistance - Math.Max(0,carInfrontRelativeVelocity) - 7.5f));
        //float stopRatio = Math.Min(carRatio, lightRatio);
        //Debug.Log(carRatio + "  " + lightRatio + "  " + stopRatio);

        /*float stopRatio = 1;

        if (currentTarget.stop || currentTarget.slow)
        {
            if (carInfrontDistance < targetDistance)
            {
                if (carInfrontDistance <= 7.5f) stopRatio = 1f; 
                else stopRatio = (carInfrontDistance - 7.5f) / Math.Max(carInfrontDistance - 7.5f, carInfrontRelativeVelocity);
            }
            else
            {
                if (targetDistance <= 2.5f) stopRatio = forwardVelocity * 0.1f;
                else if (dotProduct < 0f) stopRatio = -targetDistance / 2.5f;
                else stopRatio = 1 - Math.Max(0, 0.1f * (forwardVelocity - (targetDistance - 2.5f)));
            }
        }*/
        //Debug.Log(stopRatio);



        reward += (stopRatio < 0 || centerRatio < 0 || distanceChange < 0 || angleRatio < 0) ? (-1f * Math.Abs(stopRatio * centerRatio * angleRatio * distanceChange)) : (stopRatio * centerRatio * angleRatio * distanceChange);

        //if (forwardVelocity < 0) reward += 0.1f * forwardVelocity;

        //if (dotProduct < 0) reward -= 0.25f;

        if (!((currentTarget.stop || currentTarget.slow) && (carInfrontDistance < carController.MaxSpeed || targetDistance < carController.MaxSpeed)))
        {
            reward -= 0.05f;
        }

            return reward;
    }

    private float FindStopRatio(float carInfrontRelativeVelocity, float dotProduct)
    {
        float stopRatio;
        float carDistance = 7.5f;
        float stopDistance = 5f;

        // safe 2.5m stop zone (target within 2.5m infront)
        float scaledTargetDist = targetDistance * dotProduct;
        if (0f <= scaledTargetDist && scaledTargetDist <= stopDistance) return 1f;

        if (targetDistance > carInfrontDistance && 0f <= carInfrontDistance && carInfrontDistance <= carDistance) return 1f;

        if (currentTarget.stop || currentTarget.slow)
        {
            stopRatio = Math.Min(CalculateStopRatio(carInfrontRelativeVelocity, carInfrontDistance - carDistance, dotProduct), CalculateStopRatio(forwardVelocity, targetDistance - stopDistance, dotProduct));
        }
        else if (carInfront != null)
        {
            stopRatio = CalculateStopRatio(carInfrontRelativeVelocity, carInfrontDistance - carDistance, dotProduct);
        }
        else
        {
            stopRatio = 1f;
        }

        return stopRatio;
    }

    private float CalculateStopRatio(float s, float d, float dotProduct)
    {
        if (dotProduct < 0) return -1f;

        float ratio = 1 - (s / d);
        if (ratio >= 0)
        {
            return ratio;
        }
        else if (ratio <= -1)
        {
            return 0;
        }
        else
        {
            return (float) Math.Pow(1f + ratio, 2f);
        }
    }


    public override void OnActionReceived(ActionBuffers actions)
    {
        steer = actions.ContinuousActions[0];
        accelerate = actions.ContinuousActions[1];
        brake = actions.ContinuousActions[2] >= 0.5f ? actions.ContinuousActions[2] : 0;

        // Drive
        carController.Move(steer, accelerate, accelerate, brake);

        // UPDATE VALUES
        currentTarget = environmentController.currentTarget;

        directionToTarget = new(currentTarget.x - transform.position.x, currentTarget.z - transform.position.z);
        targetAngle = Vector2.SignedAngle(new(transform.forward.x, transform.forward.z), directionToTarget);
        targetAngle = Mathf.Repeat(targetAngle + 180f, 360f) - 180f;
        targetDistance = Vector2.Distance(new(transform.position.x, transform.position.z), new(currentTarget.x, currentTarget.z));

        forwardVelocity = carController.CurrentSpeed * (Vector3.Dot(transform.forward, carController.CurrentVelocity) >= 0 ? 1 : -1);

        float currentReward = 0f;

        // Get reward

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

        if (environmentController.CheckReachedTarget())
        {
            reachedTarget = true;
            float inverseTurnSpeed = 1 - (Math.Max(forwardVelocity - 15f, 0) / (carController.MaxSpeed - 15f));

            currentReward += targetReward * inverseTurnSpeed;

            timeOfLastTarget = Time.time;
        }

        currentReward += CalculateReward();
        //Debug.Log(GetCumulativeReward());
        
        if (StepCount > 10) AddReward(currentReward);

        if (environmentController.finished)
        {
            AddReward(50f);
            Debug.Log(StepCount + ":  " + GetCumulativeReward());
            EndEpisode();
            environmentController.ResetCars();
        }

    }

    public override void CollectObservations(VectorSensor sensor) {


        nextTarget = environmentController.nextTarget;

        directionToNextTarget = new(nextTarget.x - transform.position.x, nextTarget.z - transform.position.z);
        nextTargetAngle = Vector2.SignedAngle(new(transform.forward.x, transform.forward.z), directionToNextTarget);
        nextTargetAngle = Mathf.Repeat(nextTargetAngle + 180f, 360f) - 180f;
        nextTargetDistance = Vector2.Distance(new(transform.position.x, transform.position.z), new(nextTarget.x, nextTarget.z));

        nextStop = environmentController.nextStop;

        directionToNextStop = new(nextStop.x - transform.position.x, nextStop.z - transform.position.z);
        nextStopAngle = Vector2.SignedAngle(new(transform.forward.x, transform.forward.z), directionToNextStop);
        nextStopAngle = Mathf.Repeat(nextStopAngle + 180f, 360f) - 180f;
        nextStopDistance = Vector2.Distance(new(transform.position.x, transform.position.z), new(nextStop.x, nextStop.z));

        Vector2 target1Info = new(targetAngle, targetDistance);
        Vector2 target2Info = new(nextTargetAngle, nextTargetDistance);
        Vector2 nextStopInfo = new(nextStopAngle, nextStopDistance);

        carInfront = environmentController.FindNearestVehicleInfront();

        float carInfrontAngle = 0f;
        carInfrontDistance = 100f;
        carInfrontRelativeVelocity = 0f;

        if (carInfront != null)
        {
            Vector2 carInfrontDirection = new(carInfront.transform.position.x - transform.position.x, carInfront.transform.position.z - transform.position.z);
            carInfrontAngle = Vector2.SignedAngle(new(transform.forward.x, transform.forward.z), carInfrontDirection);
            carInfrontAngle = Mathf.Repeat(carInfrontAngle + 180f, 360f) - 180f;

            carInfrontDistance = Vector3.Distance(transform.position, carInfront.transform.position);
            carInfrontRelativeVelocity = environmentController.CalculateRelativeSpeedTowards(carInfront.GetComponent<Rigidbody>());
        }

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
        //continuousActions[0] = CrossPlatformInputManager.GetAxis("Horizontal");
        //continuousActions[1] = CrossPlatformInputManager.GetAxis("Vertical");
        //continuousActions[2] = CrossPlatformInputManager.GetAxis("Jump");

        continuousActions[0] = Input.GetAxis("Horizontal");
        continuousActions[1] = Input.GetAxis("Vertical");
        continuousActions[2] = Input.GetKey(KeyCode.Space) ? 1 : 0;
    }

}

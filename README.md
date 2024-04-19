# Self-Driving Agent Project

## Overview
This project implements a self-driving agent within a Unity simulation environment using ML-Agents. The agent learns to navigate through various scenarios such as turns, junctions, and obeying traffic lights. The project utilises curriculum learning to gradually increase task complexity and enhance the agent's capabilities.

## Usage

### Setup Environment
Ensure you have Unity and ML-Agents installed on your system. You can follow the installation instructions provided in the [ML-Agents documentation](https://github.com/Unity-Technologies/ml-agents).

### Open Unity Project
Open the Unity project provided in the repository. You can find it in the `Self-Driving-Agent-Project` folder.

### Training
1. Modify the hyperparameters in the configuration files located in the `Configurations` folder to customise the training settings.
2. Run the training script to start training the self-driving agent:

   mlagents-learn Configurations/trainer_config.yaml --run-id=SelfDrivingAgent

3. Monitor the training progress using TensorBoard for visualisation:

   tensorboard --logdir=Logs


### Evaluation
After training, evaluate the trained agent's performance in the simulation environment:

mlagents-learn Configurations/trainer_config.yaml --run-id=SelfDrivingAgent --evaluate

### Customisation
You can customise the training environment, agent behavior, and hyperparameters to suit your specific requirements. Explore the Unity project and configuration files to make adjustments as needed.

## Folder Structure
- **Assets:** Contains Unity assets and scripts.
- **Configurations:** Contains configuration files for training.
- **Logs:** Stores training logs for analysis.
- **Models:** Stores trained models.


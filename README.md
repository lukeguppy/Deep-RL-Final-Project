# Self-Driving Agent Project

## Overview
This project implements a self-driving agent within a Unity simulation environment using ML-Agents. The agent learns to navigate through various scenarios such as turns, junctions, and obeying traffic lights. The project utilises curriculum learning to gradually increase task complexity and enhance the agent's capabilities.

## Usage

### Running the Project

First, download and unzip the project folder.

#### 1. Install Unity
   - Download and install Unity Hub from the [Unity website](https://unity.com/).
   - Select a free personal license when prompted.

---
Either:
   - Download the 2022.3.10 download from the [Unity Editor Page](https://unity.com/releases/editor/archive).
   - Select the download method using Unity Hub.

Or:
   - Open Unity Hub and navigate to the "Installs" tab.
   - Click "Add" to add a new Unity version.
   - Select the Unity version 2022.3.10 and install it.
---
- Click 'Add' and navigate to the project folder to add the project.
- Click the added file to launch the project.

#### 2. Install ML-Agents (Not necessary if included)

##### Option 1: Using Unity Package Manager
   - Open the Unity project editor.
   - Go to `Window` -> `Package Manager`.
   - Search for "ML Agents" in the Package Manager window.
   - Click on "Install" to add ML-Agents to your project.

##### Option 2: Using pip (Alternative method)
   - Ensure you have Python installed on your system. You can download Python from the [Python website](https://www.python.org/).
   - Install ML-Agents by running the following command in your terminal or command prompt:
     ```
     pip install mlagents==2.0.1
     ```

#### Camera
When running the simulation by clicking the play button at the top of the editor, you can select either game or scene mode. The scene mode shows the simulation running from the perspective of the editor which allows free cam and ability to interact with the game objects and their properties. The game mode, however, gives access to the simulation from the view of the cameras defined which are the ''Top Camera'' and the agent's perspective. To use the top camera simply enable the ''Top Camera'' game object, if disabled the agent's perspective will be used.

#### Game Mode
As discussed the simulation can be run using game mode which uses the cameras provided. This will be set to a default value of focused, unfocused or maximised. Depending on your preference you can adjust between these.


### Activating/Deactivating Cars in Unity Project

1. **Open Unity Editor:**
   - Open the project in the Unity Editor.

2. **Navigate to Hierarchy Window:**
   - In the Unity Editor, locate the "Hierarchy" window.

3. **Activate/Deactivate Cars:**
   - In the "Hierarchy" window, find the game objects representing the cars you want to activate or deactivate.
   - To activate a car, make sure the corresponding game object is enabled (checkbox is checked).
   ![Activating/Deactivating Cars](Activate-car.jpg)
   - To deactivate a car, disable the corresponding game object (uncheck the checkbox).

- The **CarAgent** is the car used for training and follows the training path. Under the behaviour parameters it is set to default behaviour type for training and does not use a neural model for its behaviour.

- The **TestCar** is the car testing the models and also follows the training path. Under the behaviour parameters it is set to inference behaviour type for uses a neural model given for its behaviour.

- The **TestCar2** is the car testing the models and follows the testing path. It also has an inference behaviour type for uses a neural model given for its behaviour.

   - Note: The neural models are not all compatible with the training cars as they have a difference in neural network structure. The final policy is given by the "AllCars" model under Assets/NN-Models if not already included. This model should be dragged to the "Model" parameter under the "Behavior Parameters" of the desired car.

- Under **EnvCars** are the environment cars. These can be selected or deselected as desired and are categorised by the 8 different routes.

### Using ML-Agents in Unity Project
Once the project is open in Unity, you can access the training environment to use or adjust any behaviours or hyperparameters to suit your specific requirements.

#### Starting Training (optional)

If you want to train your own agent in the environment these are the steps to follow.

1. **Define The Agent Class:**
   - In the agent script given in "Assets/Scripts/CarAgent", define the inputs, observations and reward system as desired.

2. **Open Terminal/Command Prompt:**
   - Navigate to the directory of the project.

3. **Activate Python Virtual Environment (if applicable):**
   - Activate a python virtual environment using:
     ```
     source venv/bin/activate   # For Linux/Mac
     venv\Scripts\activate      # For Windows
     ```

4. **Start Training:**
   - Run the following command to start training the ML-Agents:
     ```
     mlagents-learn Assets/SelfDrivePPO.yaml --run-id=YourRunID
     ```
   - This command will start training the self-driving agent using the specified configuration file (SelfDrivePPO.yaml).

#### Viewing Results Using TensorBoard

##### Downloading TensorBoard

- If you don't have TensorBoard installed, you can install it following the instructions on [the Tensorflow website](www.tensorflow.org/install).

1. **Open Terminal/Command Prompt:**
   - Navigate to the directory of the project.

2. **View Training Progress:**
   - Run the following command to start TensorBoard and visualize the training progress:
     ```
     tensorboard --logdir=results
     ```
   - This command will start TensorBoard server, allowing you to view training statistics and graphs in your web browser.

3. **Access TensorBoard in Web Browser:**
   - Open your web browser and navigate to the URL provided by TensorBoard in the command window (usually http://localhost:6006).
   - You can explore various training metrics, graphs, and visualisations to monitor the training progress and performance of the agent.

### Folder Structure

#### Assets
The "Assets" folder contains all the assets used in the Unity project, including models, textures, scripts, and scenes.

#### demos
The "demos" folder contains demonstrations used for imitation learning in ML-Agents.

#### results
The "results" folder contains output results generated from the agent's training.

##### Packages
The "Packages" folder contains external packages and dependencies used in the Unity project. This includes the ML-Agents package.

##### ProjectSettings
The "ProjectSettings" folder stores project-specific settings and configurations, such as input settings, physics settings, and editor preferences.

###### Library
The "Library" folder is automatically generated by Unity and contains metadata and other files used during development. It's recommended not to modify or include this folder in version control (applicable in [the git repository](https://github.com/lukeguppy/Deep-RL-Final-Project)).

###### Logs
The "Logs" folder stores training logs generated during the ML-Agents training process.

###### obj
The "obj" folder contains intermediate files generated during the build process. It can be automatically regenerated by Unity.

###### Temp
The "Temp" folder is used by Unity to store temporary files during the build process. It's safe to delete this folder as it can be regenerated by Unity.

###### UserSettings
The "UserSettings" folder stores user-specific settings and configurations, such as layout preferences and editor window positions.

###### venv
The "venv" folder typically contains a Python virtual environment used for managing project-specific Python dependencies and packages. It's commonly used in Python-based projects for dependency isolation and reproducibility.

### License
This software is distributed under the [MIT License](https://mit-license.org/).

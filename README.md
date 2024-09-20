# euroage_plus-project

<p align="center">
  <img src="https://euroageplus.unex.es/wp-content/uploads/2023/11/logo-euroageplus-uai-516x140.png" alt="Logo Euroage Plus">
</p>

## Description
This repository contains all the resources related to the serious game developed for the Euroage+ project, which is a key component of my thesis dissertation. In addition, it includes a stable and user-friendly version of the serious game, utilizing the Noetic version of the Robot Operating System (ROS). The previously developed code has been refactored to improve usability and overall performance.

## Table of Contents
- [Organization](#organization)
- [Dependecies](#dependecies)
- [Usage](#usage)

## Organization

This repository is organized into four main folders, each playing an essential role in the development and execution of the serious game. Below is a brief description of each folder:

1. **interface_program**:
   This folder contains two subfolders related to the therapist's interface: one for the server and another for the client. Together, they allow the therapist's interface to run, retrieving patient data stored in a pre-populated database.

2. **narrative_robot**:
   This folder contains a ROS package that acts as a bridge between the speech_services and story_telling packages. It also manages the robot entity, ensuring seamless communication with the therapist's interface.

3. **speech_services**:
   The speech_services package includes all services related to speech synthesis and voice recognition. Through ROS services, the robot can convert text to speech and transcribe speech to text.

4. **story_telling**:
   The story_telling package facilitates communication with the OpenAI assistant, enabling narrative management. It handles receiving responses from the assistant, defining story parameters, and processing user input.

## Dependecies

For detailed information about the dependencies of the overall project, please refer to the specific folders mentioned. Each folder contains a README file with instructions to help you install all the necessary components correctly and perform tests to ensure everything is functioning properly.

## Usage

This section will guide you on how to run the system.

Assuming you have installed all the dependencies and tested everything correctly, you should have at least the three packages mentioned here in your ROS workspace, along with the `interface_program` in your home directory. Follow these steps to get started:

1. **Launch the Narrative Robot Node**:
   ```bash
   roslaunch narrative_robot narrative_robot.launch

2. **Run the server theraphist interface**:
   ```bash
   cd interface_program/server/
   python3 server.py

3. **Run the client theraphist interface**:
   ```bash
   cd interface_program/client/
   python3 client.py

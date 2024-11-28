# EuroAGE+ Project - ROS Jazzy Distribution

<div align="center">
  <img src="https://euroageplus.unex.es/wp-content/uploads/2023/11/logo-euroageplus-uai-516x140.png" alt="Logo Euroage Plus">
  &nbsp;&nbsp;
  <img src="https://github.com/user-attachments/assets/79b1cfa2-7d07-4cf5-a753-731de974b14e" alt="ROS Jazzy" style="width: 289px;">
</div>

## Description
This repository contains all the resources related to the serious game developed for the [EuroAGE+ project](https://euroageplus.unex.es/pt-pt/), which was a key component of my MSc thesis dissertation [1] and was presented in the ICARA 2025 conference [2]. It also includes a stable and user-friendly version of the game, implemented with the Jazzy distribution of the Robot Operating System (ROS) 2. The project was migrated from the ROS Noetic branch to ensure compatibility with this distribution of ROS.

## Citation
You're free to reuse the source code in this repository provided that its authors' copyright is retained and you cite in your work the following publication:

Almeida, Pedro V., & Rocha, Rui P. (2025). AI-powered storytelling with a social assistive robot to foster cognitive health in seniors. 11th International Conference on Automation, Robotics, and Applications (ICARA 2025), Zagreb, Croatia.


## Table of Contents
- [Organization](#organization)
- [Dependecies](#dependecies)
- [Usage](#usage)

## Organization

This repository is organized into four main folders, each playing an essential role in the design and implementation of the serious game. Below is a brief description of each folder:

1. **`interface_program`**:
   This folder contains two subfolders related to the therapist's interface: one for the server and another for the client. Together, they allow the therapist's interface to run, retrieving patient data stored in a pre-populated database.

2. **`narrative_robot`**:
   This folder contains a ROS package that acts as a bridge between the `speech_services` and `story_telling` packages. It also manages the robot entity, ensuring seamless communication with the therapist's interface.

3. **`speech_services`**:
   The `speech_services` package includes all services related to speech synthesis and voice recognition. Through ROS services, the robot can convert text to speech and transcribe speech to text.

4. **`story_telling`**:
   The `story_telling` package facilitates communication with the OpenAI assistant, enabling narrative management. It handles receiving responses from the assistant, defining story parameters, and processing user input.

## Dependecies

For detailed information about the dependencies of the overall project, please refer to the specific folders mentioned. Each folder contains a README file with instructions to help you install all the necessary components correctly and perform tests to ensure everything is functioning properly.

## Usage

This section will guide you on how to run the system.

Assuming you have installed all the dependencies and tested everything correctly, you should have at least the three packages mentioned here in your ROS workspace, along with the `interface_program` in your ___home directory___. Follow these steps to get started:

1. **Launch the Narrative Robot Node**:
   * Open a terminal, navigate to your ROS workspace directory, and run the following command:
   ```bash
   ros2 launch narrative_robot narrative_robot.launch

2. **Run the Therapist Interface**:
   * In a new terminal, navigate to the server directory and execute:
   ```bash
   cd interface_program/server/
   python3 server.py
   ```
   
   * Open another terminal and run the client by executing:
   ```bash
   cd interface_program/client/
   python3 client.py
   ```

Now that you have everything started, you'll need the credentials for the therapist:

* **Username**: caritas@gmail.com
* **Password**: qwerty

The interface is intuitive, allowing you to generate stories based on your requests.

## References

[1] Almeida, Pedro V. (2024). Social robot interaction with human users through natural language (in Portuguese) [Master's Thesis, University of Coimbra]. [https://hdl.handle.net/10316/116624](https://hdl.handle.net/10316/116624)
[2] Almeida, Pedro V., & Rocha, Rui P. (2025). AI-powered storytelling with a social assistive robot to foster cognitive health in seniors. 11th International Conference on Automation, Robotics, and Applications (ICARA 2025), Zagreb, Croatia.

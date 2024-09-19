# Euroage_plus Project

## Description
This repository contains all the resources related to the serious game developed for the Euroage+ project as part of my thesis dissertation. Additionally, it features a stable and user-friendly version of the Noetic Robot Operating System (ROS), as the previously developed code has been adapted for enhanced usability.

## Table of Contents
- [Organization](#organization)
- [Usage](#usage)

## Organization

This repository is organized into four main folders, each playing an essential role in the development and execution of the serious game for the Euroage+ project. Below is a brief description of each folder:

1. **interface_program**:
   This folder contains two subfolders related to the therapist's interface: one for the server and another for the client. Together, they allow the therapist's interface to run, retrieving patient data stored in a pre-populated database.

2. **narrative_robot**:
   This folder contains is a ROS package that serves as the bridge between the speech_services and story_telling packages. Additionally, it is responsible for managing the robot entity, ensuring communication with the therapist's interface.

3. **speech_services**:
   The speech_services package consists of all services related to speech synthesis and voice translation. Using ROS services, the robot can convert text to speech and transform a user's spoken words into text.

4. **story_telling**:
   The story_telling package manages communication with the OpenAI assistant, allowing for narrative management. This package handles receiving responses from the assistant, as well as defining the story's parameters and processing user responses.

## Usage

To run this serious game you need to do the followed steps:

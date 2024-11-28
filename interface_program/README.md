# `interface_program` Python GUI

## Description

The `interface_program` provides a graphical user interface (GUI) for the storytelling robot system. It allows users to define story parameters, control the flow of the narrative (e.g., pausing or stopping), and save different player profiles, making it easy to use with various participants while keeping their details saved for future use.

## Citation
You're free to reuse the source code in this repository provided that its authors' copyright is retained and you cite in your work the following publication:

*Almeida, Pedro V., & Rocha, Rui P. (2025). AI-powered storytelling with a social assistive robot to foster cognitive health in seniors. 11th International Conference on Automation, Robotics, and Applications (ICARA 2025), Zagreb, Croatia.*

### Key Features

- **Define Story Parameters**: Easily set up parameters for the story, such as player details and story preferences.
- **Control the Narrative**: Pause, resume, or stop the narrative as needed.
- **Save Player Profiles**: Save and load different player profiles to retain their preferences and story settings.
- **Interactive Interface**: User-friendly GUI built with Tkinter for easy interaction with the storytelling robot system.

### Python-Based Implementation

The GUI is implemented using the `Tkinter` library, which is a built-in Python library for creating graphical user interfaces. The graphic components of the GUI are located in the `client_euroage` folder. 

In order to store participant data, the `server_euroage` folder handles saving and managing information. Communication between the client and server is done via sockets, ensuring seamless data transfer.

Additionally, the `client_euroage` folder contains a third-party connection to the `narrative_robot`, enabling the robot to be controlled directly from the interface.

### Running the GUI

To launch the interface, you need to run two separate terminals:

1. **Server Terminal**: 
   Navigate to the folder containing the server code and run the following command to start the server:

   ```bash
   python3 server.py
   ```

2. **Client Terminal**: 
   Navigate to the folder containing the client code and run the following command to start the client:

   ```bash
   python3 client.py
   ```

This will initiate the client program, which connects to the server and communicates with the narrative_robot system. You will then be able to interact with the GUI and control the robot’s storytelling features.

Once both the server and client are running, the interface will be fully functional, allowing you to set story parameters, control the narrative, and save player profiles.

## Acknowledgments

Special thanks to:
- **Prof. Rui P. Rocha** ([rprocha@isr.uc.pt](mailto:rprocha@isr.uc.pt)) for his continuous guidance, support, and motivation throughout this project.
- **Prof. Fernando Perdigão** ([fp@deec.uc.pt](mailto:fp@deec.uc.pt)) for his valuable and critical insights.


## Contact

For any issues or further inquiries, feel free to contact the package maintainers at [pedro.almeida@isr.uc.pt](pedro.almeida@isr.uc.pt).

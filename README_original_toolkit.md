# RaPToR: Rapid Prototyping Toolkit for Robotics

RaPToR is created for quickly prototyping robot behaviour with Robot Operating System 2 (ROS 2) while reducing the entry barrier to implementing robot applications.

## Instructions to run locally
1. Install ROS 2 onto the device that connects to your robot. Make sure to install the same version as your Linux image.

2. Follow either of the following steps:
    - Compile everything and run the main.py.
    - Run the toolkit with.

            cd Toolkit
            python3 main.py

3. To test the toolkit is running properly, run the following command on a different terminal window

        python3 listen.py

    This will run the example listener that will print out the current sensor values. It will start displaying sensor values after you activate at least one sensor from the graphical user interface.

## Files
### Required files
The files required for the RaPToR toolkit are as follow:

- actions.py
- getters.py
- main.py
- move.py
- recordings.json
- sensor_websocket.py
- sensors.py
- template_generator.py
- terminal.py

### Temporary files
The following files are not required for the toolkit to function normally:

- temp.py
    
    This is only used for debugging

- listen.py

    This is used to test if the websocket is set up and running properly

- template.py

    This is the template code file that you can generate using the toolkit. The provided one is generated using the iRobot Create 3 Educational Robot. [^1]

---
[^1]: https://edu.irobot.com/what-we-offer/create3
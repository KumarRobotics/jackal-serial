# Jackal Service

This repo contains the code for controling the jackal and jeti transmitter. For teleoperation a signal is sent from the jeti transmitter to the jeti reciever.
 The `jeti_joy` node gets this signal over serial and translates it to a `Joy` msg and publishes it on the `/joy` topic. The `jackal_teleop_node` converts the joy topic 
to a velocity command in m/s for linear and rad/sec for angular. This gets sent to the `jackal_velocity_controller` which applies those commands to a differential
drive controller. This publishes `joint_states` for each wheel that then get translated to a `Drive` msg to be serialized and sent to the micro controller over
serial.

### Modules
 - `jackal-bridge`: A zmq bridge that can send velocity commands over zmq. This is now deprecated.
 - `jackal-control`: this contains the config files and standalone launch files to start the differential drive controller.
 - `jackal-description`: this contains urdf descriptions of the jackal that the differential drive controller uses to calculate the drive to each wheel.
 - `jackal-launch`: contains the main launch file for the jackal.
 - `jackal-serial`: connects to the jackal over serial and sends serialized timesync and drive commands. I'm working on deserializing what we get
back from the microcontroller but its not necessary to make the robot move correctly.
 - `jackal-teleop`: conects to the jeti reciever and converts joy msgs to velocity commands.
 - `serivce`: contains a service file to start the docker container upon boot. 

Everything that can be started as a component starts as one. I'm still working on makeing the velocity controller a component, it seems it does not support this out of the box.

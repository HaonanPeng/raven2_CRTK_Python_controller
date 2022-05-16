# raven2_CRTK_Python_controller
This is a controller for RAVEN II surgical robot, based on CRTK API, using Python

***Please notice that this Python controller for RAVEN is still under development and the safety of using it has not been fully tested. If you want to use it, please contact me at penghaonan1993@gmail.com and I am glad to help!***

<p align="center">
  <img src="https://github.com/HaonanPeng/raven2_CRTK_Python_controller/blob/main/fig/sum_raven_crtk_py.png" width="750" height="369">
</p>

# To use the keyboard controller:
First, normally start RAVEN by:

```
roslaunch raven_2 raven_2.launch
```

For more details about starting RAVEN, please refer to this [Quick Start Guide](https://github.com/HaonanPeng/raven2_CRTK_Python_controller/blob/main/doc/UW-BRL%20RAVEN%20Quick-Start%20Guide%20-%20Google%20Docs.pdf). **And please do not forget to manually move all the joint a little bit before pressing the silver button to 'bump the encoders'.**


Then, in a new terminal, find the path of the 'python_controller' and start the keyboard controller by:

```
python run_r2_keyboard_controller.py
```

Please notice that the current version only allows one key input at a time for stability.

To change velocity, please go to 'run_r2_keyboard_controller.py', velocity variable of eah joint is at the beginning of the code.

To change maximum allowed velocity, please go to 'raven_py_controller.py' and find the line:

```
self.max_jr = np.array([5*Deg2Rad, 5*Deg2Rad, 0.02, 15*Deg2Rad, 15*Deg2Rad, 15*Deg2Rad, 15*Deg2Rad, 15*Deg2Rad, 15*Deg2Rad, 15*Deg2Rad, 15*Deg2Rad, 15*Deg2Rad, 15*Deg2Rad, 15*Deg2Rad, 15*Deg2Rad])
```

The first entry is the maximum velocity of joint 1, so on and so forth. However, entries after 8th does not make sense because RAVEN only has 7 joints.

# General Workflow of Joint-level Control and Data recording

<p align="center">
  <img src="https://github.com/HaonanPeng/raven2_CRTK_Python_controller/blob/main/fig/exp_recording_workflow.png" width="750" height="369">
</p>

## Start RAVEN
For details about starting RAVEN, please refer to this [Quick Start Guide](https://github.com/HaonanPeng/raven2_CRTK_Python_controller/blob/main/doc/UW-BRL%20RAVEN%20Quick-Start%20Guide%20-%20Google%20Docs.pdf). **And please do not forget to manually move all the joint a little bit before pressing the silver button to 'bump the encoders'.**

First, normally start RAVEN by:
```
roslaunch raven_2 raven_2.launch
```

If you are also using external joint encoders that need to be calibrated during initialization, please do not press the silver button for now.

## Start External Joint Encoder
If you are using the external joint encoders, please go to 'joint_encoder/'  and run:
```
python run_r2_joint_encodersIII_pub.py
```
After running this, press the silver button to start RAVEN homing. The encoder code will allow 60 second for RAVEN homing. After this, the encoders will be calibrated according to the Mechanical limit that RAVEN has reached during homing.

After this, the calibrated external joint poses will be published to ROS topic 'ext_jpos'.

## Start ROS Bag Recording
First, make sure 'python_controller/r2_enc_CRTK_recorder.launch' is with raven2.launch in the RAVEN CRTK folder. Then, ROS bag recording can be started by:
```
roslaunch raven_2 r2_enc_CRTK_recorder.launch
```
This will record CRTK joint poses, ravenstate and external joint poses. The rate of ravenstate and CRTK will be toggled from 1000Hz to 100Hz to avoid huge ROS bags. Recording can be stopped by Ctrl + C

## Start Runtime Monitor
If real-time monitoring of joint poses and joint motion commands is needed, the Runtime Monitor of RAVEN can be started by:
```
python run_r2_runtime_monitor.py
```
However, please notice that the runtime monitor only has refreshing rate of ~0.6Hz. 

## Start Controlling RAVEN
There are multiple ways to control RAVEN with RAVEN CRTK Python Controller.

1) Keyboard controller
Manually controlling RAVEN based on joint-level, using keyboard:
```
python run_r2_keyboard_controller.py
```

2) Random joint movement
Let RAVEN joints moving randomly:
```
python run_r2_random_joint_movement.py
```

3) Trajectory following
Let RAVEN follow a pre-defined trajectory:
```
python run_r2_trajectory_follow.py
```

4) Go to a target joint pose
Let RAVEN go to a target joint pose:
```
python run_r2_goto.py
```
## Extract Data from ROS Bags
After you run the ROS bag recording, and complete the task, use Ctrl+C to stop ROS bag recording.
```
roslaunch raven_2 r2_enc_CRTK_recorder.launch
```
Then, find the recorded ROG bags and copy them to the workspace.
Next, run the bag reader:
```
python run_r2_bag_readers.py
```
It will extract recorded data in ROS bags to .csv file, where each line is a recorded message and the index of column can be found here:
https://docs.google.com/spreadsheets/d/1zIyyZu1IgiSwZPnXcyceeg1Pl0jn3FcNLhonxUrfP5c/edit?usp=sharing

# Manually Controlling RAVEN Using Mantis Leader Controller

<p align="center">
  <img src="https://github.com/HaonanPeng/raven2_CRTK_Python_controller/blob/main/fig/mantis_controller.jpg" width="633" height="476">
</p>


RAVEN can be teleoperated manually using the Mantis controller. In order to use this, first on the RAVEN PC, checkand record the local IP address by:
```
ifconfig
```
Then, normally start RAVEN by:
```
roslaunch raven_2 raven_2.launch
```

For details about starting RAVEN, please refer to this [Quick Start Guide](https://github.com/HaonanPeng/raven2_CRTK_Python_controller/blob/main/doc/UW-BRL%20RAVEN%20Quick-Start%20Guide%20-%20Google%20Docs.pdf). **And please do not forget to manually move all the joint a little bit before pressing the silver button to 'bump the encoders'.**

Then, on the Mantis PC, open the file on desktop named 'ipsettings.inf' and change one line with the RAVEN IP address. Next, turn on the power of Mantis and wait for the welcoming sound.

On the Mantis desktop, start "Mantis_Client.exe" and "GUI_Server.exe". Then you should see the cables of Mantis are tightened. Before using Mantis, the controllers need to be calibrated by holding it and double-cliking the button on the bottom.

<p align="center">
  <img src="https://github.com/HaonanPeng/raven2_CRTK_Python_controller/blob/main/fig/mantis_calibrating.png" width="633" height="476">
</p>

On the GUI, enter pin "123" and click "enter", you should see the following window. Please pay attention to the scale. Normally, it would be safer to start with a small scale such as 0.1.

<p align="center">
  <img src="https://github.com/HaonanPeng/raven2_CRTK_Python_controller/blob/main/fig/mantis_scale.png" width="633" height="476">
</p>

Then, go to "Engineer" tag and choose the IP adress that you just added. Check "Use ITP Reference Frame" and click "Start". If everything is fine, you should see the terminal that you start RAVEN on RAVEN PC is printing '.'. Then you should be able to control RAVEN using Mantis.

<p align="center">
  <img src="https://github.com/HaonanPeng/raven2_CRTK_Python_controller/blob/main/fig/mantis_change_ip_start.png" width="633" height="476">
</p>

Please also notice that if you choose "localhost" IP address 127.0.0.1, you could not control any real RAVEN, but instead, on Mantis PC, you can open "visualizaer.exe" to check the motion of the controller.

<p align="center">
  <img src="https://github.com/HaonanPeng/raven2_CRTK_Python_controller/blob/main/fig/mantis_local_visualizer.png"width="633" height="476">
</p>






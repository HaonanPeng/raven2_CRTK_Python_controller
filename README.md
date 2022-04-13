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

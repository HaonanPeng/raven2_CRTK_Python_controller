# raven2_CRTK_Python_controller
This is a controller for RAVEN II surgical robot, based on CRTK API, using Python

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

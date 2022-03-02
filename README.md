# raven2_CRTK_Python_controller
This is a controller for RAVEN II surgical robot, based on CRTK API, using Python

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

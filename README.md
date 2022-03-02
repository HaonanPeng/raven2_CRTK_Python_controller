# raven2_CRTK_Python_controller
This is a controller for RAVEN II surgical robot, based on CRTK API, using Python

# To use the keyboard controller:
First, normally start RAVEN by:

```
roslaunch raven_2 raven_2.launch
```

Then, in a new terminal, find the path of the 'python_controller' and start the keyboard controller by:

```
python run_r2_keyboard_controller.py
```

Please notice that the current version only allows one key input at a time for stability.

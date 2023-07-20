#!/bin/sh
python /home/supernova/raven2_CRTK_Python_controller/torque_controller/run_r2_multi_load_cell_force_pub.py &
python /home/supernova/raven2_CRTK_Python_controller/torque_controller/run_r2_force_control.py &
python /home/supernova/raven2_CRTK_Python_controller/torque_controller/run_r2_torque_control.py

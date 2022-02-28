# -*- coding: utf-8 -*-
"""
Created on Mon Dec  6 17:06:21 2021

@author: 75678
"""

import numpy as np
from pynput import keyboard
from pynput.keyboard import Key
import time
import utils

working = 1


utils.print_manu()
while working == 1:
    with keyboard.Events() as events:

        event = events.get(1e12)
        # Joint space control-----------------------------------------
        if event.key == keyboard.KeyCode.from_char('1'):
            utils.print_no_newline(" Moving: Joint 1 +++         ")
            
        elif event.key == keyboard.KeyCode.from_char('q'):
            utils.print_no_newline(" Moving: Joint 1 ---         ")

        elif event.key == keyboard.KeyCode.from_char('2'):
            utils.print_no_newline(" Moving: Joint 2 +++         ")
            
        elif event.key == keyboard.KeyCode.from_char('w'):
            utils.print_no_newline(" Moving: Joint 2 ---         ")

        elif event.key == keyboard.KeyCode.from_char('3'):
            utils.print_no_newline(" Moving: Joint 3 +++         ")
            
        elif event.key == keyboard.KeyCode.from_char('e'):
            utils.print_no_newline(" Moving: Joint 3 ---         ")

        elif event.key == keyboard.KeyCode.from_char('4'):
            utils.print_no_newline(" Moving: Joint 4 +++         ")
            
        elif event.key == keyboard.KeyCode.from_char('r'):
            utils.print_no_newline(" Moving: Joint 4 ---         ")

        elif event.key == keyboard.KeyCode.from_char('5'):
            utils.print_no_newline(" Moving: Joint 5 +++         ")
            
        elif event.key == keyboard.KeyCode.from_char('t'):
            utils.print_no_newline(" Moving: Joint 5 ---         ")

        elif event.key == keyboard.KeyCode.from_char('6'):
            utils.print_no_newline(" Moving: Joint 6 +++         ")
            
        elif event.key == keyboard.KeyCode.from_char('y'):
            utils.print_no_newline(" Moving: Joint 6 ---         ")

        elif event.key == keyboard.KeyCode.from_char('7'):
            utils.print_no_newline(" Moving: Joint 7 +++         ")
            
        elif event.key == keyboard.KeyCode.from_char('u'):
            utils.print_no_newline(" Moving: Joint 7 ---         ")
            
        # Cartisian space control----------------------------------------------
        elif event.key == Key.up:
            utils.print_no_newline(' Moving: Grasper forward    ')

        elif event.key == Key.down:
            utils.print_no_newline(' Moving: Grasper backward    ')

        elif event.key == Key.left:
            utils.print_no_newline(' Moving: Grasper left    ')

        elif event.key == Key.right:
            utils.print_no_newline(' Moving: Grasper right    ')

        #elif event.key == Key.page_up:
            #utils.print_no_newline(' Moving: Grasper up    ')

        #elif event.key == Key.page_down:
            #utils.print_no_newline(' Moving: Grasper down    ')
            
            



    

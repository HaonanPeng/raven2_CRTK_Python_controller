# -*- coding: utf-8 -*-
"""
Created on Mon Dec  6 18:22:45 2021

@author: 75678
"""
import sys

def print_manu():
    print('  ')
    print('-----------------------------------------')
    print('RAVEN 2 Keyboard Controller:')
    print('-----------------------------------------')
    print('[Exit]: 9')
    print('[Joint 1 +]: 1 | [Joint 1 -]: q')
    print('[Joint 2 +]: 2 | [Joint 2 -]: w')
    print('[Joint 3 +]: 3 | [Joint 3 -]: e')
    print('[Joint 4 +]: 4 | [Joint 4 -]: r')
    print('[Joint 5 +]: 5 | [Joint 5 -]: t')
    print('[Joint 6 +]: 6 | [Joint 6 -]: y')
    print('[Joint 7 +]: 7 | [Joint 7 -]: u')

    print('[Grasper open ]: o')
    print('[Grasper close ]: p')
    
    print('[Grasper farward ]: s')
    print('[Grasper backward]: x')
    print('[Grasper left    ]: z')
    print('[Grasper right   ]: c')
    print('[Grasper up      ]: f')
    print('[Grasper down    ]: v')

    print('[Grasper P -]: h')
    print('[Grasper P +]: n')
    print('[Grasper R -]: b')
    print('[Grasper R +]: m')
    print('[Grasper Y -]: g')
    print('[Grasper Y +]: j')

    print('-----------------------------------------')
    print('-----------------------------------------')
    print('Current command:')
    return None

def print_no_newline(string):
    # sys.stdout.write("\r" + '                               ')
    sys.stdout.write("\r" + string)
    sys.stdout.flush()
    return None

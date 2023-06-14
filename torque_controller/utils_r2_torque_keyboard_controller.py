# -*- coding: utf-8 -*-
"""
Created on Mon Dec  6 18:22:45 2021
@author: 75678
"""
import sys

def print_manu():
    print('  ')
    print('-----------------------------------------')
    print('RAVEN 2 Torque Keyboard Controller:')
    print('[WARNING!!!]: This is a test controller for RAVEN torque control')
    print('[WARNING!!!]: Do NOT use if you do not know what is going on.')
    print('-----------------------------------------')
    print('[Exit]: k')
    print('[Joint 1 +]: 1 | [Joint 1 -]: q')
    print('[Joint 2 +]: 2 | [Joint 2 -]: w')
    print('[Joint 3 +]: 3 | [Joint 3 -]: e')
    print('[Joint 4 +]: 4 | [Joint 4 -]: r')
    print('[Joint 5 +]: 5 | [Joint 5 -]: t')
    print('[Joint 6 +]: 6 | [Joint 6 -]: y')
    print('[Joint 7 +]: 7 | [Joint 7 -]: u')

    print('[Increase Torque 1]: a')
    print('[Decrease Torque 1]: z')
    print('[Increase Torque 2]: s')
    print('[Decrease Torque 2]: x')
    print('[Increase Torque 3]: d')
    print('[Decrease Torque 3]: c')
    print('[Increase Torque 4]: f')
    print('[Decrease Torque 4]: v')
    print('[Increase Torque 5]: g')
    print('[Decrease Torque 5]: b')
    print('[Increase Torque 6]: h')
    print('[Decrease Torque 6]: n')
    print('[Increase Torque 7]: j')
    print('[Decrease Torque 7]: m')

    
    print('[Joint 123+]: 8')
    print('[Joint 123-]: i')
    print('[Joint 456+]: 9')
    print('[Joint 456-]: o')
    print('[Joint 135+]: 0')
    print('[Joint 135-]: p')


    print('-----------------------------------------')
    print('-----------------------------------------')
    print('Current command:')
    return None

def print_no_newline(string):
    # sys.stdout.write("\r" + '                               ')
    sys.stdout.write("\r" + string)
    sys.stdout.flush()
    return None

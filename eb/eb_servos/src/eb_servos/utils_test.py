#!/usr/bin/env python3

#import sys
import os
import rospy
import logging


def min_clamp(value, min_value): # clamp pos and neg min_values
    return min(max(value, min_value), -min_value)
    
def max_clamp(value, max_value): # clamp pos and neg max_values
    return max(min(value, max_value), -max_value)



def utils_test():

    absmin = 2.5
    absmax = 3.5

    print("absmin = %0.2f, absmax = %0.2f" % (absmin , absmax ))
    for i in range(-40, 40):
        testvalue = i / 10.0
    
        minfoo = min_clamp(testvalue, absmin) 
        maxfoo = max_clamp(testvalue, absmax) 
        print("testvalue = %0.2f, min = %0.2f, max = %0.2f" % (testvalue, minfoo , maxfoo ))


    
if __name__=='__main__':

    utils_test()
    
        
        
        
    

#!/usr/bin/env python

import sys
import rospy
from stabil.srv import *

print "success!!!"

def servo_ping(x):
    rospy.wait_for_service('servo_server')
    try:
        servo_server = rospy.ServiceProxy('servo_server', ServoServer)
        resp1 = servo_server(x)
        return resp1.res
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 2:
        x = int(sys.argv[1])
    else:
        print usage()
        sys.exit(1)
    print("Requesting %s"%(x))
    print("%s begets %s"%(x, servo_ping(x)))

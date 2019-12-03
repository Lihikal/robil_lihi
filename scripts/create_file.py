# !/usr/bin/env python
import numpy as np

# import rospy
# from std_msgs.msg import String
import os
import csv


def create(a,b,c,d,e,f):


        with open('scores.csv', 'a') as csvfile:
            writer = csv.writer(csvfile, delimiter=',',
                                    quotechar='"', quoting=csv.QUOTE_MINIMAL)
            # if os.stat('scores.csv').st_size == 0:
            #     writer.writerow([soil_types])
            writer.writerow([a, b, c, d, e, f])
        csvfile.close()









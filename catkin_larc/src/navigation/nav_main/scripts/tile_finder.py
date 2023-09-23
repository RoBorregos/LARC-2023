#!/usr/bin/env python3
#ros service that return a tuple with the robot current position
#needs to be suscribed to the color detection topic and line finder topic

import rospy
from nav_main.msg import tile

class TileFinder:
    def __init__(self):
        # publishers
        self.pubTile = rospy.Publisher('/tile', tile, queue_size=10)



if __name__ == '__main__':
    rospy.init_node('tile_finder', anonymous=True)
    tile_finder = TileFinder()
    rospy.spin()
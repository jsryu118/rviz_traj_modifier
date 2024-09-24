#!/usr/bin/env python3

import rospy
from publish_track import PublishTrack 

def main():
    rospy.init_node("track_info_node")
    PublishTrack()

if __name__ == '__main__':
    main()
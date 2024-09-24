#!/usr/bin/env python3

import os
import shutil

smooth_var=10


global update
update = False

class ManageTrackHistory():
    def __init__(self,PublishTrack):
        file_path = os.path.abspath(__file__)
        current_script_path = os.path.dirname(file_path)
        directory = current_script_path + '/gui_tmp'
        if os.path.exists(directory):
            shutil.rmtree(directory)
            print("Remove existing tmp folder: ",directory)
        os.makedirs(directory)
        self.name_p1revious = directory + '/p1revious_track.txt'
        self.name_p2revious = directory + '/p2revious_track.txt'
        self.name_p3revious = directory + '/p3revious_track.txt'
        self.name_p4revious = directory + '/p4revious_track.txt'
        self.name_p5revious = directory + '/p5revious_track.txt'        
        self.name_current = directory + '/current_track.txt'
        self.name_modified = current_script_path + '/result/subsequential_track.txt'
        self.PublishTrack = PublishTrack
        self.track_save_flag = False
        


    def save_track(self,marker_array):
        print("Start saving track...")
        self.shift_saved_track_forward()
        with open(self.name_current, 'w') as file:
        # Loop through each marker in the array
            for marker in marker_array.markers:
                # print(markers.pose.orientation.w)
                # Write the pose information to a new line in the file
                file.write('{}, {}, {}, {}, {}, {}, {}, {},{},{}\n'.format(
                    marker.pose.position.x, marker.pose.position.y, marker.pose.position.z,
                    marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z, marker.pose.orientation.w,
                    marker.color.r, marker.color.g, marker.color.b))
        self.PublishTrack.track = self.PublishTrack.read_file(self.name_current,0,"normal")
        self.PublishTrack.track_adv = self.PublishTrack.read_file(self.name_current,0,"adv")
        self.PublishTrack.lookahead_onoff = self.PublishTrack.read_file(self.name_current,0,"lookahead")
        self.PublishTrack.vel_onoff = self.PublishTrack.read_file(self.name_current,0,"vel")
        shutil.copy2(self.name_current,self.name_modified)
        print("End saving track...")

    def control_z_track(self):
        directory = self.name_p1revious
        if not os.path.exists(directory):
            print('\033[91m'+ "can not load last track more than 3 times!!"+'\033[0m')
            print('\033[91m'+ "can not load last track more than 3 times!!"+'\033[0m')
            print('\033[91m'+ "can not load last track more than 3 times!!"+'\033[0m')
            print('\033[91m'+ "can not load last track more than 3 times!!"+'\033[0m')
            return
        self.shift_saved_track_backward()
        self.PublishTrack.track = self.PublishTrack.read_file(self.name_current,0,"normal")
        self.PublishTrack.track_adv = self.PublishTrack.read_file(self.name_current,0,"adv")
        self.PublishTrack.lookahead_onoff = self.PublishTrack.read_file(self.name_current,0,"lookahead")
        self.PublishTrack.vel_onoff = self.PublishTrack.read_file(self.name_current,0,"vel")
        
        self.PublishTrack.publish_interactive_marker()
        shutil.copy2(self.name_current,self.name_modified)

    def shift_saved_track_forward(self):
        self.rename_txt(self.name_p4revious,self.name_p5revious)
        self.rename_txt(self.name_p3revious,self.name_p4revious)
        self.rename_txt(self.name_p2revious,self.name_p3revious)
        self.rename_txt(self.name_p1revious,self.name_p2revious)
        self.rename_txt(self.name_current,self.name_p1revious)

    def shift_saved_track_backward(self):
        self.rename_txt(self.name_p1revious,self.name_current)
        self.rename_txt(self.name_p2revious,self.name_p1revious)
        self.rename_txt(self.name_p3revious,self.name_p2revious)
        self.rename_txt(self.name_p4revious,self.name_p3revious)
        self.rename_txt(self.name_p5revious,self.name_p4revious)
       
    def rename_txt(self,src, dst):
        if os.path.exists(src):
            os.rename(src,dst)
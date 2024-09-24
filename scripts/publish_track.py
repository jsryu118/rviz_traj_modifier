#!/usr/bin/env python3

from manage_history import ManageTrackHistory
from pop_up_display import PopUpDisplay
from track_modification_tools import *
import rospy

import shutil
import os
import numpy as np

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from geometry_msgs.msg import *




class PublishTrack():
    def __init__(self):
        self.track_manager=ManageTrackHistory(self)
        self.pop_up_display=PopUpDisplay(self)

        self.server = InteractiveMarkerServer("track_info_interactive")
        self.menu_handler = MenuHandler()

        self.main_menu_list = ["pose_smooth","vel_smooth","Anchor1","Anchor2","Pos_Straighten","Vel_Straighten","Vel_Offset","Vel_Weight","Vel_Set","Lookahead_Set","int_idx","Control-Z","Update","OnOFF","entire_xy","entire_rotation"]
        self.sub_menu_list = ["10","20","30","idx"]
        self.onoff_list = ["Lookahead","Vel"]

        for main in self.main_menu_list:
            self.menu_handler.insert( main , callback=self.processFeedback)
        for i in range(2):
            for sub in self.sub_menu_list:
                self.menu_handler.insert( sub , parent=i+1, callback=self.processFeedback)
        for menu in self.onoff_list:
            self.menu_handler.insert(menu, parent=self.main_menu_list.index("OnOFF")+1, callback=self.processFeedback)
        

        self.wp_sampling=np.nan  
        self.scale = 1.0
        self.update =False
        self.update_ey=False
        self.YAW =[]
        self.anchor1 = np.nan
        self.anchor2 = np.nan
        self.vel_smooth_pivot = np.nan
        self.pos_smooth_pivot = np.nan
        self.Vel_Offset_pivot = np.nan
        self.max_id = np.nan
        self.last_sampled_int_marker = None
        self.prev_int_makrer = InteractiveMarker()
        self.prev_marker = Marker()
  
        self.cst_pub = rospy.Publisher("track_info", MarkerArray, queue_size = 1)
        self.cst2_pub = rospy.Publisher("track_adv", MarkerArray, queue_size = 1)
        self.track_bool = rospy.Publisher("track_bool", MarkerArray, queue_size = 1)
        self.Lookahead_OnOff = rospy.Publisher("Lookahead_OnOff", MarkerArray, queue_size = 1)
        self.Vel_OnOff = rospy.Publisher("Vel_OnOff", MarkerArray, queue_size = 1)
        

        self.interactive_marker_sub = rospy.Subscriber("/track_info_interactive/update_full",InteractiveMarkerInit,self.interactiveMarkerCallback)

        file_name1 = rospy.get_param('/pub_track_gui/filename','subsequential_track.txt')
        file_path = os.path.abspath(__file__)
        self.current_script_path = os.path.dirname(file_path)
        file_name = self.current_script_path + '/result/'+ file_name1
        print(file_name)
        self.track = self.read_file(file_name,0,"normal")
        self.track_adv = self.read_file(file_name,0,"adv")
        self.lookahead_onoff = self.read_file(file_name,0,"lookahead")
        self.vel_onoff = self.read_file(file_name,0,"vel")
        self.track_len = len(np.loadtxt(file_name, delimiter=",", dtype = float))
        shutil.copy2(file_name,self.track_manager.name_current)
        self.publish_interactive_marker()

        inner_filename=self.current_script_path + "/result/innerwall.txt"
        innerwall = np.loadtxt(inner_filename, delimiter=",", dtype = float)
        outer_filename=self.current_script_path + "/result/outerwall.txt"
        outerwall = np.loadtxt(outer_filename, delimiter=",", dtype = float)

        self.wall_fining_dis = 0.1
        self.cal_ey_threshold =0.06 # minimun distance between two close wayponit

        self.innerwall=fined_wall(innerwall,self.wall_fining_dis)
        self.outerwall=fined_wall(outerwall,self.wall_fining_dis)






        self.rate = rospy.Rate(1)
        
        while not rospy.is_shutdown():
            # self.track_adv=self.track
            self.cst_pub.publish(self.track)
            self.cst2_pub.publish(self.track_adv)
            self.Lookahead_OnOff.publish(self.lookahead_onoff)
            # print(self.lookahead_onoff.markers[299].color.g)
            self.Vel_OnOff.publish(self.vel_onoff)
                
            if self.update:
                self.track_bool.publish(self.track)
                print("Update finished")
                self.update=False
            # self.cst_pub2.publish(self.track1)
            # self.cst_pub3.publish(self.track2)
            # self.cst_pub4.publish(self.track3)
            # self.cst_pub5.publish(self.track4)
            self.rate.sleep()

            self.server.applyChanges()        

    def read_file(self, filename, id,target):
        track = np.loadtxt(filename, delimiter=",", dtype = float)
        
        track_markers = MarkerArray()
        for i in range(len(track)):
            # rosmsg info visualization_msgs/Marker
            track_marker = Marker()
            track_marker.header.frame_id = "map"  
            track_marker.header.stamp = rospy.Time.now()
            track_marker.ns = "track"
            track_marker.id = id
            track_marker.type = Marker.SPHERE
            track_marker.action = Marker.ADD          
            
            track_marker.pose.position.x = track[i,0] #position x
            track_marker.pose.position.y = track[i,1] #posiiton y
            track_marker.pose.position.z = track[i,2] #velocity
            track_marker.pose.orientation.x = track[i,3] #curvature
            track_marker.pose.orientation.y = track[i,4] #left_width
            track_marker.pose.orientation.z = track[i,5] #right_width
            track_marker.pose.orientation.w = track[i,6] #psi, yaw
            # track_marker.colors=[int(track[i,3]), int(track[i,4]), int(track[i,5]),0]
            # a=Point()
            # a.x=track[i,3]
            # a.y=track[i,3]
            # a.z=track[i,3]
            # track_marker.points=[a]
            # if i==0:
            #     print(track_marker.points)
            # print(target)
            if target=="adv":
                # track_marker.color.r =1
                # track_marker.color.g =(max(track[:,2])-track[i,2])/(max(track[:,2])-min(track[:,2]))
                # track_marker.color.b =(max(track[:,2])-track[i,2])/(max(track[:,2])-min(track[:,2]))             
                track_marker.color.r =1-track[i,2]/max(track[:,2])
                track_marker.color.g =1-abs((track[i,2]/max(track[:,2]))-0.5)*2
                track_marker.color.b =track[i,2]/max(track[:,2])
            elif target=="lookahead":
                # print("LH")
                track_marker.color.r =1
                try:
                    track_marker.color.g =1-track[i,8]
                except:
                    track_marker.color.g =1
                
                track_marker.color.b =1
            
            elif target=="vel":
                track_marker.color.r =1
                track_marker.color.g =1
                
                try:
                    track_marker.color.b =1-track[i,9]
                except:
                    track_marker.color.b =1
                
            else:
                track_marker.color.r = track[i,7] # #lookahead
                if len(track[i]) ==10:
                    track_marker.color.g =track[i,8] #lookahead onoff
                    track_marker.color.b =track[i,9] #vel onoff
                else:
                    track_marker.color.g= 0
                    track_marker.color.b= 0 
            track_marker.color.a = 1
            track_marker.scale.x = 0.2
            track_marker.scale.y = 0.2
            track_marker.scale.z = 0.2
            track_markers.markers.append(track_marker)
            id += 1
        
        return track_markers
        
    def interactiveMarkerCallback(self,msg):
        list_xy=[]
        for int_marker in msg.markers:
            list_xy.append([int_marker.pose.position.x, int_marker.pose.position.y])
        # print(list_xy)
        if list_xy:
            cur=calc_curv(list_xy)
        # print(cur)
        marker_array = MarkerArray()
        cur_idx=0
        for int_marker in msg.markers:
            
            marker = Marker()
            marker.header = int_marker.header
            marker.header.frame_id = "map"
            marker.id = int_marker.controls[0].markers[0].id
            marker.pose = int_marker.pose
            ######
            # marker.pose.orientation.x = self.track.markers[marker.id].pose.orientation.x #curvature
            marker.pose.orientation.x = cur[cur_idx]
            # print(cur[cur_idx])
            # print(self.track.markers[marker.id].pose.orientation.x-cur[cur_idx])
            ######
            marker.pose.orientation.y = self.track.markers[marker.id].pose.orientation.y #left_width
            marker.pose.orientation.z = self.track.markers[marker.id].pose.orientation.z #right_width
            if self.update_ey:
                inner_ey=calculate_ey(self, marker.pose.position,self.YAW[cur_idx],True,True , self.cal_ey_threshold)
                outer_ey=calculate_ey(self, marker.pose.position,self.YAW[cur_idx],False,True,self.cal_ey_threshold)
                # if driving direction change from counterclock to clock wise, swith True and False value.
                if inner_ey<100:
                    marker.pose.orientation.y = inner_ey #left_width
                if outer_ey<100:    
                    marker.pose.orientation.z = outer_ey #right_width
                
            #########
            marker.pose.orientation.w = self.YAW[cur_idx] #psi, yaw         
            ############
            marker.ns= int_marker.controls[0].markers[0].ns
            marker.type = int_marker.controls[0].markers[0].type
            marker.action = int_marker.controls[0].markers[0].action
            marker.scale = int_marker.controls[0].markers[0].scale
            marker.color = int_marker.controls[0].markers[0].color
            # print(marker.color.g,marker.color.b)
            marker.color.a = 1
            marker_array.markers.append(marker)
            cur_idx += 1
        

        if self.track_manager.track_save_flag == True:
            self.track_manager.save_track(marker_array)
            if self.update_ey:
                self.update=True
                self.update_ey=False
            self.publish_interactive_marker()
            self.track_manager.track_save_flag = False
    
    def processFeedback(self,feedback):
        # print(feedback.menu_entry_id)
        p = feedback.pose.position
        if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            if feedback.menu_entry_id <= len(self.main_menu_list):
                selected_menu = self.main_menu_list[(feedback.menu_entry_id-1)]
                print(feedback.marker_name + ": menu item " + selected_menu + " clicked" )
                if selected_menu == "Anchor1":
                    self.anchor1 = feedback.marker_name
                elif selected_menu == "Anchor2":
                    self.anchor2 = feedback.marker_name
                elif selected_menu == "Vel_Straighten":
                    self.vel_smooth_pivot = feedback.marker_name
                    smooth_path_z(self,self.anchor1,self.vel_smooth_pivot)
                    smooth_path_z(self,self.anchor2,self.vel_smooth_pivot)
                    smooth_path_z(self,self.anchor1,self.vel_smooth_pivot)
                    smooth_path_z(self,self.anchor2,self.vel_smooth_pivot)
                    smooth_path_z(self,self.anchor1,self.vel_smooth_pivot)
                    smooth_path_z(self,self.anchor2,self.vel_smooth_pivot)
                    print("smooth vel finished")
                    self.track_manager.track_save_flag = True
                    
                elif selected_menu == "Pos_Straighten":
                    self.pos_smooth_pivot = feedback.marker_name
                    smooth_path_xy(self,self.anchor1,self.pos_smooth_pivot)
                    smooth_path_xy(self,self.anchor2,self.pos_smooth_pivot)
                    smooth_path_xy(self,self.anchor2,self.pos_smooth_pivot)
                    smooth_path_xy(self,self.anchor1,self.pos_smooth_pivot)
                    smooth_path_xy(self,self.anchor2,self.pos_smooth_pivot)
                    print("smooth pos finished")
                    self.track_manager.track_save_flag = True                
                elif selected_menu == "Vel_Offset":
                    self.pop_up_display.show_input_dialog("Vel_Offset")
                    user_input = self.pop_up_display.user_input
                    Vel_Offset(self, feedback.marker_name, user_input)
                    self.pop_up_display.user_input = 0.0
                    print("vel offset finished")
                    self.track_manager.track_save_flag = True
                elif selected_menu == "Lookahead_Set":
                    self.pop_up_display.show_input_dialog("Lookahead_Set")
                    user_input = self.pop_up_display.user_input
                    set_lookahead(self,self.anchor1, self.anchor2, user_input)
                    self.pop_up_display.user_input = 0.0
                    print("set lookahead finished")
                    self.track_manager.track_save_flag = True
                elif selected_menu == "Vel_Set":
                    self.pop_up_display.show_input_dialog("Vel_Set")
                    user_input = self.pop_up_display.user_input
                    Vel_Set(self, self.anchor1, self.anchor2, user_input)
                    self.pop_up_display.user_input = 0.0
                    print("Vel set finished")
                    self.track_manager.track_save_flag = True
                elif selected_menu == "Vel_Weight":
                    self.pop_up_display.show_input_dialog("Vel_Weight")
                    user_input = self.pop_up_display.user_input
                    Vel_Weight(self, self.anchor1, self.anchor2, user_input)
                    self.pop_up_display.user_input = 0.0
                    print("Vel Weight finished")
                    self.track_manager.track_save_flag = True
                elif selected_menu == "int_idx":
                    self.pop_up_display.INTEGER=True
                    self.pop_up_display.show_input_dialog("int_dix")
                    if self.pop_up_display.user_input is None:
                        print("something wrong..")
                    else:
                        user_input = self.pop_up_display.user_input
                        self.wp_sampling=user_input
                        self.pop_up_display.user_input = None
                        self.publish_interactive_marker()
                        self.pop_up_display.INTEGER=False

                elif selected_menu == "Control-Z":
                    self.track_manager.control_z_track()
                    print("load last track finished")
                elif selected_menu == "Update":
                    
                    self.update_ey = True
                    self.menu_handler.reApply( self.server )
                    self.server.applyChanges()
                    self.track_manager.track_save_flag = True
                elif selected_menu == "entire_xy":
                    entire_traj_transform(self, feedback.marker_name)
                    print("entire trajectory transformation finished")
                    self.track_manager.track_save_flag = True
                elif selected_menu == "entire_rotation":
                    entire_traj_rotation(self, self.anchor1, self.anchor2)
                    print("entire trajectory rotation finished")
                    self.track_manager.track_save_flag = True
                                
                else:
                    print("something wrong..")
                    exit()
                
            elif feedback.menu_entry_id > len(self.main_menu_list) and feedback.menu_entry_id <=len(self.main_menu_list)+2*len(self.sub_menu_list):

                smooth_var = 10
                a=(feedback.menu_entry_id - len(self.main_menu_list)-1)//4
                b=(feedback.menu_entry_id - len(self.main_menu_list)-1)%4
                # print(b)
                if b==3:
                    # print("idx")
                    self.pop_up_display.show_input_dialog("Smooth length")
                    smooth_var = int(self.pop_up_display.user_input)
                    
                c=[10,20,30,smooth_var]
                
                if a==0:
                    sampleCubicSplinesWithDerivative(self,feedback.marker_name,c[b],"Pose" ,self.scale)
                    print("Pose_Smooth finished")
                    self.track_manager.track_save_flag = True
                elif a==1:
                    sampleCubicSplinesWithDerivative(self, feedback.marker_name,c[b],"Vel" ,self.scale)
                    print("Vel_Smooth finished")
                    self.track_manager.track_save_flag = True
            else:        
                # print(feedback.menu_entry_id)
                self.pop_up_display.ONOFF=True
                self.pop_up_display.show_input_dialog("1(ON) OR 0(OFF)")
                user_input = self.pop_up_display.user_input
                if feedback.menu_entry_id ==25:
                    OnOff(self, self.anchor1, self.anchor2, user_input, "Lookahead")
                else:
                    OnOff(self, self.anchor1, self.anchor2, user_input, "Velocity")
                self.pop_up_display.user_input = 0.0
                print("ON/OFF finished")
                self.track_manager.track_save_flag = True
                self.pop_up_display.ONOFF=False
                
                

        # elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
        #     print(feedback.marker_name + " is now at " + str(p.x) + ", " + str(p.y) + ", " + str(p.z) )
        
        # # if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
        # #     self.track_manager.track_save_flag = True

        # return


    def CreateMarkerControl(self, interaction_marker, interaction_mode, name, w,x,y,z):
        track_marker_control = InteractiveMarkerControl()
        track_marker_control.always_visible = True
        track_marker_control.name = name
        track_marker_control.orientation.w = w
        track_marker_control.orientation.x = x
        track_marker_control.orientation.y = y
        track_marker_control.orientation.z = z
        track_marker_control.interaction_mode = interaction_mode
        interaction_marker.controls.append(track_marker_control) 

    def publish_interactive_marker(self):
        id = 0
        self.YAW=[]
        for i in range(self.track_len):
            marker=self.track.markers[i]
            
        # for marker in self.track.markers:
            int_marker = InteractiveMarker()
            int_marker.header.frame_id = "map"
            int_marker.name ="wp"+str(id)
            # if id%wp_sampling ==0:
            #     int_marker.description = str(id)
            int_marker.header.stamp = rospy.Time.now()
            int_marker.scale = 0.5 # size of arrow
            int_marker.pose.position.x = marker.pose.position.x #position x
            int_marker.pose.position.y = marker.pose.position.y #posiiton y
            int_marker.pose.position.z = marker.pose.position.z #velocity
            # if i==0:
            #     print(self.distance_between_int_markers(int_marker,self.last_sampled_int_marker))
            if np.isnan(self.wp_sampling):
                if distance_between_int_markers(int_marker,self.last_sampled_int_marker) > 1.0:
                    sampling = True
                    self.last_sampled_int_marker = int_marker
                    # int_marker.description = "wp: "+str(id)+"\nV:"+str(marker.pose.position.z)+"\nL:"+str(marker.color.r)
                    int_marker.description = "wp: {}\nV: {:.2f}\nL: {:.2f}".format(id, marker.pose.position.z, marker.color.r)
            else:
                if i%self.wp_sampling==0:
                    sampling=True
                    int_marker.description = "wp: {}\nV: {:.2f}\nL: {:.2f}".format(id, marker.pose.position.z, marker.color.r)
                    
            # if distance_between_int_markers(int_marker,self.last_sampled_int_marker) > 0.5:
            #     sampling = True
            #     self.last_sampled_int_marker = int_marker
            #     # int_marker.description = "wp: "+str(id)+"\nV:"+str(marker.pose.position.z)+"\nL:"+str(marker.color.r)
            #     int_marker.description = "wp: {}\nV: {:.2f}\nL: {:.2f}".format(id, marker.pose.position.z, marker.color.r)

            # print("debug", marker.pose.orientation.w)
            # yaw = marker.pose.orientation.w
            vector = cal_slope(self.track.markers[(i-1)%self.track_len],marker,self.track.markers[(i+1)%self.track_len])
            # print(vector)
            yaw=cal_yaw(vector)
            ####################### yaw값을 업데이트 해줘야한다
            qx = 0.0
            qy = 0.0
            qz = np.sin(yaw / 2.0)
            qw = np.cos(yaw / 2.0)
            norm = np.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
            qx /= norm
            qy /= norm
            qz /= norm
            qw /= norm
            int_marker.pose.orientation.x = qx
            int_marker.pose.orientation.y = qy
            int_marker.pose.orientation.z = qz
            int_marker.pose.orientation.w = qw

            # # visualize marker
            track_marker_control = InteractiveMarkerControl()
            track_marker_control.always_visible = True
            track_marker = Marker()
            track_marker.id = id
            # track_marker.ns = "track"
            track_marker.header.stamp = rospy.Time.now()
            track_marker.type = Marker.SPHERE
            track_marker.ns = "track"
            track_marker.color.r = marker.color.r # lookahead
            track_marker.color.g = marker.color.g
            track_marker.color.b = marker.color.b
            if sampling:
                track_marker.color.a = 0
            else:
                 track_marker.color.a = 0
            track_marker.scale.x = 0.2
            track_marker.scale.y = 0.2
            track_marker.scale.z = 0.2
            self.max_id = id

            track_marker_control.markers.append(track_marker)
            self.YAW.append(yaw)
            int_marker.controls.append(track_marker_control)
            # if id%wp_sampling ==0:
            #     # move_x
            #     self.CreateMarkerControl(int_marker,InteractiveMarkerControl.MOVE_AXIS,"move_x",1,1,0,0)
            #     # move_y
            #     self.CreateMarkerControl(int_marker,InteractiveMarkerControl.MOVE_AXIS,"move_y",1,0,0,1)
            #     # move_z
            #     self.CreateMarkerControl(int_marker,InteractiveMarkerControl.MOVE_AXIS,"move_z",1,0,1,0)
            #     # menu
            #     self.CreateMarkerControl(int_marker,InteractiveMarkerControl.BUTTON,"menu",0,0,0,1)

            # self.server.insert(int_marker, self.processFeedback)
            # self.menu_handler.apply( self.server, int_marker.name )
            # self.track_adv.markers[id] = copy.deepcopy(track_marker)
            if sampling:
                self.CreateMarkerControl(int_marker,InteractiveMarkerControl.MOVE_AXIS,"move_x",1,1,0,0) # create move_x arrow
                self.CreateMarkerControl(int_marker,InteractiveMarkerControl.MOVE_AXIS,"move_y",1,0,0,1) # create move_y arrow
                self.CreateMarkerControl(int_marker,InteractiveMarkerControl.MOVE_AXIS,"move_z",1,0,1,0) # create move_z arrow
                # self.CreateMarkerControl(int_marker,InteractiveMarkerControl.BUTTON,"menu",0,0,0,1)# menu
            self.server.insert(int_marker, self.processFeedback)
            self.menu_handler.apply( self.server, int_marker.name )
            
            # if self.distance_between_int_markers(self.last_sampled_int_marker,self.prev_int_makrer) > 1.5:
            #     self.prev_marker.color.a =1
            #     self.prev_int_makrer.description= "wp: "+str(id-1)+"\nL:"+str(self.prev_marker.color.r)
            #     self.CreateMarkerControl(self.prev_int_makrer,InteractiveMarkerControl.MOVE_AXIS,"move_x",1,1,0,0) # move_x
            #     self.CreateMarkerControl(self.prev_int_makrer,InteractiveMarkerControl.MOVE_AXIS,"move_y",1,0,0,1) # move_y
            #     self.CreateMarkerControl(self.prev_int_makrer,InteractiveMarkerControl.MOVE_AXIS,"move_z",1,0,1,0) # move_z
            #     self.CreateMarkerControl(self.prev_int_makrer,InteractiveMarkerControl.BUTTON,"menu",0,0,0,1)# menu

            self.prev_int_makrer = int_marker
            self.prev_marker = marker
            sampling = False
            id += 1
        self.server.applyChanges()
        self.last_sampled_int_marker = None
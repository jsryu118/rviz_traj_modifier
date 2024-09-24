#!/usr/bin/env python3

import numpy as np
from scipy import interpolate

from visualization_msgs.msg import *
from geometry_msgs.msg import *


def is_pivots_nan(pivot1,pivot2):
    if (1-np.isnan(pivot1))*(1-np.isnan(pivot2)):
        return False
    else:
        return True

def fined_wall(wall, min_dis):
    new_list=[]
    p=0
    for i in range(len(wall)):
        j = (i+1)%len(wall)
        dis_x= wall[j,0]-wall[i,0]
        dis_y= wall[j,0]-wall[i,0]
        dis=np.sqrt(dis_x**2+dis_y**2)
        if dis >min_dis:
            for k in range (int(dis//min_dis)):
                new_point=[wall[i,0]+dis_x*(k+1)/(dis//min_dis+1),wall[i,1]+dis_y*(k+1)/(dis//min_dis+1)]
                new_index = i+k+p+1
                new_list.append([new_point,new_index])
                # new_list.append([[wall[i,0]+dis_x*(k+1)/(dis//min_dis+1),wall[i,1]+dis_y*(k+1)/(dis//min_dis+1)],i+k+p+1])
            p+=int(dis//min_dis)

    refined_wall=insert_new_points(wall,new_list)

    return refined_wall

def insert_new_points(wall , new_list):
    # print(len(wall))
    for i in range(len(new_list)):
        wall = np.insert(wall, new_list[i][1] ,new_list[i][0],axis=0)
    return wall

def calc_curv(traj):
    traj=np.array(traj)
    # print(len(traj[:,0]))
    dx = np.gradient(traj[:,0])
    dy = np.gradient(traj[:,1])    
    # print(len(dx))
    d2x = np.gradient(dx)
    d2y = np.gradient(dy)
    # print(len(d2x))
    curvature = (dx * d2y - d2x * dy) / (dx * dx + dy * dy)**1.5
    # print(curvature[0])
    return curvature

def calculate_ey(pub_track, position, yaw, is_inner, is_counterclock,threshold):
    x=position.x
    y=position.y
    sign=(2*is_inner-1)*(2*is_counterclock-1)
    rot_yaw=yaw+sign*np.pi/2
    ey=1000.0
    if is_inner:
        wall=pub_track.innerwall
        for i in range(len(wall)):
            dot=np.cos(rot_yaw)*(wall[i,0]-x)+np.sin(rot_yaw)*(wall[i,1]-y)
            ver_dis = np.abs(np.cos(rot_yaw)*(wall[i,1]-y)+np.sin(rot_yaw)*(wall[i,0]-x))
            # angle=np.arccos(dot/np.sqrt((wall[i,0]-x)**2+(wall[i,1]-y)**2))
            if (ver_dis<threshold and ey>np.abs(dot)):
                ey=np.abs(dot)
    else:
        wall=pub_track.outerwall
        for i in range(len(wall)):
            dot=np.cos(rot_yaw)*(wall[i,0]-x)+np.sin(rot_yaw)*(wall[i,1]-y)
            ver_dis = np.abs(np.cos(yaw)*(wall[i,0]-x)+np.sin(yaw)*(wall[i,1]-y))
            # angle=np.arccos(dot/np.sqrt((wall[i,0]-x)**2+(wall[i,1]-y)**2))
            if (dot>0 and ver_dis<threshold and ey>np.abs(dot)):
                ey=np.abs(dot)
    # min_angle=np.pi
    # for i in range(len(wall)):
    #     dot=np.cos(rot_yaw)*(wall[i,0]-x)+np.sin(rot_yaw)*(wall[i,1]-y)
    #     angle=np.arccos(dot/np.sqrt((wall[i,0]-x)**2+(wall[i,1]-y)**2))
    #     if angle<min_angle:
    #         min_angle=angle
    #         ey=np.abs(dot)
    return ey




def path_contain_zero_wp(m1_id, m2_id ,max_id):
    diff = abs(m1_id - m2_id)
    if diff > 0.5*max_id:
        return True
    else:
        return False
    
def smooth_path_xy(pub_track, marker1_name, marker2_name):
    m1_p = pub_track.server.get(marker1_name).pose.position
    m2_p = pub_track.server.get(marker2_name).pose.position
    m1_p = np.array([m1_p.x, m1_p.y])
    m2_p = np.array([m2_p.x, m2_p.y])
    m1_id = int(marker1_name[2:])
    m2_id = int(marker2_name[2:])
    direction = m2_p - m1_p
    distance = np.linalg.norm(direction)
    if distance ==0:
        unit_vector = 0
    else:
        unit_vector = direction / distance
    
    if path_contain_zero_wp(m1_id, m2_id, pub_track.max_id):
        step_size = distance / (pub_track.max_id - abs(m1_id - m2_id))
        if m1_id < m2_id:
            tmp1 = range(m1_id, -1,-1)
            tmp2 = range(pub_track.max_id,m2_id-1,-1)
            smooth_range = list(tmp1)+list(tmp2)
        else:
            tmp1 = range(m2_id,-1,-1)
            tmp2 = range(pub_track.max_id,m1_id-1,-1)
            smooth_range = list(tmp1)+list(tmp2)
    else:
        if m1_id -m2_id ==0:
            step_size = 0
        else:
            step_size = distance/ abs(m1_id - m2_id)
        if m1_id < m2_id:
            smooth_range = range(m1_id, m2_id+1,1)
        else:
            smooth_range = range(m1_id, m2_id-1,-1)

    for i in smooth_range:
        if path_contain_zero_wp(i,m1_id,pub_track.max_id):
            offset = (pub_track.max_id - abs(i-m1_id)) * step_size
        else:
            offset = abs(i-m1_id) * step_size

        position = tuple(m1_p + offset*unit_vector)
        marker_name = "wp"+str(i)
        ori_pose = pub_track.server.get(marker_name).pose
        new_pose = Pose()
        new_pose.position.x = position[0]
        new_pose.position.y = position[1]
        new_pose.position.z = ori_pose.position.z
        new_pose.orientation = ori_pose.orientation
        pub_track.server.setPose(marker_name, new_pose)

    pub_track.menu_handler.reApply( pub_track.server )
    pub_track.server.applyChanges()
    return

def smooth_path_z(pub_track, marker1_name, marker2_name):
    m1_p = pub_track.server.get(marker1_name).pose.position
    m2_p = pub_track.server.get(marker2_name).pose.position
    m1_p = np.array([m1_p.z])
    m2_p = np.array([m2_p.z])
    m1_id = int(marker1_name[2:])
    m2_id = int(marker2_name[2:])
    direction = m2_p - m1_p
    distance = np.linalg.norm(direction)
    if distance ==0:
        unit_vector = 0
    else:
        unit_vector = direction / distance
    
    if path_contain_zero_wp(m1_id, m2_id,pub_track.max_id):
        step_size = distance / (pub_track.max_id - abs(m1_id - m2_id))
        if m1_id < m2_id:
            tmp1 = range(m1_id, -1,-1)
            tmp2 = range(pub_track.max_id,m2_id-1,-1)
            smooth_range = list(tmp1)+list(tmp2)
        else:
            tmp1 = range(m2_id,-1,-1)
            tmp2 = range(pub_track.max_id,m1_id-1,-1)
            smooth_range = list(tmp1)+list(tmp2)
    else:
        if m1_id -m2_id ==0:
            step_size = 0
        else:
            step_size = distance/ abs(m1_id - m2_id)
        if m1_id < m2_id:
            smooth_range = range(m1_id, m2_id+1,1)
        else:
            smooth_range = range(m1_id, m2_id-1,-1)

    for i in smooth_range:
        if path_contain_zero_wp(i,m1_id,pub_track.max_id):
            offset = (pub_track.max_id - abs(i-m1_id)) * step_size
        else:
            offset = abs(i-m1_id) * step_size

        position = tuple(m1_p + offset*unit_vector)
        marker_name = "wp"+str(i)
        ori_pose = pub_track.server.get(marker_name).pose
        new_pose = Pose()
        new_pose.position.x = ori_pose.position.x
        new_pose.position.y = ori_pose.position.y
        new_pose.position.z = position[0]
        new_pose.orientation = ori_pose.orientation
        pub_track.server.setPose(marker_name, new_pose)

    pub_track.menu_handler.reApply( pub_track.server )
    pub_track.server.applyChanges()
    return

def Vel_Offset(pub_track, marker_name, user_input):
    ori_pose = pub_track.server.get(marker_name)
    ori_pose.pose.position.z = ori_pose.pose.position.z + user_input     
    pub_track.menu_handler.reApply( pub_track.server )
    pub_track.server.applyChanges()
    return

def Vel_Set(pub_track, marker1_name, marker2_name, user_input):    
    m1_id = int(marker1_name[2:])
    m2_id = int(marker2_name[2:])
    if path_contain_zero_wp(m1_id, m2_id,pub_track.max_id):
        if m1_id < m2_id:
            tmp1 = range(m1_id, -1,-1)
            tmp2 = range(pub_track.max_id,m2_id-1,-1)
            smooth_range = list(tmp1)+list(tmp2)
        else:
            tmp1 = range(m2_id,-1,-1)
            tmp2 = range(pub_track.max_id,m1_id-1,-1)
            smooth_range = list(tmp1)+list(tmp2)
    else:
        if m1_id < m2_id:
            smooth_range = range(m1_id, m2_id+1,1)
        else:
            smooth_range = range(m1_id, m2_id-1,-1)

    for i in smooth_range:
        marker_name = "wp"+str(i)
        int_marker = pub_track.server.get(marker_name)
        int_marker.pose.position.z = np.float64(user_input)

    pub_track.menu_handler.reApply( pub_track.server )
    pub_track.server.applyChanges()
    return

def Vel_Weight(pub_track, marker1_name, marker2_name, user_input):    
    m1_id = int(marker1_name[2:])
    m2_id = int(marker2_name[2:])
    if path_contain_zero_wp(m1_id, m2_id,pub_track.max_id):
        if m1_id < m2_id:
            tmp1 = range(m1_id, -1,-1)
            tmp2 = range(pub_track.max_id,m2_id-1,-1)
            smooth_range = list(tmp1)+list(tmp2)
        else:
            tmp1 = range(m2_id,-1,-1)
            tmp2 = range(pub_track.max_id,m1_id-1,-1)
            smooth_range = list(tmp1)+list(tmp2)
    else:
        if m1_id < m2_id:
            smooth_range = range(m1_id, m2_id+1,1)
        else:
            smooth_range = range(m1_id, m2_id-1,-1)

    for i in smooth_range:
        marker_name = "wp"+str(i)
        int_marker = pub_track.server.get(marker_name)
        int_marker.pose.position.z = int_marker.pose.position.z*np.float64(user_input)

    pub_track.menu_handler.reApply( pub_track.server )
    pub_track.server.applyChanges()
    return

def cal_unit_vec(yaw):
    return np.array([np.cos(yaw), np.sin(yaw)])

def cal_yaw(unit_vector):
    yaw = np.arctan2(unit_vector[1], unit_vector[0])
    return yaw

def sampleCubicSplinesWithDerivative(pub_track,marker_name, resolution,target, scale):
    '''
    Compute and sample the cubic splines for a set of input points with
    optional information about the tangent (direction AND magnitude). The 
    splines are parametrized along the traverse line (piecewise linear), with
    the resolution being the step size of the parametrization parameter.
    The resulting samples have NOT an equidistant spacing.

    Arguments:      points: a list of n-dimensional points
                    tangents: a list of tangents
                    resolution: parametrization step size
    Returns:        samples

    Notes: Lists points and tangents must have equal length. In case a tangent
        is not specified for a point, just pass None. For example:
                    points = [[0,0], [1,1], [2,0]]
                    tangents = [[1,1], None, [1,-1]]

    '''
    smooth_idx=int(marker_name[2:])
    # print(self.server.get(marker_name))
    p=pub_track.server.get(marker_name).pose.position
    p1=pub_track.server.get('wp'+str((smooth_idx-resolution)%pub_track.track_len)).pose.position
    p2=pub_track.server.get('wp'+str((smooth_idx+resolution)%pub_track.track_len)).pose.position
    # tan=self.cal_unit_vec(self.server.get(marker_name).controls[0].markers[0].color.r)
    # tan1=self.cal_unit_vec(self.server.get('wp'+str((smooth_idx-resolution)%self.track_len)).controls[0].markers[0].color.r)
    # tan2=self.cal_unit_vec(self.server.get('wp'+str((smooth_idx+resolution)%self.track_len)).controls[0].markers[0].color.r)
    
    

    points=[]
    tangents=[]
    if target == "Pose":
        tan=cal_unit_vec(pub_track.YAW[smooth_idx])
        # print(self.server.get(marker_name).controls[0].markers[0].color.b)
        tan1=cal_unit_vec(pub_track.YAW[(smooth_idx-resolution)%pub_track.track_len])
        tan2=cal_unit_vec(pub_track.YAW[(smooth_idx+resolution)%pub_track.track_len])
        # print(tan)
        points.append([p1.x,p1.y]) ; tangents.append(tan1)
        points.append([p.x,p.y]) ; tangents.append(tan)
        points.append([p2.x,p2.y]) ; tangents.append(tan2)

    elif target == "Vel":
        vec1=np.array([1, (pub_track.server.get('wp'+str((smooth_idx-resolution+1)%pub_track.track_len)).pose.position.z-pub_track.server.get('wp'+str((smooth_idx-resolution-1)%pub_track.track_len)).pose.position.z)/2])
        vec=np.array([1, (pub_track.server.get('wp'+str((smooth_idx+1)%pub_track.track_len)).pose.position.z-pub_track.server.get('wp'+str((smooth_idx-1)%pub_track.track_len)).pose.position.z)/2])
        vec2=np.array([1, (pub_track.server.get('wp'+str((smooth_idx+resolution+1)%pub_track.track_len)).pose.position.z-pub_track.server.get('wp'+str((smooth_idx+resolution-1)%pub_track.track_len)).pose.position.z)/2])
        
        

        points.append([0,p1.z]) ; tangents.append(vec1/np.linalg.norm(vec1))
        points.append([resolution,p.z]) ; tangents.append(vec/np.linalg.norm(vec))
        points.append([resolution*2,p2.z]) ; tangents.append(vec2/np.linalg.norm(vec2))



    tangents = np.dot(tangents, scale*np.eye(2))
    points = np.asarray(points)
    nPoints, dim = points.shape

    # Parametrization parameter s.
    dp = np.diff(points, axis=0)                 # difference between points
    dp = np.linalg.norm(dp, axis=1)              # distance between points
    d = np.cumsum(dp)                            # cumsum along the segments
    d = np.hstack([[0],d])                       # add distance from first point
    l = d[-1]                                    # length of point sequence
    nSamples = resolution*2+1                 # number of samples 
    s,r = np.linspace(0,l,nSamples,retstep=True) # sample parameter and step

    # Bring points and (optional) tangent information into correct format.
    assert(len(points) == len(tangents))
    data = np.empty([nPoints, dim], dtype=object)
    for i,p in enumerate(points):
        t = tangents[i]
        # Either tangent is None or has the same
        # number of dimensions as the point p.
        assert(t is None or len(t)==dim)
        fuse = list(zip(p,t) if t is not None else zip(p,))
        data[i,:] = fuse

    # Compute splines per dimension separately.
    samples = np.zeros([nSamples, dim])
    for i in range(dim):
        poly = interpolate.BPoly.from_derivatives(d, data[:,i])
        samples[:,i] = poly(s)
    # print(samples)
    
    
    for i in range(resolution*2+1):

        marker_name = "wp"+str((smooth_idx-resolution+i)%pub_track.track_len)
        ori_pose = pub_track.server.get(marker_name).pose
        new_pose = Pose()
        if target=="Pose":
            new_pose.position.x = samples[i][0]
            new_pose.position.y = samples[i][1]
            new_pose.position.z = ori_pose.position.z
            # new_pose.orientation.z=0.2
            # new_pose.orientation.w=0.2
        elif target=="Vel":
            new_pose.position.x = ori_pose.position.x
            new_pose.position.y = ori_pose.position.y
            new_pose.position.z = samples[i][1]
            

        new_pose.orientation = ori_pose.orientation
        pub_track.server.setPose(marker_name, new_pose)        
    
    pub_track.menu_handler.reApply( pub_track.server )
    pub_track.server.applyChanges()
    return

def entire_traj_transform(pub_track, clicked_marker_name):
    idx=int(clicked_marker_name[2:])
    new_pos=pub_track.server.get(clicked_marker_name).pose.position
    original_pos=pub_track.track.markers[idx].pose.position
    x_disp=new_pos.x-original_pos.x
    y_disp=new_pos.y-original_pos.y
    
    for i in range(pub_track.track_len):
        if i==idx:
            continue
        else:
            marker_name = "wp"+str(i)
            ori_pose = pub_track.server.get(marker_name).pose
            new_pose = Pose()
            new_pose.position.x = ori_pose.position.x+x_disp
            new_pose.position.y = ori_pose.position.y+y_disp
            new_pose.position.z = ori_pose.position.z
            new_pose.orientation = ori_pose.orientation
        pub_track.server.setPose(marker_name, new_pose)        
    
    pub_track.menu_handler.reApply( pub_track.server )
    pub_track.server.applyChanges()
    return       
    
def rotate_point( point, angle, center):
    # Calculate the angle in radians
    
    # Translate the point to the origin
    translated_x = point.x - center.x
    translated_y = point.y - center.y
    
    # Perform the rotation
    rotated_x = translated_x * np.cos(angle) - translated_y * np.sin(angle)
    rotated_y = translated_x * np.sin(angle) + translated_y * np.cos(angle)
    
    # Translate the point back to its original position
    result_x = rotated_x + center.x
    result_y = rotated_y + center.y
    
    return result_x, result_y
        
def entire_traj_rotation(pub_track, marker1_name, marker2_name):
    m1_id = int(marker1_name[2:])
    m2_id = int(marker2_name[2:])
    m1_int_pos=pub_track.server.get(marker1_name).pose.position
    m2_int_pos=pub_track.server.get(marker2_name).pose.position
    m1_track_pos=pub_track.track.markers[m1_id].pose.position
    m2_track_pos=pub_track.track.markers[m2_id].pose.position
    dis_sqr_1=(m1_int_pos.x - m1_track_pos.x)**2+(m1_int_pos.y - m1_track_pos.y)**2
    dis_sqr_2=(m2_int_pos.x - m2_track_pos.x)**2+(m2_int_pos.y - m2_track_pos.y)**2
    if dis_sqr_1<dis_sqr_2:
        fix_pos=pub_track.track.markers[m1_id].pose.position
        mov_ori_pose=pub_track.track.markers[m2_id].pose.position
        mov_new_pose=pub_track.server.get(marker2_name).pose.position
    else:
        fix_pos=pub_track.track.markers[m2_id].pose.position
        mov_ori_pose=pub_track.track.markers[m1_id].pose.position
        mov_new_pose=pub_track.server.get(marker1_name).pose.position
    ori_vector=[mov_ori_pose.x-fix_pos.x,mov_ori_pose.y-fix_pos.y]
    new_vector=[mov_new_pose.x-fix_pos.x,mov_new_pose.y-fix_pos.y]
    dot_product=ori_vector[0]*new_vector[0]+ori_vector[1]*new_vector[1]
    cross_product=ori_vector[0]*new_vector[1]-ori_vector[1]*new_vector[0]
    ori_magnitude=np.sqrt(ori_vector[0]**2+ori_vector[1]**2)
    new_magnitude=np.sqrt(new_vector[0]**2+new_vector[1]**2)

    
    # Calculate the rotation angle
    if cross_product>0:
        angle = np.arccos(dot_product / (ori_magnitude*new_magnitude))
    else:
        angle = -np.arccos(dot_product / (ori_magnitude*new_magnitude))
    # Use one of the points as the center of rotation (e.g., m1_int_pos)
    center = fix_pos
    
    for i in range(pub_track.track_len):
        marker_name = "wp" + str(i)
        if i == m1_id or i==m2_id:
            ori_pose=pub_track.track.markers[i].pose
        else:
            ori_pose = pub_track.server.get(marker_name).pose
        new_x, new_y = rotate_point(ori_pose.position, angle, center)
        
        new_pose = Pose()
        new_pose.position.x = new_x
        new_pose.position.y = new_y
        new_pose.position.z = ori_pose.position.z
        new_pose.orientation = ori_pose.orientation
        
        pub_track.server.setPose(marker_name, new_pose)

    pub_track.menu_handler.reApply(pub_track.server)
    pub_track.server.applyChanges()


def set_lookahead(pub_track, marker1_name, marker2_name, input_value):
    print(marker1_name[2:])
    m1_id = int(marker1_name[2:])
    m2_id = int(marker2_name[2:])
    if path_contain_zero_wp(m1_id, m2_id,pub_track.max_id):
        if m1_id < m2_id:
            tmp1 = range(m1_id, -1,-1)
            tmp2 = range(pub_track.max_id,m2_id-1,-1)
            smooth_range = list(tmp1)+list(tmp2)
        else:
            tmp1 = range(m2_id,-1,-1)
            tmp2 = range(pub_track.max_id,m1_id-1,-1)
            smooth_range = list(tmp1)+list(tmp2)
    else:
        if m1_id < m2_id:
            smooth_range = range(m1_id, m2_id+1,1)
        else:
            smooth_range = range(m1_id, m2_id-1,-1)

    for i in smooth_range:
        marker_name = "wp"+str(i)
        int_marker = pub_track.server.get(marker_name)
        int_marker.controls[0].markers[0].color.r = np.float32(input_value)
        
    pub_track.menu_handler.reApply( pub_track.server )
    pub_track.server.applyChanges()
    return

def OnOff(pub_track,marker1_name, marker2_name, input_value,Target):
    # print(marker1_name[2:]) 
    m1_id = int(marker1_name[2:])
    m2_id = int(marker2_name[2:])
    if path_contain_zero_wp(m1_id, m2_id,pub_track.max_id):
        if m1_id < m2_id:
            tmp1 = range(m1_id, -1,-1)
            tmp2 = range(pub_track.max_id,m2_id-1,-1)
            smooth_range = list(tmp1)+list(tmp2)
        else:
            tmp1 = range(m2_id,-1,-1)
            tmp2 = range(pub_track.max_id,m1_id-1,-1)
            smooth_range = list(tmp1)+list(tmp2)
    else:
        if m1_id < m2_id:
            smooth_range = range(m1_id, m2_id+1,1)
        else:
            smooth_range = range(m1_id, m2_id-1,-1)
    
    for i in smooth_range:
        marker_name = "wp"+str(i)
        int_marker = pub_track.server.get(marker_name)
        if Target=="Lookahead":
            int_marker.controls[0].markers[0].color.g = input_value
            int_marker.controls[0].markers[0].color.r = pub_track.track.markers[i].color.r
            int_marker.controls[0].markers[0].color.b = pub_track.track.markers[i].color.b
        if Target=="Velocity":
            int_marker.controls[0].markers[0].color.b = input_value
            int_marker.controls[0].markers[0].color.r = pub_track.track.markers[i].color.r
            int_marker.controls[0].markers[0].color.g = pub_track.track.markers[i].color.g
    pub_track.menu_handler.reApply( pub_track.server )
    pub_track.server.applyChanges()
    return



def distance_between_int_markers(int_marker1, int_marker2):
    if int_marker2 == None:
        return 999
    m1_p = np.array([int_marker1.pose.position.x, int_marker1.pose.position.y, int_marker1.pose.position.z])
    m2_p = np.array([int_marker2.pose.position.x, int_marker2.pose.position.y, int_marker2.pose.position.z])
    distance = np.linalg.norm(m1_p-m2_p)
    # print(distance)
    return distance

def cal_slope(prev_marker, cur_marker, next_marker):
    p_x=prev_marker.pose.position.x
    p_y=prev_marker.pose.position.y
    c_x=cur_marker.pose.position.x
    c_y=cur_marker.pose.position.y
    n_x=next_marker.pose.position.x
    n_y=next_marker.pose.position.y
    vec_1=np.array([c_x-p_x,c_y-p_y])
    vec_2=np.array([n_x-c_x,n_y-c_y])
    # print(vec_1)
    norm_vec_1=vec_1/np.linalg.norm(vec_1)
    norm_vec_2=vec_2/np.linalg.norm(vec_2)
    # print(norm_vec_1)
    return (vec_1+vec_2)/np.linalg.norm(norm_vec_1+norm_vec_2)

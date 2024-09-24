#!/usr/bin/env python3

import os
import rospy
import numpy as np

def main():
    file_name1 = rospy.get_param('/pub_track_gui/filename')
    file_path = os.path.abspath(__file__)
    current_script_path = os.path.dirname(file_path)
    file_name = current_script_path + '/result/'+ file_name1
    print(file_name)
    track=np.loadtxt(file_name, delimiter=",", dtype = float)
    new_track=interpolate_track(track, 0.3)

    new_file_name = file_name[:-4] + "2.txt"  # Assumes original file ends with '.txt'
    np.savetxt(new_file_name, new_track, delimiter=",")
    # shutil.copy2(file_name,track_manager.name_current)

def interpolate_track(track, desired_spacing):
    interpolated_track = []
    
    if len(track) < 2:
        return track  # Nothing to interpolate if there are fewer than 2 points
    
    interpolated_track.append(track[0])  # Add the first point as is

    for i in range(1, len(track)):

        new_point = track[i].copy()
        x1, y1, speed1 = track[i - 1,0],track[i - 1,1],track[i - 1,2]
        x2, y2, speed2 = track[i,0], track[i,1], track[i,2]
        
        # Calculate the Euclidean distance between the two points
        distance = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        # print(distance)
        
        if distance > desired_spacing:
            num_interpolations = int(round(distance / desired_spacing))
            # print(num_interpolations)
            dx = (x2 - x1) / num_interpolations
            dy = (y2 - y1) / num_interpolations
            ds = (speed2 - speed1) / num_interpolations
            # print(dx, dy, ds)
            
            for j in range(1, num_interpolations):
                xi = x1 + j * dx

                yi = y1 + j * dy
                si = speed1 + j * ds
                
                new_point[0] = xi
                new_point[1] = yi
                new_point[2] = si

                interpolated_track.append(new_point.copy())

        
        interpolated_track.append(track[i])  # Add the current point as is

    return np.array(interpolated_track)



if __name__ == '__main__':
    main()
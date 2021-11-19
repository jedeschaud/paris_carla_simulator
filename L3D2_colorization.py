import glob
import os
import sys
from pathlib import Path
import numpy as np
import math
import struct
import time
from operator import itemgetter
from skimage import io

from modules import ply

# Bug in CARLA v993f440b (0.9.10) the LiDAR has bad semantic for class 6 (RoadlLine)
# If point semantic = 7 and semantic_image = 6 then semantic = 6

def main():

    for i_map in [0, 1, 2, 3, 4, 5, 6]:
    
        root_dir = "L3D2_Dataset_CARLA_v993f440b/Town0" + str(i_map+1)
        
        start_record = time.time()
        folder_input_raw_data = root_dir+"/generated"
        folder_input_point_cloud = root_dir+"/georef"
        folder_output = root_dir+"/georef_colorized"

        # Create folders or remove files
        os.makedirs(folder_output) if not os.path.exists(folder_output) else [os.remove(f) for f in glob.glob(folder_output+"/*") if os.path.isfile(f)]
         
        # Intrinsic parameters
        fx = 2048/(2*math.tan((120.0/2.0)*(math.pi/180.0)))
        fy = fx
        cu = 2048/2.0
        cv = 2048/2.0
        K = np.array([[fx,0,cu,0],[0,fy,cv,0],[0,0,1,0]])

        # Fixed part of extrinsic camera parameters
        # Camera to camera projection frame
        # rot_x -90
        Rx = np.array([[1,0,0],[0,0,-1],[0,1,0]])
        # rot_z -90
        Rz = np.array([[0,-1,0],[1,0,0],[0,0,1]])
        R_cam_proj = Rx.dot(Rz)
        tf_cam_proj = np.vstack((np.hstack((R_cam_proj,[[0],[0],[0]])), [0,0,0,1]))

        # LiDAR -> Ladybug
        with open(folder_input_raw_data+"/lidar_to_ladybug.txt", 'r') as transfo_file:
            transfo_file.readline()
            tf_lidar_ladybug = np.vstack((np.asarray(transfo_file.readline().split(), float).reshape(3,4), [0,0,0,1]))

        # Assumption : ladybug with 0 angle in roll and pitch
        # Ladybug to cam
        R_ladybug_cam_list = []
        for i in range(5):
            R_ladybug_cam_list.append(np.array([[math.cos(i*(-2*math.pi/5)), math.sin(i*(-2*math.pi/5)), 0], [-math.sin(i*(-2*math.pi/5)), math.cos(i*(-2*math.pi/5)), 0], [0, 0, 1]]))
        # rot -90 y
        Ry = np.array([[math.cos(-math.pi/2), 0, -math.sin(-math.pi/2)], [0, 1, 0], [math.sin(-math.pi/2), 0, math.cos(-math.pi/2)]])
        R_ladybug_cam_list.append(Ry)
        tf_ladybug_cam_list = []
        for R_ladybug_cam in R_ladybug_cam_list:
            tf_ladybug_cam_list.append(np.vstack((np.hstack((R_ladybug_cam, [[0],[0],[0]])), [0,0,0,1])))

        # Lidar to cam projection coordinate
        tf_lidar_cams_proj = []
        for tf_ladybug_cam in tf_ladybug_cam_list:
            tf_lidar_cams_proj.append(tf_cam_proj.dot(tf_ladybug_cam).dot(tf_lidar_ladybug))

        ply_files = sorted(glob.glob(folder_input_point_cloud+"/point_cloud_*.ply"))
        
        ts_camera_file = open(folder_input_raw_data+"/full_ts_camera.txt", 'r')
        ts_camera_file.readline()
        ts_camera = np.asarray(ts_camera_file.readline().split(), float)[-1]
        
        # Opening the images
        index_camera = 0
        img_data = []
        for img_path in sorted(glob.glob(folder_input_raw_data+"/images_rgb/"+format(index_camera, '04d')+"_*.png")):
            img_data.append(io.imread(img_path))
        img_ss_data = []
        for img_path in sorted(glob.glob(folder_input_raw_data+"/images_ss/"+format(index_camera, '04d')+"_*.png")):
            img_ss_data.append(io.imread(img_path))

        # Open poses file and read header
        poses_file = open(folder_input_raw_data+"/full_poses_lidar.txt", 'r')
        poses_file.readline()
        line_pose = np.array(poses_file.readline().split(), float)
        tf_geo_to_lidar = np.linalg.inv(np.vstack((line_pose[:-1].reshape(3,4), [0,0,0,1])))
        ts_tf_geo_to_lidar = line_pose[-1]
        while ((ts_camera-ts_tf_geo_to_lidar)>1e-5):
            line_pose = np.array(poses_file.readline().split(), float)
            tf_geo_to_lidar = np.linalg.inv(np.vstack((line_pose[:-1].reshape(3,4), [0,0,0,1])))
            ts_tf_geo_to_lidar = line_pose[-1]

        end_of_camera = False
        for i_ply, f in enumerate(ply_files):
                
            data = ply.read_ply(f)
            nbr_pts = len(data)
            
            color_point_cloud = np.empty([nbr_pts,3], dtype = np.uint32)
            semantic_point_cloud = np.empty([nbr_pts,1], dtype = np.uint32)
            semantic_image_point_cloud = np.empty([nbr_pts,1], dtype = np.uint32)

            i = 0
            while ((i < nbr_pts) and (not(end_of_camera))):

                if (data[i]['timestamp'] > ts_camera): 
                    line = np.asarray(ts_camera_file.readline().split(), float)
                    if (len(line)>0):
                        ts_camera = line[-1]
                        index_camera +=1
                        img_data = []
                        for img_path in sorted(glob.glob(folder_input_raw_data+"/images_rgb/"+format(index_camera, '04d')+"_*.png")):
                            img_data.append(io.imread(img_path))
                        img_ss_data = []
                        for img_path in sorted(glob.glob(folder_input_raw_data+"/images_ss/"+format(index_camera, '04d')+"_*.png")):
                            img_ss_data.append(io.imread(img_path))
                        while ((ts_camera-ts_tf_geo_to_lidar)>1e-4):
                            line_pose = np.array(poses_file.readline().split(), float)
                            tf_geo_to_lidar = np.linalg.inv(np.vstack((line_pose[:-1].reshape(3,4), [0,0,0,1])))
                            ts_tf_geo_to_lidar = line_pose[-1]
                    else:
                        end_of_camera = True
                
                if (not(end_of_camera)):
                    last_point = i
                    while ((last_point<nbr_pts) and (data[last_point]['timestamp'] <= ts_camera)): 
                        last_point +=1

                    pt_in_geo_frame = np.vstack(np.array([data[i:last_point]['x'], data[i:last_point]['y'], data[i:last_point]['z'], np.full(last_point-i, 1)]))
                    pt_in_lidar_frame = tf_geo_to_lidar.dot(pt_in_geo_frame)
                    
                    # Projection in image
                    norm_list = []
                    pts_img_list = []
                    for tf_lidar_cam_proj in tf_lidar_cams_proj:
                        pts_cam = tf_lidar_cam_proj.dot(pt_in_lidar_frame)
                        pts_img = K.dot(pts_cam)
                        pts_img /= pts_img[2]
                        pts_img = pts_img[:-1]
                        pts_img[:, pts_cam[2]<=0] += 100000
                        pts_img_list.append(pts_img.T)
                        norm_list.append(np.linalg.norm(pts_img - np.array([[cu], [cv]]), axis=0))
                    
                    # Choose best camera
                    cam_proj = np.nanargmin(np.vstack(norm_list), axis=0)
                    pts_proj = np.rint(np.hstack(pts_img_list).reshape(last_point-i, 6, 2)[np.arange(last_point-i), cam_proj]).astype(int)
                    
                    # Reject impossible pixel
                    index_u_min = pts_proj[:,0]>=2048
                    index_u_max = pts_proj[:,0]<0
                    index_v_min = pts_proj[:,1]>=2048
                    index_v_max = pts_proj[:,1]<0
                    index = (index_u_min|index_v_min|index_u_max|index_v_max).T
                    pts_proj[index] = 0
                    
                    # Colorization
                    color = np.array(img_data)[cam_proj,pts_proj[:,1],pts_proj[:,0],:-1]
                    color[index] = np.array([0,255,255])        # Cyan = No Color
                    color_point_cloud[i:last_point,:] = color
                    
                    # Semantic Colorization
                    color_ss = np.array(img_ss_data)[cam_proj,pts_proj[:,1],pts_proj[:,0],:-1]
                    color_ss[index] = np.array([0,255,255])        # Cyan -> Unlabeled
                    semantic_image_point_cloud[i:last_point,:] = color_ss[:,0].reshape(len(color_ss[:,0]),1)
                    
                    # Change semantic point for RoadLine (bug in CARLA)
                    semantic_point_cloud[i:last_point,:] = (data[i:last_point]['semantic']).reshape(-1,1)
                    chosen_points = np.logical_and((semantic_point_cloud[i:last_point,:]==7).flatten(),(semantic_image_point_cloud[i:last_point,:]==6).flatten())
                    semantic_point_cloud[i:last_point,:][chosen_points,:] = 6

                    i = last_point
         
            if (not(end_of_camera)):
                field_names = ['x','y','z','x_lidar_position','y_lidar_position','z_lidar_position','cos_angle_lidar_surface','timestamp','index_frame','instance','semantic', 'red', 'green', 'blue', 'semantic_image']
                ply_file_path = folder_output+"/point_cloud_rgb_%02d.ply"%i_ply
                if ply.write_ply(ply_file_path, [data['x'], data['y'], data['z'], data['x_sensor_position'], data['y_sensor_position'], data['z_sensor_position'], data['cos'], data['timestamp'], data['index_frame'], data['index'], semantic_point_cloud, color_point_cloud, semantic_image_point_cloud], field_names):
                    print("Export : "+ply_file_path)
                else:
                    print('ply.write_ply() failed')
                
                
        print("Elapsed time : ", time.time()-start_record)

if __name__ == '__main__':
        main()

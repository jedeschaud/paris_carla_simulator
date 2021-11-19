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


SIZE_POINT_CLOUD = 100000000

def main():

    for i_map in [0, 1, 2, 3, 4, 5, 6]:
    
        root_dir = "L3D2_Dataset_CARLA_v993f440b/Town0" + str(i_map+1)
        
        start_record = time.time()
        folder_input_point_cloud = root_dir+"/georef_colorized"
        folder_output = root_dir+"/georef_colorized_no_cyan_and_merge"
        
        # Create folders or remove files
        os.makedirs(folder_output) if not os.path.exists(folder_output) else [os.remove(f) for f in glob.glob(folder_output+"/*") if os.path.isfile(f)]
        
        ply_files = sorted(glob.glob(folder_input_point_cloud+"/point_cloud_*.ply"))
        
        point_cloud_float32_out = np.empty([SIZE_POINT_CLOUD,8], dtype = np.float32)
        point_cloud_int32_out = np.empty([SIZE_POINT_CLOUD,4], dtype = np.uint32)
        point_cloud_uchar_out = np.empty([SIZE_POINT_CLOUD,3], dtype = np.ubyte)
        
        index_point = 0
        output_point_cloud = False
        
        for i_ply, f in enumerate(ply_files):
        
            if not(output_point_cloud):
                data = ply.read_ply(f)
                nbr_pts = len(data)
                
                i = 0
                while (i < nbr_pts):
                
                    if ((data[i]['red']==0) and (data[i]['green']==255) and (data[i]['blue']==255)): # Cyan = remove point
                        i += 1
                    else :
                        point_cloud_float32_out[index_point,:] = np.array([data[i]['x'], data[i]['y'], data[i]['z'], data[i]['x_lidar_position'], data[i]['y_lidar_position'], data[i]['z_lidar_position'], data[i]['cos_angle_lidar_surface'], data[i]['timestamp']])
                        point_cloud_int32_out[index_point,:] = np.array([data[i]['index_frame'], data[i]['instance'], data[i]['semantic'], data[i]['semantic_image']])
                        point_cloud_uchar_out[index_point,:] = np.array([data[i]['red'], data[i]['green'], data[i]['blue']])
                        index_point +=1
                        if (index_point == SIZE_POINT_CLOUD):
                            field_names = ['x','y','z','x_lidar_position','y_lidar_position','z_lidar_position','cos_angle_lidar_surface','timestamp','index_frame','instance','semantic', 'semantic_image', 'red', 'green', 'blue', ]
                            ply_file_path = folder_output+"/Town0%01d.ply"%(i_map+1)
                            if ply.write_ply(ply_file_path, [point_cloud_float32_out, point_cloud_int32_out, point_cloud_uchar_out], field_names):
                                print("Export : "+ply_file_path)
                                output_point_cloud = True
                            else:
                                print('ply.write_ply() failed')
                            index_point = 0
                        i += 1
                
                
        print("Elapsed time : ", time.time()-start_record)        
                
if __name__ == '__main__':
        main()
            
        
        
        
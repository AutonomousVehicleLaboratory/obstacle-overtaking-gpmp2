import csv
import cv2
import matplotlib.pyplot as plt
from math import floor, ceil
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as R


'''
Map Parameters
--------------
'''
cell_size = 1                                         # Size of the cell 0.1 m x 0.1 m
map_height_offset = 10                               # Height Offset of the Map, e.g., if 60, then 30 top, 30 bottom
map_width_offset = 10                                 # Width Offset of the Map, e.g., if 60, then 30 left, 30 right
road_width = 5                                        # Road Width in Unit Cell

'''
Load Trajectory Files
---------------------
'''

# Load the pose data (from rosbag)
poseData = []
with open("data/obs-avd-2019-10-22-08-44-02-current_pose.csv", 'r') as file:
    csvreader = csv.reader(file)
    header = next(csvreader)
    poseData.append(header)
    for row in csvreader:
        poseData.append(row)

# Store all the trajectory 3D positions in a numpy array
pos_ls = []
for idx in range(1, len(poseData)):
    x_pos, y_pos, z_pos = poseData[idx][5:8]
    x_pos, y_pos, z_pos = float(x_pos), float(y_pos), float(z_pos)
    pos_ls.append([x_pos, y_pos, z_pos])
    
# Load the ref Traj data
RefTraj = []
with open("data/global_plan.csv", 'r') as file:
    csvreader = csv.reader(file)
    header = next(csvreader)
    RefTraj.append(header)
    for row in csvreader:
        RefTraj.append(row)

RefTraj_X = []
RefTraj_Y = []
min_map_height, max_map_height = float('inf'), float('-inf')
min_map_width, max_map_width = float('inf'), float('-inf')

# for idx in range(1,len(RefTraj)):
# Region of Interest Data Points
for idx in range(1,101):
    x_pos, y_pos = float(RefTraj[idx][0]), float(RefTraj[idx][1])
    # Ignore the zeros-zeros initialization
    if idx > 1:
        min_map_height = min(min_map_height, y_pos)
        max_map_height = max(max_map_height, y_pos)
        min_map_width = min(min_map_width, x_pos)
        max_map_width = max(max_map_width, x_pos)
    RefTraj_X.append(x_pos)
    RefTraj_Y.append(y_pos)
    
'''
Load Labeled Obstacles File
---------------------------
'''
# Load the lidar points
label_df = pd.read_json('data/label_data_obstacles.json')
lidarData = []

# Extract the truck object information
truck = label_df[label_df["Truck"].str.len() != 0]
x_center_truck, y_center_truck, z_center_truck, x_len_truck, y_len_truck, z_len_truck, x_rot_truck, y_rot_truck, z_rot_truck = truck.iat[0,1]         # Position in meters, Orientation in degrees (Positive Clockwise)
pos_truck = np.array([[x_center_truck],[y_center_truck],[z_center_truck],[1.0]])                                                                      # Lidar Frame
#print(x_len_truck, y_len_truck)

# Extract the van object information
x_center_van, y_center_van, z_center_van, x_len_van, y_len_van, z_len_van, x_rot_van, y_rot_van, z_rot_van = 11.734661365671949,5.2403188962579863,0.77907542911370786,7.68055215895081,2.9153533088167247,2.7494292253812116,0,0,5.2812376882571357        
pos_van = np.array([[x_center_van],[y_center_van],[z_center_van],[1.0]])   
#print(x_len_van, y_len_van)      

'''
Reference Trajectory Data Extraction
------------------------------------
'''
global_plan_x = [0]*100
global_plan_y = [0]*100

start_PosX, start_PosY, start_PosZ = pos_ls[0]
global_plan_x[0] = start_PosX
global_plan_y[0] = start_PosY

global_plan_x[1:100] = RefTraj_X[1:100]
global_plan_y[1:100] = RefTraj_Y[1:100]

min_map_width = min(global_plan_x)
max_map_width = max(global_plan_x)
min_map_height = min(global_plan_y)
max_map_height = max(global_plan_y)

global_plan_x_arr = np.array(global_plan_x)
global_plan_y_arr = np.array(global_plan_y)

'''
Initialize Map
--------------
'''
map_height = max_map_height - min_map_height + map_height_offset
map_width = max_map_width - min_map_width + map_width_offset
map_arr_height, map_arr_width = int(map_height / cell_size) + 1, int(map_width / cell_size) + 1

# Build the initial map
UCSD_MAP = np.zeros((map_arr_height, map_arr_width), dtype=np.uint8)

'''
Reference Trajectory Data Plot
------------------------------
'''
OGM_plan_x_arr = global_plan_x_arr - min_map_width + map_width_offset // 2
OGM_plan_y_arr = global_plan_y_arr - min_map_height + map_height_offset // 2

for idx in range(len(OGM_plan_x_arr)):
    UCSD_MAP[int(OGM_plan_y_arr[idx])-road_width:int(OGM_plan_y_arr[idx])+road_width+1, int(OGM_plan_x_arr[idx])-road_width:int(OGM_plan_x_arr[idx])+road_width+1] = 2

'''
Obstacle Bounding Boxes Plot
----------------------------
'''
# Construct tranformation matrix of vehicle frame wrt map/world frame at t = 13.715118169784546
vehicle_PosX, vehicle_PosY, vehicle_PosZ = poseData[136][5:8]
vehicle_OrientationX, vehicle_OrientationY, vehicle_OrientationZ, vehicle_OrientationW = poseData[136][8:12]

r_vehicle = R.from_quat([vehicle_OrientationX, vehicle_OrientationY, vehicle_OrientationZ, vehicle_OrientationW])
rotMat_Vehicle = r_vehicle.as_matrix()
world_T_vehicle = np.eye(4)
world_T_vehicle[:3,:3] = rotMat_Vehicle
world_T_vehicle[0,3], world_T_vehicle[1,3], world_T_vehicle[2,3] = vehicle_PosX, vehicle_PosY, vehicle_PosZ
#print(world_T_vehicle)

# Construct tranformation matrix of vehicle frame wrt map/world frame at t = 20.271127223968506    
vehicle2_PosX, vehicle2_PosY, vehicle2_PosZ = poseData[200][5:8]
vehicle2_OrientationX, vehicle2_OrientationY, vehicle2_OrientationZ, vehicle2_OrientationW = poseData[200][8:12]

r_vehicle2 = R.from_quat([vehicle2_OrientationX, vehicle2_OrientationY, vehicle2_OrientationZ, vehicle2_OrientationW])
rotMat_Vehicle2 = r_vehicle2.as_matrix()
world_T_vehicle2 = np.eye(4)
world_T_vehicle2[:3,:3] = rotMat_Vehicle2
world_T_vehicle2[0,3], world_T_vehicle2[1,3], world_T_vehicle2[2,3] = vehicle2_PosX, vehicle2_PosY, vehicle2_PosZ
#print(world_T_vehicle)

# Construct transformation matrix of lidar frame wrt vehicle frame
r_truck = R.from_quat([0.0, 0.07143909459110181, 0.0, 0.9974449637769512])
rotMat_Truck = r_truck.as_matrix()
vehicle_T_lidar = np.eye(4)
vehicle_T_lidar[:3,:3] = rotMat_Truck
vehicle_T_lidar[0,3], vehicle_T_lidar[1,3], vehicle_T_lidar[2,3] = 2.64, 0.0, 1.98

pos_truck = world_T_vehicle @ vehicle_T_lidar @ pos_truck

pos_van = world_T_vehicle2 @ vehicle_T_lidar @ pos_van

# Define the center, width, height, and angle of rotation of the rectangle
center_truck = (floor(pos_truck[0][0] - min_map_width + map_width_offset // 2), floor(pos_truck[1][0] - min_map_height + map_height_offset //2 )-2)
width_truck, height_truck = x_len_truck, y_len_truck
angle_truck = 19.5

center_van = (floor(pos_van[0][0] - min_map_width + map_width_offset // 2), floor(pos_van[1][0] - min_map_height + map_height_offset //2 ))
width_van, height_van = x_len_van, y_len_van
angle_van = 10.2812

# Create a rotation matrix
rotation_matrix_truck = cv2.getRotationMatrix2D(center_truck, angle_truck, 1)
rotation_matrix_van = cv2.getRotationMatrix2D(center_van, angle_van, 1)

# # Perform the rotation on the rectangle
rect_truck_arr = np.array([[center_truck[0]-width_truck/2, center_truck[1]-height_truck/2], [center_truck[0]-width_truck/2, center_truck[1]+height_truck/2], [center_truck[0]+width_truck/2, center_truck[1]+height_truck/2], [center_truck[0]+width_truck/2, center_truck[1]-height_truck/2]], dtype='float32')
#print(rect_truck_arr)
rect_truck = rect_truck_arr.reshape(4,1,2)
rotated_rect_truck = cv2.transform(rect_truck, rotation_matrix_truck)
rotated_rect_truck = np.int32(rotated_rect_truck)
print(rotated_rect_truck)

rect_van = np.array([[center_van[0]-width_van/2, center_van[1]-height_van/2], [center_van[0]-width_van/2, center_van[1]+height_van/2], [center_van[0]+width_van/2, center_van[1]+height_van/2], [center_van[0]+width_van/2, center_van[1]-height_van/2]], dtype='float32')
rect_van = rect_van.reshape(4,1,2)
rotated_rect_van = cv2.transform(rect_van, rotation_matrix_van)
rotated_rect_van = np.int32(rotated_rect_van)

imaginary_obs_1 = rect_truck_arr + 8
imaginary_obs_1 = imaginary_obs_1.reshape(4,1,2)
rotated_img_obs_1 = cv2.transform(imaginary_obs_1, rotation_matrix_truck)
rotated_img_obs_1 = np.int32(rotated_img_obs_1)

imaginary_obs_2 = np.array([[62,70],[63,74],[78,69],[77,65]])
imaginary_obs_2 = imaginary_obs_2.reshape(4,1,2)
#rotated_img_obs_2 = cv2.transform(imaginary_obs_2,rotation_matrix_truck)
rotated_img_obs_2 = np.int32(imaginary_obs_2)
#print(rotated_img_obs_2)

imaginary_obs_3 = np.array([[85,63],[86,67],[101,62],[100,58]])
imaginary_obs_3 = imaginary_obs_3.reshape(4,1,2)
rotated_img_obs_3 = np.int32(imaginary_obs_3)

imaginary_obs_4 = np.array([[85,63],[86,73],[101,68],[100,58]])
imaginary_obs_4 = imaginary_obs_4.reshape(4,1,2)
rotated_img_obs_4 = np.int32(imaginary_obs_4)

'''
Occupancy Grid Map Visualization
--------------------------------
'''
# # Fill the rotated rectangle on the binary image

cv2.fillPoly(UCSD_MAP,[rotated_rect_truck],255*0)
#cv2.fillPoly(UCSD_MAP,[rotated_rect_van],255*0)
#cv2.fillPoly(UCSD_MAP,[rotated_img_obs_1],255*0)
#cv2.fillPoly(UCSD_MAP,[rotated_img_obs_2],255*0)
cv2.fillPoly(UCSD_MAP,[rotated_img_obs_3],255*0)
#cv2.fillPoly(UCSD_MAP,[rotated_img_obs_4],255*0)

#print(len(UCSD_MAP[0]))

roi_map = np.zeros((60,90),dtype = np.uint8)
for i in range(60):
    for j in range(90):
        roi_map[i][j] = UCSD_MAP[100-i][j]

np.savetxt("ucsd_map_it5.txt",UCSD_MAP,fmt = '%d')

plt.imshow(roi_map, cmap='gray') 
plt.imshow(UCSD_MAP, cmap = 'gray')
plt.title('UCSD Map')
plt.gca().invert_yaxis()
plt.show()



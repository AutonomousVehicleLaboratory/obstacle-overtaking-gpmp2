import csv
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from scipy.spatial.transform import Rotation as R
import numpy as np
import math



# Load the lidar points
label_df = pd.read_json('data/label_data_obstacles.json')
lidarData = []

# Extract the truck object information
truck = label_df[label_df["Truck"].str.len() != 0]
x_center_truck, y_center_truck, z_center_truck, x_len_truck, y_len_truck, z_len_truck, x_rot_truck, y_rot_truck, z_rot_truck = truck.iat[0,1]         # Position in meters, Orientation in degrees (Positive Clockwise)
pos_truck = np.array([[x_center_truck],[y_center_truck],[z_center_truck],[1.0]])                                                                      # Lidar Frame
#print("length of truck ")
#print (x_len_truck)

# Extract the van object information
x_center_van, y_center_van, z_center_van, x_len_van, y_len_van, z_len_van, x_rot_van, y_rot_van, z_rot_van = 11.734661365671949,5.2403188962579863,0.77907542911370786,7.68055215895081,2.9153533088167247,2.7494292253812116,0,0,5.2812376882571357        
pos_van = np.array([[x_center_van],[y_center_van],[z_center_van],[1.0]])         
print("length of van")
print (y_len_van)

# Load the pose date (from rosbag)
poseData = []
with open("data/obs-avd-2019-10-22-08-44-02-current_pose.csv", 'r') as file:
    csvreader = csv.reader(file)
    header = next(csvreader)
    poseData.append(header)
    for row in csvreader:
        poseData.append(row)

start_PosX, start_PosY, start_PosZ = poseData[1][5:8]

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
for i in range(1,len(RefTraj)):
    #print(i)
    RefTraj_X.append(float(RefTraj[i][0]))
    RefTraj_Y.append(float(RefTraj[i][1]))

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
print("pos_truck in map frame")
print(pos_truck)

pos_van = world_T_vehicle2 @ vehicle_T_lidar @ pos_van
print("pos_van in map frame")
print(pos_van)

## plot grid map!

fig = plt.figure()
ax = fig.add_subplot(111)

# plot truck and van
r1 = patches.Rectangle((pos_truck[0,0]-x_len_truck/2,pos_truck[1,0]-y_len_truck/2), x_len_truck, y_len_truck, color="green", alpha=0.50, angle = -19.5)
r2 = patches.Rectangle((pos_van[0,0]-x_len_van/2,pos_van[1,0]-y_len_van/2), x_len_van, y_len_van, color="red", alpha=0.50, angle = -10.2812)

print('left corner x coordinate')
print(pos_truck[0,0] - x_len_truck/2)
print('left corner y coordinate')
print(pos_truck[1,0] - y_len_truck/2)

tempX_truck = -x_len_truck/2
tempY_truck = -y_len_truck/2

ax.add_patch(r1)
ax.add_patch(r2)

angle_truck = 19.5*np.pi/180
angle_van = 5.2812*np.pi/180

rotatedX_truck = -170.119
rotatedY_truck = -393.751

# plot ego car start pose
x_start_ego = [float(start_PosX)]
y_start_ego = [float(start_PosY)]
plt.plot(x_start_ego, y_start_ego, marker="o", markersize=2, markeredgecolor="red", markerfacecolor="green")

# plot ref trajectory

road_width = 3

global_plan_x = [0]*100
global_plan_y = [0]*100

global_plan_x[0] = -206.2080078125
global_plan_y[0] = -370.92401123046875

global_plan_x[1:100] = RefTraj_X[1:100]
global_plan_y[1:100] = RefTraj_Y[1:100]

plt.plot(global_plan_x,global_plan_y,linestyle = 'dashed', color = 'black')

def offset(x,y, distance):
    length = len(x)
    x1 = x[1]
    y1 = y[1]
    z = distance
    newX = []
    newY = []
    for i in range(length):
        x2 = x[i]
        y2 = y[i]
        # tangential slope approximation
        try:
            slope = (y2 - y1) / (x2 - x1)
            # perpendicular slope
            pslope = -1/slope  # (might be 1/slope depending on direction of travel)
        except ZeroDivisionError:
            continue
        mid_x = (x1 + x2) / 2
        mid_y = (y1 + y2) / 2

        sign = ((pslope > 0) == (x1 > x2)) * 2 - 1

        delta_x = sign * z / ((1 + pslope**2)**0.5)
        delta_y = pslope * delta_x

        newX.append(mid_x + delta_x) 
        newY.append(mid_y + delta_y)
        x1, y1 = x2, y2
    return newX, newY


left_lane_x, left_lane_y = offset(global_plan_x, global_plan_y, 5)
right_lane_x, right_lane_y = offset(global_plan_x, global_plan_y, -5)

plt.plot(left_lane_x, left_lane_y, linestyle = 'solid', color = 'blue')
plt.plot(right_lane_x, right_lane_y, linestyle = 'solid', color = 'blue')


#plt.xlim(-300, 200)
#plt.ylim(-450,-350)


plt.grid(True)

plt.show()




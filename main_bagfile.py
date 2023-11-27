from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
from pathlib import Path
from rosbags.typesys import get_types_from_msg, register_types
import matplotlib.pyplot as plt
import numpy as np
import sys
import argparse

parser = argparse.ArgumentParser(
                    prog='pose_live_graph.py',
                    description='Graphs plots of position from odomtery data vs. ekf2 generated data',
                    epilog='Text at the bottom of help')

parser.add_argument('-bf', '--bagfolder')
parser.add_argument('-o', '--out')
parser.add_argument('-ao', '--angleoffset')

args = parser.parse_args()

if args.bagfolder is None:
    print("Add path to bag files: -bf <path> or --bagfolder <path>")
    sys.exit()
if args.out is None:
    print("add output file name for figures: -o <f_name> or --out <f_name>")
    sys.exit()
if args.angleoffset is None:
    angle_offset = 0
else:
    angle_offset = args.angleoffset

# plain dictionary to hold message definitions
add_types = {}
msg_files = [
    'VehicleLocalPosition',
    'SensorGps'
]

# add message types
for file in msg_files:
    msg = Path(file + ".msg").read_text()
    add_types.update(get_types_from_msg(msg, 'px4_msgs/msg/' + file))

# add custom msgs to rosbags type system
register_types(add_types)

local_position_array = []  # np.zeros(shape=(1019, 3))
odometry_array = []  # np.zeros(shape=(713, 3))

if len(sys.argv) < 2:
    print("Add bag file path")
    sys.exit()

# create reader instance and open for reading
with Reader(args.bagfolder) as reader:
    # topic and msgtype information is available on .connections list
    for connection in reader.connections:
        # print(connection.topic, connection.msgtype)
        pass
    # iterate over messages
    for connection, timestamp, rawdata in reader.messages():
        if connection.topic == '/zed/zed_node/odom':
            msg = deserialize_cdr(rawdata, connection.msgtype)
            odometry_array.append([msg.header.stamp.sec + (1e-9 * msg.header.stamp.nanosec), msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])

        if connection.topic == '/fmu/out/vehicle_local_position':
            msg = deserialize_cdr(rawdata, connection.msgtype)
            local_position_array.append([msg.timestamp, msg.x, msg.y, msg.z])

local_position_array = np.stack(local_position_array)  # By default properly sorted by time
odometry_array = np.stack(odometry_array)

local_position_array[:, 0] *= 1e-6  # from microseconds to seconds (epoch time)
start_time = local_position_array[0, 0]

# Time starts at 0
local_position_array[:, 0] -= start_time
odometry_array[:, 0] -= start_time
print(f"start_time {start_time}")
print(local_position_array.shape, local_position_array[0:5, 0])
print(odometry_array.shape, odometry_array[0:5, 0])

# print(list(local_position_array[:, 0]).index(min(local_position_array[:, 0])))
# print(list(local_position_array[:, 0]).index(max(local_position_array[:, 0])))

# normalize position (half the points should be above 0 and half below)
# no change of scale though
local_position_array[:, 1] -= local_position_array[:, 1].mean()
local_position_array[:, 2] -= local_position_array[:, 2].mean()
local_position_array[:, 3] -= local_position_array[:, 3].mean()

odometry_array[:, 1] -= odometry_array[:, 1].mean()
odometry_array[:, 2] -= odometry_array[:, 2].mean()
odometry_array[:, 3] -= odometry_array[:, 3].mean()

fig, ax = plt.subplots(2, 2)

for i in range(2):
    for j in range(2):

        if i == 0 and j==0:
            ax[i][j].set_xlim(-10, 10)
            ax[i][j].set_ylim(-10, 10)

        # ax[i][j].set_xlim
        # ax[i][j].set_ylim

theta = int(angle_offset) * (np.pi/180)  # 135 degrees to radians
rotation_matrix = np.array([[np.cos(theta), np.sin(theta)], [-np.sin(theta), np.cos(theta)]], dtype=np.float32)

x = -local_position_array[:, 1]
y = local_position_array[:, 2]

mean_x = np.mean(x)
mean_y = np.mean(y)

x -= mean_x
y -= mean_y

z = rotation_matrix @ np.array([x, y])

x = z[0] + mean_x
y = z[1] + mean_y

# ax[0][0].plot(local_position_array[:, 1], local_position_array[:, 2], label='local_position')
ax[0][0].plot(x, y, label='local_position')
ax[0][0].plot(odometry_array[:, 1], odometry_array[:, 2], label='odometry')
ax[0][0].title.set_text("XY Plot of Movement")
ax[0][0].set_xlabel("x")
ax[0][0].set_ylabel("y")

ax[0][1].plot(local_position_array[:, 0], local_position_array[:, 1], label='local_position_x')
ax[0][1].plot(odometry_array[:, 0], odometry_array[:, 1], label='odometry_x')
ax[0][1].title.set_text("Time vs X")
ax[0][1].set_xlabel("t")
ax[0][1].set_ylabel("x")

ax[1][0].plot(local_position_array[:, 0], local_position_array[:, 2], label='local_position_y')
ax[1][0].plot(odometry_array[:, 0], odometry_array[:, 2], label='odometry_y')
ax[1][0].title.set_text("Time vs Y")
ax[1][0].set_xlabel("t")
ax[1][0].set_ylabel("y")

ax[1][1].plot(local_position_array[:, 0], local_position_array[:, 3], label='local_position_z')
ax[1][1].plot(odometry_array[:, 0], odometry_array[:, 3], label='odometry_z')
ax[1][1].title.set_text("Time vs Z")
ax[1][1].set_xlabel("t")
ax[1][1].set_ylabel("z")

fig.text(0, 0, f"angle offset: {theta*(180/np.pi):.2f} degrees", color='red')
fig.legend(["local_position", "odometry"], loc="lower right")

# set the spacing between subplots
fig.tight_layout()

plt.savefig(f"2D_LocalPosition_vs_Odometry_{args.out}")

ax = plt.figure().add_subplot(projection='3d')
# ax.plot(local_position_array[:, 1], local_position_array[:, 2], local_position_array[:, 3], label='local_position')
ax.plot(x, y, local_position_array[:, 3], label='local_position')
ax.plot(odometry_array[:, 1], odometry_array[:, 2], odometry_array[:, 3], label='odometry')

ax.text(0, -5, -5, f"angle offset: {theta*(180/np.pi):.2f} degrees", color='red')

ax.legend()

plt.savefig(f"3D_LocalPosition_vs_Odometry_{args.out}")
plt.show()
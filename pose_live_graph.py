import roslibpy
import argparse
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

client = roslibpy.Ros('10.42.0.1', 9090)
client.run()
print(client.is_connected)

vlc_subscriber = roslibpy.Topic(
    client,
    '/fmu/out/vehicle_local_position',
    'px4_msgs/msg/VehicleLocalPosition'
)

odom_subscriber = roslibpy.Topic(
    client,
    '/zed/zed_node/odom',
    'nav_msgs/msg/Odometry'
)

vlc_x, vlc_y, vlc_z, vlc_time, odom_x, odom_y, odom_z, odom_time = [], [], [], [], [], [], [], []
odom_center_x, odom_center_y, odom_center_z = 0, 0, 0
init_values_set = False

def odom_callback(msg) -> None:
    global init_values_set, odom_center_x, odom_center_y, odom_center_z
    # print('\n', msg['pose']['pose']['position']['x'])
    # print(msg['pose']['pose']['position']['y'])
    # print(msg['pose']['pose']['position']['z'], '\n')

    odom_time.append(msg['header']['stamp']['sec'] + 1e-9 * msg['header']['stamp']['nanosec'])
    odom_x.append(msg['pose']['pose']['position']['x'])
    odom_y.append(-1 * msg['pose']['pose']['position']['y'])
    odom_z.append(-1 * msg['pose']['pose']['position']['z'])

    if not init_values_set:
        odom_center_x, odom_center_y, odom_center_z = odom_x[-1], odom_y[-1], odom_z[-1]
        init_values_set = True

def vlc_callback(msg) -> None:
    # print(msg)
    vlc_time.append(msg['timestamp'] * 1e-6)
    vlc_x.append(msg['x'])
    vlc_y.append(msg['y'])
    vlc_z.append(msg['z'])


# Like and Subscribe
vlc_subscriber.subscribe(vlc_callback)
odom_subscriber.subscribe(odom_callback)

# Create figure for plotting
fig, ax = plt.subplots(2, 2)
fig.tight_layout()
# This function is called periodically from FuncAnimation

# ax_3d = plt.figure().add_subplot(projection='3d')

def animate(i):
    # Draw x and y lists
    ax[0][0].clear()
    ax[0][1].clear()
    ax[1][0].clear()
    ax[1][1].clear()

    ax[0][0].plot(vlc_x, vlc_y, label='local_position')
    ax[0][0].plot(odom_x, odom_y, label='odometry')
    ax[0][0].title.set_text("XY Plot of Movement")
    ax[0][0].set_xlabel("x")
    ax[0][0].set_ylabel("y")

    ax[0][1].plot(vlc_time, vlc_x, label='local_position_x')
    ax[0][1].plot(odom_time, odom_x, label='odometry_x')
    ax[0][1].title.set_text("Time vs X")
    ax[0][1].set_xlabel("t")
    ax[0][1].set_ylabel("x")

    ax[1][0].plot(vlc_time, vlc_y, label='local_position_y')
    ax[1][0].plot(odom_time, odom_y, label='odometry_y')
    ax[1][0].title.set_text("Time vs Y")
    ax[1][0].set_xlabel("t")
    ax[1][0].set_ylabel("y")

    ax[1][1].plot(vlc_time, vlc_z, label='local_position_z')
    ax[1][1].plot(odom_time, odom_z, label='odometry_z')
    ax[1][1].title.set_text("Time vs Z")
    ax[1][1].set_xlabel("t")
    ax[1][1].set_ylabel("z")

    ax[0][0].set_xlim(odom_center_x + 1.5, odom_center_x - 1.5)
    ax[0][0].set_ylim(odom_center_y + 1.5, odom_center_y - 1.5)

# Set up plot to call animate() function periodically
ani = animation.FuncAnimation(fig, animate, interval=20)
plt.show()

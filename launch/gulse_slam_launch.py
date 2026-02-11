import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # --- YOL TANIMLARI ---
    package_dir = get_package_share_directory('gulse_scan_driver')
    urdf_path = os.path.join(package_dir, 'resource', 'gulse_robot.urdf')
    
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        # A. Robot State Publisher: URDF modelini ve eklemleri yayınlar
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_desc, 
                'use_sim_time': False # PC Saati Stratejisi
            }]
        ),

        # B. Webots Driver: Webots simülasyonu ile ROS 2 arasındaki köprü
        Node(
            package='webots_ros2_driver',
            executable='driver',
            output='screen',
            parameters=[{
                'robot_description': urdf_path,
                'use_sim_time': False,
                'set_robot_state_publisher': False, # Çakışmayı önleyen kritik ayar
            }]
        ),

        # C. Köprü (Bridge): GPS ve IMU verilerinden odom hesabı yapar
        Node(
            package='gulse_scan_driver',
            executable='gps_bridge',
            name='gps_bridge',
            output='screen',
            parameters=[{'use_sim_time': False}]
        )

        # SLAM Toolbox buradan kaldırıldı; artık panel.py üzerinden yönetilecek!
    ])
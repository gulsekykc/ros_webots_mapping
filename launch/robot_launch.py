import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_controller import WebotsController

def generate_launch_description():
    package_name = 'gulse_scan_driver'
    package_dir = get_package_share_directory(package_name)
    world = os.path.join(package_dir, 'worlds', 'GaziSim_Alpha.wbt')
    
    # URDF dosyasının tam yolunu bulalım
    robot_description_path = os.path.join(package_dir, 'resource', 'gulse_robot.urdf')

    # Segfault hatasını önlemek için URDF içeriğini string olarak okuyoruz
    with open(robot_description_path, 'r') as f:
        robot_description_content = f.read()

    # 1. Webots'u başlatan aksiyon
    webots = ExecuteProcess(
        cmd=['/usr/local/webots/webots', '--mode=realtime', world],
        output='screen'
    )

    # 2. Robot sürücüsü aksiyonu
    gulse_robot_driver = WebotsController(
        robot_name='GulseScan',
        parameters=[
            {'robot_description': robot_description_content},
        ]
    )

    # 3. Sürücüyü Webots'tan 10 saniye sonra başlatacak zamanlayıcı
    delayed_driver = TimerAction(
        period=10.0,
        actions=[gulse_robot_driver]
    )

    return LaunchDescription([
        webots,
        delayed_driver,
    ])

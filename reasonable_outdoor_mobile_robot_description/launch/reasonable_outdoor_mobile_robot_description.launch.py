import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
import launch
import launch_ros.actions
import xacro
from launch_ros.substitutions import FindPackageShare

share_dir_path = os.path.join(get_package_share_directory('reasonable_outdoor_mobile_robot_description'))
xacro_path = os.path.join(share_dir_path, 'robots', 'reasonable_outdoor_mobile_robot.urdf.xacro')
urdf_path = os.path.join(share_dir_path, 'robots', 'reasonable_outdoor_mobile_robot.urdf')

def generate_launch_description():
    # xacroをロード
    doc = xacro.process_file(xacro_path)
    # xacroを展開してURDFを生成
    robot_desc = doc.toprettyxml(indent='  ')
    # urdf_pathに対してurdfを書き出し
    f = open(urdf_path, 'w')
    f.write(robot_desc)
    f.close()
    rsp = launch_ros.actions.Node(package='robot_state_publisher',
                                  executable='robot_state_publisher',
                                  output='both',
                                  remappings=[('robot_description', 'robot_description')],
                                  # argumentsでURDFを出力したパスを指定
                                  arguments=[urdf_path])
    jsp = launch_ros.actions.Node(package='joint_state_publisher_gui',
                                  executable='joint_state_publisher_gui',
                                  output='both',
                                  remappings=[('robot_description', 'robot_description')],
                                  # argumentsでURDFを出力したパスを指定
                                  arguments=[urdf_path])

    return launch.LaunchDescription([rsp, jsp])

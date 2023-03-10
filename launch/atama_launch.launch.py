from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    firstPackage = LaunchConfiguration('firstPackage')
    secondPackage = LaunchConfiguration('secondPackage')
    thirdPackage = LaunchConfiguration('thirdPackage')
    fourthPackage = LaunchConfiguration('fourthPackage')
    mode = LaunchConfiguration('mode')
    port = LaunchConfiguration('port')
    
    firstPackage_launch_arg = DeclareLaunchArgument(
        'firstPackage',
        default_value='atama'
    )
    secondPackage_launch_arg = DeclareLaunchArgument(
        'secondPackage',
        default_value='tachimawari'
    )
    thirdPackage_launch_arg = DeclareLaunchArgument(
        'thirdPackage',
        default_value='aruku'
    )
    fourthPackage_launch_arg = DeclareLaunchArgument(
        'fourthPackage',
        default_value='kansei'
    )
    mode_launch_arg = DeclareLaunchArgument(
        'mode',
        default_value=None,
        description='choose mode between sdk/cm740!',
        choices=['sdk', 'cm740']
    )
    port_launch_arg = DeclareLaunchArgument(
        'port',
        default_value=['/dev/ttyUSB1'],
        description='choose port name if /dev/ttyUSB0 not working!'
    )
    
    firstNode = Node(
        package=firstPackage,
        executable='main',
        name='atama',
        arguments=['./src/atama/data/'],
        output='screen'
    )
    secondNode = Node(
        package=secondPackage,
        executable='main',
        name='tachimawari',
        arguments=[mode],
        output='screen'
    )
    thirdNode = Node(
        package=thirdPackage,
        executable='main',
        name='aruku',
        arguments=['./src/aruku/data/'],
        output='screen'
    )
    fourthNode = Node(
        package=fourthPackage,
        executable='main',
        name='kansei',
        arguments=[port],
        output='screen'
    )

    return LaunchDescription([
        firstPackage_launch_arg,
        secondPackage_launch_arg,
        thirdPackage_launch_arg,
        fourthPackage_launch_arg,
        mode_launch_arg,
        port_launch_arg,
        firstNode,
        secondNode,
        thirdNode,
        fourthNode
    ])
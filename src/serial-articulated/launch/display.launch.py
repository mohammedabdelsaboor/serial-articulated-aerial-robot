import os
import tempfile
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('serial-articulated')
    # Parent dir so gz resolves model://serial-articulated/meshes/...
    resource_path = os.path.dirname(pkg_share)

    # Process dragon.xacro → URDF string (uses dragon_arm.xacro macro)
    xacro_path = os.path.join(pkg_share, 'urdf', 'dragon.xacro')
    dragon_arm_path = os.path.join(pkg_share, 'urdf', 'dragon_arm.xacro')
    robot_description = xacro.process_file(
        xacro_path,
        mappings={'dragon_arm_file': dragon_arm_path},
    ).toxml()

    # Write processed URDF to a temp file so gz sim can load it via file://
    tmp_urdf = tempfile.NamedTemporaryFile(
        mode='w', suffix='.urdf', delete=False, prefix='dragon_processed_'
    )
    tmp_urdf.write(robot_description)
    tmp_urdf.flush()
    urdf_path = tmp_urdf.name

    # Build a world SDF that loads the URDF directly via file:// URI
    world_content = f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="dragon_world">

    <plugin filename="gz-sim-physics-system"
            name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system"
            name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster"/>

    <!-- No gravity so the robot hovers in place without a flight controller -->
    <gravity>0 0 0</gravity>

    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- Welding wall: face at Y=1.3995, weld line X=-0.685..−0.215 at Z=1.5 -->
    <model name="weld_wall">
      <static>true</static>
      <pose>-0.45 1.4245 1.5 0 0 0</pose>
      <link name="wall_link">
        <collision name="collision">
          <geometry><box><size>1.2 0.05 2.0</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>1.2 0.05 2.0</size></box></geometry>
          <material>
            <ambient>0.8 0.6 0.4 1</ambient>
            <diffuse>0.8 0.6 0.4 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane><normal>0 0 1</normal><size>100 100</size></plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane><normal>0 0 1</normal><size>100 100</size></plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <include>
      <uri>file://{urdf_path}</uri>
      <name>dragon</name>
      <pose>0 0 1.5 0 0 0</pose>

      <!-- Arm joint position controllers -->
      <plugin filename="gz-sim-joint-position-controller-system"
              name="gz::sim::systems::JointPositionController">
        <joint_name>joint1_dummy</joint_name>
        <topic>/arm/joint1/cmd_pos</topic>
        <p_gain>100</p_gain>
        <i_gain>0.1</i_gain>
        <d_gain>10</d_gain>
        <i_max>1</i_max>
        <i_min>-1</i_min>
        <cmd_max>100</cmd_max>
        <cmd_min>-100</cmd_min>
      </plugin>
      <plugin filename="gz-sim-joint-position-controller-system"
              name="gz::sim::systems::JointPositionController">
        <joint_name>joint2_dummy</joint_name>
        <topic>/arm/joint2/cmd_pos</topic>
        <p_gain>100</p_gain>
        <i_gain>0.1</i_gain>
        <d_gain>10</d_gain>
        <i_max>1</i_max>
        <i_min>-1</i_min>
        <cmd_max>100</cmd_max>
        <cmd_min>-100</cmd_min>
      </plugin>
      <plugin filename="gz-sim-joint-position-controller-system"
              name="gz::sim::systems::JointPositionController">
        <joint_name>Joint3_dummy</joint_name>
        <topic>/arm/joint3/cmd_pos</topic>
        <p_gain>100</p_gain>
        <i_gain>0.1</i_gain>
        <d_gain>10</d_gain>
        <i_max>1</i_max>
        <i_min>-1</i_min>
        <cmd_max>100</cmd_max>
        <cmd_min>-100</cmd_min>
      </plugin>

      <!-- Spin propellers: right CW (+50), left CCW (-50) -->
      <plugin filename="gz-sim-joint-controller-system"
              name="gz::sim::systems::JointController">
        <joint_name>link1_right_prop</joint_name>
        <initial_velocity>50.0</initial_velocity>
      </plugin>
      <plugin filename="gz-sim-joint-controller-system"
              name="gz::sim::systems::JointController">
        <joint_name>link1_left_prop</joint_name>
        <initial_velocity>-50.0</initial_velocity>
      </plugin>
      <plugin filename="gz-sim-joint-controller-system"
              name="gz::sim::systems::JointController">
        <joint_name>link2_right_prop</joint_name>
        <initial_velocity>50.0</initial_velocity>
      </plugin>
      <plugin filename="gz-sim-joint-controller-system"
              name="gz::sim::systems::JointController">
        <joint_name>link2_left_prop</joint_name>
        <initial_velocity>-50.0</initial_velocity>
      </plugin>
      <plugin filename="gz-sim-joint-controller-system"
              name="gz::sim::systems::JointController">
        <joint_name>link3_right_prop</joint_name>
        <initial_velocity>50.0</initial_velocity>
      </plugin>
      <plugin filename="gz-sim-joint-controller-system"
              name="gz::sim::systems::JointController">
        <joint_name>link3_left_prop</joint_name>
        <initial_velocity>-50.0</initial_velocity>
      </plugin>
      <plugin filename="gz-sim-joint-controller-system"
              name="gz::sim::systems::JointController">
        <joint_name>link4_right_prop</joint_name>
        <initial_velocity>50.0</initial_velocity>
      </plugin>
      <plugin filename="gz-sim-joint-controller-system"
              name="gz::sim::systems::JointController">
        <joint_name>link4_left_prop</joint_name>
        <initial_velocity>-50.0</initial_velocity>
      </plugin>
    </include>

  </world>
</sdf>"""

    # Write to a temp file so gz sim can read it
    tmp_world = tempfile.NamedTemporaryFile(
        mode='w', suffix='.sdf', delete=False, prefix='dragon_world_'
    )
    tmp_world.write(world_content)
    tmp_world.flush()
    world_file = tmp_world.name

    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=resource_path,
    )

    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'arm.rviz')],
    )

    # Set robot pose via CLI after Gazebo has had time to fully start.
    # The <pose> tag in <include> is ignored for URDF files by Gazebo Harmonic,
    # so we force it here using the set_pose service.
    set_initial_pose = TimerAction(
        period=6.0,
        actions=[ExecuteProcess(
            cmd=[
                'gz', 'service', '-s', '/world/dragon_world/set_pose',
                '--reqtype', 'gz.msgs.Pose',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '2000',
                '--req',
                'name: "dragon" '
                'position: {x: 0.0, y: 1.3995, z: 1.5} '
                'orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}'
            ],
            output='screen',
        )]
    )

    return LaunchDescription([
        set_gz_resource_path,
        gz_sim,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz,
        set_initial_pose,
    ])

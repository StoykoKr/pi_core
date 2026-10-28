import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('robot_bringup')

    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    slam_params_file = os.path.join(pkg_share, 'config', 'slam_toolbox_params.yaml')

    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    return LaunchDescription([

        DeclareLaunchArgument('use_sim_time', default_value='true'),

        Node(
            package='robot_bringup',
            executable='spatial_projector',
            name='spatial_projector_node',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                # camera_height is only a fallback; TF value wins.
                'camera_height': 1.3,
                'camera_vfov_deg': 90.0,
                'max_range': 4.0,
                'near_lockout_range': 0.6,
                # 0.35, not 0.20: a mere 15% depth error on a floor pixel
                # projects to ~0.2 m height (camera_height * rel_error), so
                # 0.20 turned depth noise into obstacles. The 0.0-0.30 m
                # band is the LIDAR's job (max_obstacle_height 0.3), so
                # nothing is lost by starting the camera band at 0.35.
                'band_z_min': 0.35,
                'band_z_max': 1.70,
                # 0.25 base, dynamic multiplier caps at 2x inside the crop
                # -> up to 50% tolerance for warped floor. Safe because the
                # vertical-structure rescue exempts walls/legs/edges from
                # ground rejection, and a tabletop under a 1.3 m camera is
                # ~58% off the floor-depth curve, still outside 50%.
                'ground_rej_rel': 0.25,
                'vert_check_rows': 12,
                'vert_rel_tol': 0.05,
                # Clearing rays stop this far short of believed surfaces.
                'clearing_endpoint_margin': 0.15,
                # Two-tier gating: strict gate for UPDATING the alignment,
                # loose gate for CONTINUING to publish with the smoothed
                # model on frames where the per-frame fit is degenerate
                # (wire racks, overhangs -- exactly where marking matters).
                'residual_gate': 0.20,
                'publish_residual_gate': 0.35,
                'fit_alpha_stable': 0.10,
                'optical_frame': 'front_camera_depth_optical_link',#'front_camera_link' ,#'front_camera_depth_optical_link',
                'base_frame': 'base_link',
            }],
            remappings=[
                # 1. Map the internal raw image subscriber to O3DE's RGB stream
                ('/image_raw', '/camera_image_color'),
                # 2. Real intrinsics from O3DE instead of the vfov fallback.
                # The fallback assumes fx == fy, but 640x480 resized to
                # 320x256 is NOT aspect-preserving (fx ~120 vs fy 128), so
                # without this every point was ~7% off laterally -> extra
                # curvature at the sides. Verify the topic name with
                # `ros2 topic list` (O3DE camera "Color Camera Info" topic).
                ('camera_info_in', '/color_camera_info'),
            ]
        ),
        # Persistent map-frame memory of camera-detected obstacles. The
        # global costmap's camera_layer marks from this node's output
        # (/camera/obstacle_memory) so that lidar-invisible obstacles are
        # remembered even after the local window scrolls away, the SLAM map
        # resizes, or a costmap gets cleared.
        Node(
            package='robot_bringup',
            executable='camera_memory',
            name='camera_memory',
            output='screen',
            # Safety-relevant node: if it ever dies, the camera marking
            # pipeline silently disappears. Respawn keeps it alive; the
            # crash still shows in the log, so bugs stay visible.
            respawn=True,
            respawn_delay=2.0,
            parameters=[{
                'use_sim_time': True,
                'map_frame': 'map',
                # THE BUG: these were never set, so camera_memory looked up
                # TF to the default 'camera_optical_frame' -- a frame that
                # does not exist -- threw on EVERY cloud callback, silently
                # returned, and therefore never ingested a single point.
                # Every camera mark ever seen in the costmaps came from the
                # (since removed) raw camera_live source, which is why the
                # memory pipeline's death was invisible until now.
                'optical_frame': 'front_camera_depth_optical_link',
                'base_frame': 'base_link',
                # WAS 'voxel_size' -- the node declares voxel_size_xy /
                # voxel_size_z, so the old value was silently ignored.
                'voxel_size_xy': 0.10,
                'voxel_size_z': 0.15,
                # OccupancyGrid mirror consumed by the global costmap's
                # camera_memory_layer (StaticLayer). Geometry copied from
                # the SLAM map so the two static layers never disagree.
                'publish_grid': True,
                'slam_map_topic': '/map',
                'grid_topic': '/camera/memory_grid',
                # A mark must be seen in 3 frames >= 0.4 s apart before it
                # is published: one-frame depth glitches never reach the
                # costmap. Real obstacles are confirmed in ~1.5 s of view.
                'min_hits': 3,
                'min_hit_separation': 0.4,
                # Miss decay throttled to once per interval (frame-rate
                # independent); matches min_hit_separation cadence.
                'fov_clear_interval': 0.4,
                # Unconfirmed candidates are dropped after this long.
                'unconfirmed_timeout': 30.0,
                # Confirmed marks self-heal after 10 min unless the camera
                # re-confirms them. Long enough to remember the
                # lidar-invisible table across the whole map; short enough
                # that a phantom (or a moved box) does not block an aisle
                # forever. Set 0.0 to never forget.
                'decay_seconds': 600.0,
                'publish_rate': 2.0,
                'max_points': 200000,
            }],
        ),

        Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan',
            output='screen',
            remappings=[
                ('depth', '/camera/depth'),
                ('depth_camera_info', '/camera/camera_info'),
                ('scan', '/camera_scan')
            ],
            parameters=[{
                'output_frame': 'base_link', #'front_camera_link', #stupid shit is kinda broken cus of O3DE I think
                'scan_height': 180,
                'range_min': 0.4,
                'range_max': 4.0,
                'use_sim_time': True
            }]
        ),
        
        Node(
    package='nav2_lifecycle_manager',
    executable='lifecycle_manager',
    name='lifecycle_manager_slam',
    output='screen',
    parameters=[
        {'use_sim_time': True},
        {'autostart': True},
        {'node_names': ['slam_toolbox']} # Manages ONLY slam_toolbox without bonds
    ]
),

        Node(
            parameters=[
                slam_params_file,
                {'use_sim_time': True}
            ],
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen'
        ),

       Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_optical_tf_publisher',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0', 
                '--yaw', '0',   #-1.57079632679 is the usual transformation but cus O3DE gives already transformed..
                '--pitch', '0', 
                '--roll', '0',  #-1.57079632679 is the usual transformation but cus O3DE gives already transformed..
                '--frame-id', 'front_camera_link', 
                '--child-frame-id', 'front_camera_depth_optical_link'
            ],
            parameters=[{'use_sim_time': True}]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'true',
                'params_file': nav2_params_file
            }.items()
        )
    ])
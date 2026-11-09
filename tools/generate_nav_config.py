from pathlib import Path
import yaml
p = Path('src/landerpi_simulation/config')
w = h = 120
cells = bytearray([254] * (w * h))
for yy in range(h):
    for xx in range(w):
        x = -2 + xx * 0.05
        y = -2 + (h - 1 - yy) * 0.05
        if xx in (0, w - 1) or yy in (0, h - 1) or (0.6 <= x <= 0.9 and (abs(y) <= 0.2 or abs(y - 1.5) <= 0.2)):
            cells[yy * w + xx] = 0
(p / 'sorting_map.pgm').write_bytes(f'P5\n{w} {h}\n255\n'.encode() + cells)
(p / 'sorting_map.yaml').write_text(yaml.safe_dump(dict(image='sorting_map.pgm', resolution=0.05, origin=[-2.0, -2.0, 0.0], negate=0, occupied_thresh=0.65, free_thresh=0.25)))
params = {}

def node(name, values):
    params[name] = {'ros__parameters': {'use_sim_time': True, **values}}
node('planner_server', dict(expected_planner_frequency=2.0, planner_plugins=['GridBased'], GridBased={'plugin': 'nav2_navfn_planner/NavfnPlanner', 'tolerance': 0.02, 'use_astar': False, 'allow_unknown': False}))
node('controller_server', dict(controller_frequency=8.0, min_x_velocity_threshold=0.001, min_y_velocity_threshold=0.001, min_theta_velocity_threshold=0.001, failure_tolerance=1.0, progress_checker_plugin='progress_checker', goal_checker_plugins=['goal_checker'], controller_plugins=['FollowPath'], progress_checker={'plugin': 'nav2_controller::SimpleProgressChecker', 'required_movement_radius': 0.01, 'movement_time_allowance': 20.0}, goal_checker={'plugin': 'nav2_controller::SimpleGoalChecker', 'xy_goal_tolerance': 0.04, 'yaw_goal_tolerance': 0.05, 'stateful': True}, FollowPath={'plugin': 'landerpi_simulation/HolonomicController', 'min_vel_x': -0.15, 'min_vel_y': -0.15, 'max_vel_x': 0.2, 'max_vel_y': 0.2, 'max_vel_theta': 0.4, 'min_speed_xy': 0.0, 'max_speed_xy': 0.22, 'min_speed_theta': 0.0, 'acc_lim_x': 0.5, 'acc_lim_y': 0.5, 'acc_lim_theta': 1.0, 'decel_lim_x': -0.5, 'decel_lim_y': -0.5, 'decel_lim_theta': -1.0, 'vx_samples': 5, 'vy_samples': 5, 'vtheta_samples': 6, 'sim_time': 1.2, 'linear_granularity': 0.03, 'angular_granularity': 0.04, 'transform_tolerance': 0.3, 'xy_goal_tolerance': 0.04, 'trans_stopped_velocity': 0.03, 'short_circuit_trajectory_evaluation': True, 'stateful': True, 'critics': ['RotateToGoal', 'Oscillation', 'BaseObstacle', 'GoalAlign', 'PathAlign', 'PathDist', 'GoalDist'], 'BaseObstacle.scale': 0.05, 'PathAlign.scale': 0.0, 'PathAlign.forward_point_distance': 0.1, 'GoalAlign.scale': 0.0, 'GoalAlign.forward_point_distance': 0.1, 'PathDist.scale': 24.0, 'GoalDist.scale': 24.0, 'RotateToGoal.scale': 24.0, 'RotateToGoal.slowing_factor': 5.0, 'RotateToGoal.lookahead_time': -1.0}))
for (name, frame, rolling) in [('local_costmap', 'odom', True), ('global_costmap', 'map', False)]:
    values = dict(use_sim_time=True, global_frame=frame, robot_base_frame='base_footprint', update_frequency=3.0, publish_frequency=2.0, resolution=0.05, robot_radius=0.12, rolling_window=rolling, track_unknown_space=False, transform_tolerance=0.3, plugins=['static_layer', 'obstacle_layer', 'inflation_layer'], static_layer={'plugin': 'nav2_costmap_2d::StaticLayer', 'map_subscribe_transient_local': True}, obstacle_layer={'plugin': 'nav2_costmap_2d::ObstacleLayer', 'observation_sources': 'scan', 'scan': {'topic': '/scan', 'data_type': 'LaserScan', 'clearing': True, 'marking': True, 'max_obstacle_height': 2.0, 'raytrace_max_range': 3.0, 'obstacle_max_range': 2.5}}, inflation_layer={'plugin': 'nav2_costmap_2d::InflationLayer', 'cost_scaling_factor': 10.0, 'inflation_radius': 0.2}, always_send_full_costmap=True)
    if rolling:
        values.update(width=3, height=3)
    params[name] = {name: {'ros__parameters': values}}
node('bt_navigator', dict(global_frame='map', robot_base_frame='base_footprint', odom_topic='/odom', bt_loop_duration=100, default_server_timeout=1000, plugin_lib_names=['nav2_compute_path_to_pose_action_bt_node', 'nav2_follow_path_action_bt_node', 'nav2_rate_controller_bt_node', 'nav2_pipeline_sequence_bt_node', 'nav2_recovery_node_bt_node', 'nav2_clear_costmap_service_bt_node', 'nav2_goal_updated_condition_bt_node', 'nav2_round_robin_node_bt_node', 'nav2_spin_action_bt_node', 'nav2_wait_action_bt_node', 'nav2_back_up_action_bt_node', 'nav2_is_stuck_condition_bt_node', 'nav2_goal_reached_condition_bt_node', 'nav2_initial_pose_received_condition_bt_node', 'nav2_reinitialize_global_localization_service_bt_node', 'nav2_distance_controller_bt_node', 'nav2_speed_controller_bt_node', 'nav2_truncate_path_action_bt_node', 'nav2_goal_updater_node_bt_node', 'nav2_single_trigger_bt_node', 'nav2_navigate_to_pose_action_bt_node', 'nav2_navigate_through_poses_action_bt_node', 'nav2_compute_path_through_poses_action_bt_node', 'nav2_remove_passed_goals_action_bt_node', 'nav2_transform_available_condition_bt_node', 'nav2_time_expired_condition_bt_node', 'nav2_distance_traveled_condition_bt_node']))
node('behavior_server', dict(costmap_topic='local_costmap/costmap_raw', footprint_topic='local_costmap/published_footprint', cycle_frequency=10.0, behavior_plugins=['spin', 'backup', 'wait'], spin={'plugin': 'nav2_behaviors/Spin'}, backup={'plugin': 'nav2_behaviors/BackUp'}, wait={'plugin': 'nav2_behaviors/Wait'}, global_frame='odom', robot_base_frame='base_footprint', transform_tolerance=0.3, simulate_ahead_time=2.0, max_rotational_vel=0.4, min_rotational_vel=0.1, rotational_acc_lim=1.0))
(p / 'nav2.yaml').write_text(yaml.safe_dump(params, sort_keys=False))

import yaml
LENGTH, WIDTH = 0.60, 0.40
src = '/opt/ros/jazzy/share/nav2_bringup/params/nav2_params.yaml'
out = '/home/ariyan/ros2_ws/src/hexapod_mapping/config/nav2_params.yaml'
p = yaml.safe_load(open(src))
def fix(o):
    if isinstance(o, dict):
        return {k: (False if k == 'use_sim_time' else fix(v)) for k, v in o.items()}
    if isinstance(o, list): return [fix(v) for v in o]
    return 'base_link' if o == 'base_footprint' else o
p = fix(p)
p['amcl']['ros__parameters']['laser_max_range'] = 12.0
c = p['controller_server']['ros__parameters']
c['controller_frequency'] = 10.0
c['progress_checker']['required_movement_radius'] = 0.1
c['progress_checker']['movement_time_allowance'] = 30.0
c['general_goal_checker']['xy_goal_tolerance'] = 0.25
c['general_goal_checker']['yaw_goal_tolerance'] = 0.5
c['FollowPath'] = {
  'plugin': 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
  'desired_linear_vel': 0.10, 'lookahead_dist': 0.4,
  'min_lookahead_dist': 0.3, 'max_lookahead_dist': 0.6,
  'use_velocity_scaled_lookahead_dist': False,
  'use_rotate_to_heading': True, 'rotate_to_heading_min_angle': 0.5,
  'rotate_to_heading_angular_vel': 0.5, 'max_angular_accel': 3.2,
  'allow_reversing': False,
  'use_regulated_linear_velocity_scaling': False,
  'use_cost_regulated_linear_velocity_scaling': False,
  'use_collision_detection': True,
  'max_allowed_time_to_collision_up_to_carrot': 1.5,
  'transform_tolerance': 0.5}
L, W = LENGTH / 2, WIDTH / 2
for cm in ('local_costmap', 'global_costmap'):
    q = p[cm][cm]['ros__parameters']
    q.pop('robot_radius', None)
    q['footprint'] = f'[[{L}, {W}], [{L}, -{W}], [-{L}, -{W}], [-{L}, {W}]]'
    if 'inflation_layer' in q:
        q['inflation_layer']['inflation_radius'] = 0.35
yaml.safe_dump(p, open(out, 'w'), sort_keys=False)
print('wrote', out)

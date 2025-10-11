"""
open the waypoints fiile

from there:

- get positions and orientations for given list of ids

- sort the positions based on distance from origin

- send the list to main_inspection script




"""

# import yaml

# with open("/home/qasob/go2_autonomous_inspection/src/go2_waypoint_nav/navigation_path/test_waypoint.yaml") as stream:
#         try:
#             data = yaml.safe_load(stream)
#         except yaml.YAMLError as exc:
#             print(exc)


# given_ids = [2, 6, 3]


def get_waypoint_by_id(waypoints_data, target_id):
    """
    Get position and orientation for a waypoint with the given ID
    """
    for waypoint in waypoints_data['waypoints']:
        if waypoint['pose']['object']['object_id'] == target_id:
            orientation = waypoint['pose']['orientation']
            position = waypoint['pose']['position']
            return (position, orientation, target_id)
    return None, None



def filtered_waypoints_by_ids(waypoints_data, id_list):
    waypoints_list = []
    for id in id_list:
        pose = get_waypoint_by_id(waypoints_data, id)
        waypoints_list.append(pose)


    print("Unsorted waypoints:", waypoints_list)
    # Sort waypoints based on distance from origin (0,0)
    sorted_waypoints = sorted(waypoints_list, key=lambda item: item[0]['x']**2 + item[0]['y']**2)

    return sorted_waypoints


def generate_object_inspection_points(waypoints):
    inspection_points = []
    for waypoint in waypoints:
        position, orientation, obj_id = waypoint
        x, y, z = position['x'], position['y'], position['z']
        theta = orientation['z']
        
        inspection_points.append({'type': 'waypoint_follower', 'x': x, 'y': y, 'z': z})
        # inspection_points.append({'type': 'rotate_robot', 'theta': theta})
        # inspection_points.append({'type': 'detect_closest', 'class_name': obj_id.split(':')[1], 'timeout': 20.0, 'confirm_frames': 3, 'require_depth': False})

    
    return inspection_points




import yaml

with open("/home/qasob/go2_autonomous_inspection/src/go2_waypoint_nav/navigation_path/test_waypoint.yaml") as stream:
        try:
            data = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)

given_ids = [2, 3, 6]  # List of IDs to filter for

def get_waypoint_by_id(waypoints_data, target_id):
    """
    Get position and orientation for a waypoint with the given ID
    """
    for waypoint in waypoints_data['waypoints']:
        if waypoint['pose']['object']['object_id'] == target_id:
            orientation = waypoint['pose']['orientation']
            position = waypoint['pose']['position']
            return position, orientation
    return None, None

def get_waypoints_by_ids(waypoints_data, id_list):
    """
    Get positions and orientations for all waypoints with IDs in the given list
    """
    filtered_waypoints = []
    filtered_orientations = []
    
    for target_id in id_list:
        position, orientation = get_waypoint_by_id(waypoints_data, target_id)
        if position is not None and orientation is not None:
            x, y, z = position['x'], position['y'], position['z']
            theta = orientation['z']
            
            filtered_waypoints.append({'type': 'waypoint_follower', 'x': x, 'y': y, 'z': z, 'id': target_id})
            filtered_orientations.append({'type': 'rotate_robot', 'theta': theta, 'id': target_id})
            
            print(f"ID {target_id}: Position({x:.3f}, {y:.3f}, {z:.3f}), Orientation theta={theta:.3f}")
        else:
            print(f"Warning: No waypoint found with ID {target_id}")
    
    return filtered_waypoints, filtered_orientations

# Get waypoints for the specified IDs
sequence = [{'type': 'rotate_robot', 'theta': 0.0}]
waypoints, orientations = get_waypoints_by_ids(data, given_ids)
        
combined_sequence = []
for wp, ori in zip(waypoints, orientations):
    combined_sequence.append(wp)
    combined_sequence.append(ori)

print(f"\nFiltered waypoints for IDs {given_ids}:")
print("Combined sequence:", combined_sequence)
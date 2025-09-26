import yaml

with open("/home/qasob/go2_autonomous_inspection/src/go2_waypoint_nav/navigation_path/test_waypoint.yaml") as stream:
        try:
            data = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)
    
orientation = data['waypoints'][0]['pose']['orientation']
position = data['waypoints'][0]['pose']['position']

theta = orientation['w']
x, y, z = position['x'], position['y'], position['z'] 
id = data['waypoints'][0]['pose']['object']['object_id']

sequence = [{'type': 'rotate_robot', 'theta': 0.0}]
waypoints = []
orientations = []

for i in range(len(data['waypoints'])):
    orientation = data['waypoints'][i]['pose']['orientation']
    position = data['waypoints'][i]['pose']['position']
    object_id = data['waypoints'][i]['pose']['object']

    theta = orientation['z']
    x, y, z = position['x'], position['y'], position['z'] 
    id = object_id['object_id']
    
    waypoints.append({'type': 'waypoint_follower', 'x': x, 'y': y, 'z': z})
    orientations.append({'type': 'rotate_robot', 'theta': theta})
        
combined_sequence = []
for wp, ori in zip(waypoints, orientations):
    combined_sequence.append(wp)
    combined_sequence.append(ori)

print("Waypoints:", combined_sequence)
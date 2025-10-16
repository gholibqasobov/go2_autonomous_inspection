from flask import Flask, render_template, request, jsonify, redirect, url_for, send_from_directory
import os
import yaml
import subprocess
import json
import config

app = Flask(__name__)

@app.route("/")
def index():
    with open(config.WAYPOINTS_PATH, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    points = data.get("waypoints", [])

    return render_template("index.html", points=points)

@app.route("/run", methods=["POST"])
def run():
    data = request.get_json()
    selected_points = data.get("points", [])

    print("Selected points:", selected_points)

    # Convert all point IDs to strings for command
    obj_ids = [str(i['pose']['object']['object_id']) for i in selected_points]

    # Build the ROS2 command
    cmd = ["ros2", "run", "go2_waypoint_nav", "inspection_client_main"] + obj_ids
    print("Running command:", " ".join(cmd))

    '''try:
        result = subprocess.run(cmd, capture_output=True, text=True, check=True)
        print("ROS2 output:\n", result.stdout)
        print("ROS2 errors:\n", result.stderr)
    except subprocess.CalledProcessError as e:
        print("ROS2 command failed:", e.stderr)'''
    
    data = {"obj_ids": obj_ids}
    print('data:     ', data)
    # Redirect to finish page after completion
    return redirect(url_for("result",  **data))



@app.route("/result")
def result():
    print(request.args)
    names = request.args.getlist("obj_ids")
    print('names:     ', names)
    result = {}
    runs={}
    with open(config.RUNS_PATH, "r", encoding="utf-8") as f:
        for line in f:
            item = json.loads(line)
            runs[item.get("target_id", "")]=item.get("image_file")
    for name in names:
        if name in runs.keys():
            result[name] = runs[name]
        else:
            result[name] = "NO IMAGE"
    print('result:     ', result)
    return render_template('result.html', images=result)

# Serve images through Flask's static route
@app.route('/images/<filename>')
def image(filename):
    if filename=='NO IMAGE':
        return send_from_directory('./', 'noimage.png')
    return send_from_directory(config.IMAGES_PATH, filename)

if __name__ == "__main__":
    # Run the app in debug mode for development
    app.run(host="0.0.0.0", port=6500, debug=True)
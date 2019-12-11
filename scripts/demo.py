#!/usr/bin/env python
from flask import Flask, render_template
from flask_cors import CORS
import threading
import rospy
from vision.msg import ObjectPosition
import json


class DetectedObject:
    """
    This class is used to efficiently store the objects until they are requested by the webserver
    """

    def __init__(self, name, x, y, z, distance):
        self.name = name
        self.x, self.y, self.z = x, y, z
        self.distance = distance

def callback(data):
    """
    This callback is called when a new message is recieved, creates a new DetectedObjects instance and adds it to the
    list if the list is full the item that was first added is removed

    :param data: Data recieved from ros message
    :return:
    """
    obj = DetectedObject(data.type, data.x, data.y, data.z, data.distance)

    if len(detected_objects) < 10:
        detected_objects.append(obj)
    else:
        detected_objects.pop(0)
        detected_objects.append(obj)

#init threaded ROS node so that it doesnt conflict with flask
threading.Thread(target=lambda: rospy.init_node("Object_listener", disable_signals=True, anonymous=True)).start()
sub = rospy.Subscriber("/vision/object_position", ObjectPosition, callback=callback)

#init  flask app and empty list for objects
app = Flask(__name__)
CORS(app)
detected_objects = []

@app.route('/')
def index():
    """
    Simply returns the Index.html inside templates to the client
    :return: Html file
    """
    return render_template('Index.html')


@app.route('/detections')
def get_detections():
    """
    creates a json from the detected_objects list and returns is as a JSON to the client
    :return: JSON containing the 10 most recent detected objects
    """

    response = app.response_class(
        response=json.dumps([ob.__dict__ for ob in detected_objects]),
        status=200,
        mimetype='application/json'
    )
    return response


if __name__ == '__main__':
    app.run()

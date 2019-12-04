from flask import Flask
import threading
import rospy
from vision.msg import ObjectPosition
import json

detected_objects = []

class detectedobject:

    def __init__(self, name, x, y, z, distance):
        self.name = name
        self.x, self.y, self.z = x, y, z
        self.distance = distance

    def __dict__(self):
        return {
            "type": self.name,
            "x": self.x,
            "y": self.y,
            "z": self.z,
            "distance": self.distance
        }

def callback(data):
    obj = detectedobject(data.type, data.x, data.y, data.z, data.distance)

    if len(detected_objects) < 10:
        detected_objects.append(obj)
    else:
        detected_objects.pop(0)
        detected_objects.append(obj)



threading.Thread(target=lambda: rospy.init_node("Object listener", disable_signals=True, anonymous=True)).start()
sub = rospy.Subscriber("Object listener", ObjectPosition,callback=callback)
app = Flask(__name__)



@app.route('/')
def hello_world():
    return 'Hello World!'


@app.route('/detections')
def get_detections():
    data = json.dumps(detected_objects)

    response = app.response_class(
        response=json.dumps(data),
        status=200,
        mimetype='application/json'
    )
    return response


if __name__ == '__main__':
    app.run()

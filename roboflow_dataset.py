from roboflow import Roboflow
rf = Roboflow(api_key="wTEWsIeWscN00L6dODfk")
project = rf.workspace("dronerangers").project("crack-detection-kjeab")
version = project.version(10)
dataset = version.download("yolov8-obb")
#!/usr/bin/env python3
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError # Import CvBridgeError
from sensor_msgs.msg import Image
import torch
from ultralytics import YOLO # Import YOLOv8 library correctly
import subprocess
# Initialize model once
model = YOLO('/home/lab/workspace_husarion/src/followline_see/src/best1.pt')
bad_fruits = ['Rotten Banana']

#def play_sound_locally():
#    subprocess.run(['plink', 'user@IP', '-pw', 'afplay', 'path/sound-bad.wav'])

def callback(data):
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))
        return

    results = model.predict(cv_image, conf=0.5 , iou=0.75)
    # Process detections
    if results:
        for result in results:
            boxes = result.boxes.cpu().numpy()  
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy [0].astype(int)
                conf = box.conf [0]
                cls = int(box.cls [0])
                class_name = model.names [cls]
                if class_name in bad_fruits:
                    print('bad')
                    #play_sound_locally()
                cv2.rectangle(cv_image, (x1, y1), (x2, y2), (255, 0, 0), 2)
                label = f'{class_name} {conf:.2f}'
                cv2.putText(cv_image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        #cv2.imshow("YOLOv8 Detection", cv_image)

        cv2.waitKey(3)

if __name__ == '__main__':
    rospy.init_node('yolov8_object_detector', anonymous=True)
    rospy.Subscriber("camera/color/image_raw", Image, callback)
    rospy.spin()

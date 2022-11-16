#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
import sys
import os
import cv2
import time

from pathlib import Path
import depthai as dai
import numpy as np
import json
import blobconverter

#import itertools

SCORE_CRITERIA = 0.25

RUN_VELOCITY = 0.3
TURN_VELOCITY = 0.3

speed = RUN_VELOCITY
turn = TURN_VELOCITY

width = 416
height = 416
half_width = width / 2

isVelPub = False

class DetectionNode(Node):
    def __init__(self):
        super().__init__('detection_node')

        # ros2 publishers
        self.img_pub = self.create_publisher(Image, '/image_raw', 10)
        self.cam_info_pub = self.create_publisher(CameraInfo, '/camera_info', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/det_cmd_vel', 10)

        # parse config
        #configPath = Path('/home/ubuntu/ros2_ws/src/oak_d_lite/oak_d_lite/json/yolov4-tiny.json')
        configPath = Path('/home/ubuntu/ros2_ws/src/oak_d_lite/oak_d_lite/json/yolov7-tiny.json')

        #print(__file__)
        print(configPath)
        if not configPath.exists():
            raise ValueError("Path {} does not exist!".format(configPath))

        with configPath.open() as f:
            config = json.load(f)
        nnConfig = config.get("nn_config", {})

        # parse input shape
        if "input_size" in nnConfig:
            W, H = tuple(map(int, nnConfig.get("input_size").split('x')))

        # extract metadata
        metadata = nnConfig.get("NN_specific_metadata", {})
        classes = metadata.get("classes", {})
        coordinates = metadata.get("coordinates", {})
        anchors = metadata.get("anchors", {})
        anchorMasks = metadata.get("anchor_masks", {})
        iouThreshold = metadata.get("iou_threshold", {})
        confidenceThreshold = metadata.get("confidence_threshold", {})

        print(metadata)

        # parse labels
        self.nnMappings = config.get("mappings", {})
        self.labels = self.nnMappings.get("labels", {})
        print(self.labels)

        # get model path
        #nnPath = 'yolov4_tiny_coco_416x416'
        nnPath = 'yolov7tiny_coco_416x416'
        if not Path(nnPath).exists():
            print("No blob found at {}. Looking into DepthAI model zoo.".format(nnPath))
            nnPath = str(blobconverter.from_zoo(nnPath, shaves = 6, zoo_type = "depthai", use_cache=True))
        
        #nnPath = '/home/ubuntu/ros2_ws/src/oak_d_lite/oak_d_lite/yolov7_tiny_openvino_2021.4_6shave.blob'

        # Create pipeline
        self.pipeline = dai.Pipeline()

        # Define sources and outputs
        self.camRgb = self.pipeline.create(dai.node.ColorCamera)
        self.detectionNetwork = self.pipeline.create(dai.node.YoloDetectionNetwork)

        self.xoutRgb = self.pipeline.create(dai.node.XLinkOut)
        self.nnOut = self.pipeline.create(dai.node.XLinkOut)

        self.xoutRgb.setStreamName("rgb")
        self.nnOut.setStreamName("nn")
        
        # Properties
        self.camRgb.setPreviewSize(416, 416)
        self.camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        self.camRgb.setInterleaved(False)
        self.camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR) # The NN model expects BGR input.
        self.camRgb.setFps(10)

        # Network specific settings
        self.detectionNetwork.setConfidenceThreshold(confidenceThreshold)
        self.detectionNetwork.setNumClasses(classes)
        self.detectionNetwork.setCoordinateSize(coordinates)
        self.detectionNetwork.setAnchors(anchors)
        self.detectionNetwork.setAnchorMasks(anchorMasks)
        self.detectionNetwork.setIouThreshold(iouThreshold)
        self.detectionNetwork.setBlobPath(nnPath)
        self.detectionNetwork.setNumInferenceThreads(2)
        self.detectionNetwork.input.setBlocking(False)
        
        # Linking
        self.camRgb.preview.link(self.detectionNetwork.input)
        self.detectionNetwork.passthrough.link(self.xoutRgb.input)
        self.detectionNetwork.out.link(self.nnOut.input)

        # bridge
        self.bridge = CvBridge()
        self.cam_info = CameraInfo()
        self.get_logger().info("initiate Oak-D-Lite ros2 node ...")
        self.frame_grabber()

    # Connect to device and start pipeline
    def frame_grabber(self):
        with dai.Device(self.pipeline) as device:
            # Output queues will be used to get the video frames and nn data from the outputs defined above
            qRgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
            qDet = device.getOutputQueue(name="nn", maxSize=4, blocking=False)

            frame = None
            detections = []
            startTime = time.monotonic()
            counter = 0
            color2 = (255, 255, 255)

            def cmdVelPublisher(obj, bbox):
                moving_forward = 0.0
                turning_left = 0.0
                score = 0.0

                no_action = True

                id = self.labels[obj.label]
                score = obj.confidence
                #xCenter = obj.xmin + (obj.xmax - obj.xmin) / 2
                xCenter = bbox[0] + (bbox[2] - bbox[0]) / 2

                if score >= SCORE_CRITERIA:
                    moving_forward = speed * 1.0
                    turning_left = turn * (half_width - xCenter) / half_width
                    no_action = False

                if no_action:
                    #moving_forward = 0.0
                    #turning_left = 0.0
                    return
                
                print(f'obj={id}, score={score}, moving={moving_forward}, turning={turning_left}')

                velocity = Twist()

                velocity.linear.x = moving_forward
                velocity.linear.y = 0.0
                velocity.linear.z = 0.0

                velocity.angular.x = 0.0
                velocity.angular.y = 0.0
                velocity.angular.z = turning_left

                self.cmd_vel_pub.publish(velocity)
                global isVelPub
                isVelPub = True

            # nn data, being the bounding box locations, are in <0..1> range - ame they need to be normalized with frwidth/height
            def frameNorm(frame, bbox):
                normVals = np.full(len(bbox), frame.shape[0])
                normVals[::2] = frame.shape[1]
                return (np.clip(np.array(bbox), 0, 1) * normVals).astype(int)

            def displayFrame(name, frame, detections):
                color = (255, 0, 0)
                for detection in detections:
                    #print(detection)
                    #print(detection.confidence)

                    label = self.labels[detection.label]
                    bbox = frameNorm(frame, (detection.xmin, detection.ymin, detection.xmax, detection.ymax))
                    cv2.putText(frame, label, (bbox[0] + 10, bbox[1] + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                    cv2.putText(frame, f"{int(detection.confidence * 100)}%", (bbox[0] + 10, bbox[1] + 40), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                    cv2.rectangle(frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), color, 2)
                    
                    #if label == 'dog' or label == 'teddy bear':
                    #    cmdVelPublisher(detection, bbox)
                        
                # Show the frame
                #cv2.imshow(name, frame)

            cnt = 0
            while True:
                inRgb = qRgb.get()
                inDet = qDet.get()
                now = self.get_clock().now().to_msg()
                global isVelPub
                if (cnt % 20) == 0:
                    isVelPub = False

                if inRgb is not None:
                    frame = inRgb.getCvFrame()
                    cv2.putText(frame, "NN fps: {:.2f}".format(counter / (time.monotonic() - startTime)),
                        (2, frame.shape[0] - 4), cv2.FONT_HERSHEY_TRIPLEX, 0.4, color2)

                if inDet is not None:
                    detections = inDet.detections
                    counter += 1
                    #print(counter)
                    #print(detections)
                    
                    #for detection in detections:
                    #    print(self.labels[detection.label], detection.confidence * 100, detection.xmax - detection.xmin, detection.ymax - detection.ymin)

                if frame is not None:
                    displayFrame("rgb", frame, detections)
                    frame_ros = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
                    self.cam_info.header.stamp = now
                    frame_ros.header.stamp = now
                    self.img_pub.publish(frame_ros)
                    self.cam_info_pub.publish(self.cam_info)

                if isVelPub is False:
                    velocity = Twist()

                    velocity.linear.x = 0.0
                    velocity.linear.y = 0.0
                    velocity.linear.z = 0.0

                    velocity.angular.x = 0.0
                    velocity.angular.y = 0.0
                    velocity.angular.z = 0.0

                    self.cmd_vel_pub.publish(velocity)
                
                cnt += 1
                #if cv2.waitKey(1) == ord('q'):
                #    break                
            
def main(args=None):
    rclpy.init(args=args)
    detection_node = DetectionNode()
    rclpy.spin(detection_node)
    detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)

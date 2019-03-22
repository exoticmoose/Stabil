#!/usr/bin/env python
from objects.msg import rosObjectHolder
from objects.msg import rosObject

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

import pyrealsense2 as rs
import numpy as np
import rospy
import cv2
import os
import time

# Sets the critical distance for ROS messages
CRITICAL_DIST = 1.5

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(rs.stream.depth, 424, 240, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 424, 240, rs.format.bgr8, 30)

# Start streaming
profile = pipeline.start(config)

# Hole Filling Filter, Fill using nearest pixel
holeFilter = rs.hole_filling_filter()
holeFilter.set_option(rs.option.holes_fill, 2)

depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()

clippingDistance = 3 / depth_scale;

# Alignment of Color and Depth
align_to = rs.stream.color
align = rs.align(align_to)

# initialize the list of class labels MobileNet SSD was trained to detect, then generate a set of bounding box colors for each class
CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat",
           "bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
           "dog", "horse", "motorbike", "person", "pottedplant", "sheep",
           "sofa", "train", "tvmonitor"]

IGNORE = ["background", "aeroplane", "bicycle", "bird", "boat",
              "bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
              "dog", "horse", "motorbike", "pottedplant", "sheep",
              "sofa", "train", "tvmonitor"]

COLORS = np.random.uniform(0, 255, size=(len(CLASSES), 3))

newdir = os.getcwd() + "/src/objects/scripts"
os.chdir(newdir);

net = cv2.dnn.readNetFromCaffe("MobileNetSSD_deploy.prototxt.txt", "MobileNetSSD_deploy.caffemodel")
net.setPreferableTarget(cv2.dnn.DNN_TARGET_OPENCL);
os.chdir("/home/nvidia/catkin_ws");

tracker = cv2.TrackerKCF_create();
# Temp Globals

class objectHolder:
    # For easy init, should just accept NN bounding box format
    def __init__(self, frame, boundingBox, objClassifier, distances, objectIdentifier):
        # ML Data Values
        self.boundingBox = boundingBox;
        self.classifier = objClassifier  # Object Classifier
        self.dist = distances  # Distance of detected object
        self.frameCounter = 0  # Frame Counter

        self.objectID = objectIdentifier
        (self.frameHeight, self.frameWidth) = frame.shape[:2]

        # (startX, startY, endX, endY)
        self.startX = int(boundingBox[0])
        self.startY = int(boundingBox[1])
        self.endX = int(boundingBox[2])
        self.endY = int(boundingBox[3])

        # Tracking bounding box init
        self.trackingBox = (self.startX, self.startY, self.endX - self.startX, self.endY - self.startY)

        # Tracking API stuff
        self.frameLimit = 5;
        self.tracker = cv2.TrackerKCF_create()  # Initiates the object tracker
        self.detected = self.tracker.init(frame, self.trackingBox)

        # Debugging Purposes
        self.colorImg = 0  # Color Img/Template of detected object

    # Returns the box in regular masking format
    def getBoundImage(self):
        return (self.startX, self.startY, self.endX, self.endY)

    # Returns the box in tracking box format
    def getTrackingBox(self):
        self.trackingBox = (self.startX, self.startY, self.endX - self.startX, self.endY - self.startY);
        return self.trackingBox;

    # Updates with the new box
    def update(self, frame, depthframe):
        detection, newBox = self.tracker.update(frame)

        if not detection:
            self.frameCounter = self.frameCounter + 1
        else:
            self.frameCounter = 0
            self.trackingBox = newBox

            # Update Image Masking box
            self.startX = int(newBox[0]) # StartX
            self.startY = int(newBox[1]) # StartY
            self.endX = int(newBox[0] + newBox[2]) # EndX = StartX + X width
            self.endY = int(newBox[1] + newBox[3]) # EndY = StartY + Y height

            # Update Bounding box coordinates
            self.boundingBox = (self.startX, self.startY, self.endX, self.endY)

            cropBox = self.boundingBox
            # Setting boundaries for depth
            if cropBox[0] < 0:
                tempStartX = 0;
            else:
                tempStartX = cropBox[0]

            if cropBox[1] < 0:
                tempStartY = 0;
            else:
                tempStartY = cropBox[1]

            if cropBox[2] > self.frameWidth:
                tempEndX = self.frameWidth;
            else:
                tempEndX = cropBox[2]

            if cropBox[3] > self.frameHeight:
                tempEndY = self.frameHeight;
            else:
                tempEndY = cropBox[3]


            croppedDepth = depthframe[tempStartY:tempEndY, tempStartX:tempEndX].astype(np.uint16)
	    #croppedDepth = np.where((croppedDepth > clippingDistance) | (croppedDepth <= 0), clippingDistance, croppedDepth)

            # Converting depth matrix to meters
            trackedDepth = croppedDepth * depth_scale

            # Average of the depth matrix
	    self.dist, _, _, _ = cv2.mean(trackedDepth)

    # Returns true if found within the frame limit
    def valid(self):
        if self.frameCounter > self.frameLimit:
            return False
        else:
            return True

    def getColorCrop(self, colorFrame):
        self.colorImg = colorFrame[self.startY:self.endY, self.startX:self.endX]
        return self.colorImg;

    def getDepth(self):
        return self.dist

objHolderArray = [];
frameCounter = 0;
objIDcounter = 1;

pub = rospy.Publisher('rosObjectHolder', rosObjectHolder, queue_size=2)
image_pub = rospy.Publisher("cvStream", Image)
bridge = CvBridge()

rospy.init_node('objects', anonymous=True)

try:
    while True:

        # Wait for a coherent pair of frames: depth and color
        #print("Waiting for frames...")
        #print(time.time())
        frames = pipeline.wait_for_frames()
        #print("Got frames")
        #print(time.time())

        # Aligns the depth frames
        frames = align.process(frames)

        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # Hole Filtering
        depth_frame = holeFilter.process(depth_frame);

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        (h, w) = color_image.shape[:2]

        cleanColor = color_image;
        detectionMasked = color_image;

        #print(time.time())
		
        # Check the Object list and mask with any tracked objects...
        if len(objHolderArray) != 0:
            for i in range(len(objHolderArray)):
                # Updates all the new locations of tracked objects..
                objHolderArray[i].update(color_image, depth_image)
                (startX, startY, endX, endY) = objHolderArray[i].getBoundImage()

                # Creating the "white" mask
                mask = np.ones((h, w))

                # Draw a 0 mask for each tracked object in the mask
                cv2.rectangle(mask, (startX, startY), (endX, endY), 0, cv2.FILLED)
                mask = mask.astype(np.int8)

                # Applies the mask for the colored object detection input
                detectionMasked = cv2.bitwise_and(detectionMasked, detectionMasked, mask=mask)

        # Repeat every 6 frames...
        #print(time.time())
        blob = cv2.dnn.blobFromImage(cv2.resize(detectionMasked, (400, 240)), 0.007843, (400, 240), 127.5)
        net.setInput(blob)

        #print(time.time())
		
        if (frameCounter % 10.0) == 1:
            detections = net.forward()
            #print("Start net")
            #print(time.time())
            for i in np.arange(0, detections.shape[2]):
                # extract the confidence (i.e., probability) associated with the prediction
                confidence = detections[0, 0, i, 2]

                # filter out weak detections by ensuring the `confidence` is greater than the minimum confidence
                if confidence > 0.8:
                    # extract the index of the class label from the `detections`, then compute the (x, y)-coordinates of the bounding box for the object
                    idx = int(detections[0, 0, i, 1])
                    box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                    (startX, startY, endX, endY) = box.astype("int")

                    # Skips ignored objects
                    if CLASSES[idx] in IGNORE:
                        continue

                    # Depth cropping
                    depth = depth_image[startY:endY, startX:endX].astype(np.uint16)

                    # Converting depth matrix to meters
                    depth = depth * depth_scale
	    	    #depth = np.where((depth > clippingDistance) | (depth <= 0), clippingDistance, depth)

                    # Average of the depth matrix
		    dist, _, _, _ = cv2.mean(depth)

                    # Label description/generation
                    label = CLASSES[idx] + " %f2" % dist + " Meters"
                    #print("[INFO] {}".format(label))

                    boundingBox = (startX, startY, endX, endY)  # From NN output

                    # (Obj)- frame, NN Bounding Box, Classifier, Distance
                    tempObj = objectHolder(color_image, boundingBox, CLASSES[idx], dist, objIDcounter)

                    objIDcounter = objIDcounter + 1;

                    # Save it to the list
                    objHolderArray.append(tempObj)
					
        #print(time.time())

        if len(objHolderArray) != 0:
            # Remove any invalid tracked objects (Lost after 5 frames)
            objHolderArray[:] = [tup for tup in objHolderArray if tup.valid()]

            # Sorts the items based off distances
            objHolderArray.sort(key=lambda x: x.dist)

            for i in range(len(objHolderArray)):
                (startX, startY, endX, endY) = objHolderArray[i].getBoundImage()
                label = objHolderArray[i].classifier + " %f2" % objHolderArray[i].dist + " Meters"

                cv2.rectangle(color_image, (startX, startY), (endX, endY), COLORS[i], 2)
                y = startY - 15 if startY - 15 > 15 else startY + 15
                cv2.putText(color_image, label, (startX, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLORS[i], 2)

        frameCounter = frameCounter + 1;

        # Show the output image
        # cv2.imshow("Output", color_image)
        #key = cv2.waitKey(1)

        image_pub.publish(bridge.cv2_to_imgmsg(color_image, "bgr8"))
        #print(time.time())
        #if key == ord("c"):
        #    break;

        ## ROS MSGS
        if len(objHolderArray) > 0:
            msgArray = rosObjectHolder()
            objHolderArray.sort(key=lambda x: x.dist)

            for i in range(len(objHolderArray)):
                rosMessage = rosObject()

                rosMessage.classifier = objHolderArray[i].classifier;
                rosMessage.objectID = objHolderArray[i].objectID;

                (startX, startY, width, height) = objHolderArray[i].getTrackingBox()
                startX = startX - w/2
                rosMessage.centerX = startX + width/2
                rosMessage.width = width
                rosMessage.distance = objHolderArray[i].dist;

                if objHolderArray[i].dist < CRITICAL_DIST:
                    rosMessage.critical = True
                else:
                    rosMessage.critical = False;

                msgArray.data.append(rosMessage)

            pub.publish(msgArray)

			
        #print("Processing done")
        #print(time.time())
        #print("---------")

finally:

    # Stop streaming
    pipeline.stop()

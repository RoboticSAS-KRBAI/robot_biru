#!/usr/bin/env python3
import rospy
from robotic_sas_auv_ros.msg import ObjectDetection, ObjectDifference

class Subscriber():
    def __init__(self):
        self.object_detection = ObjectDetection()
        self.object_difference = ObjectDifference()

        self.pub_object_difference = rospy.Publisher('object_difference', ObjectDifference, queue_size=10)
        # Subscriber
        rospy.Subscriber('object_detection', ObjectDetection, self.object_detection_callback)

    def object_detection_callback(self, data):
        rospy.loginfo("Received object detection message with %d bounding boxes", len(data.bounding_boxes))
    
        # Center x-coordinate of the frame
        frame_center_x = 640 // 2

        self.object_difference.object_type = "None"
        self.object_difference.x_difference = 0
        self.largest = 0
        self.x_gate = 0
        self.x_obstacle = 0
        self.x_bucket = 0
        
        for bbox in data.bounding_boxes:
            # rospy.loginfo("Class: %s, Probability: %.2f, Coordinates: (%d, %d), (%d, %d)",
            #             bbox.class_name, bbox.probability, bbox.x_min, bbox.y_min, bbox.x_max, bbox.y_max)
            
            # Calculate the center x-coordinate of the bounding box
            center_x = (bbox.x_min + bbox.x_max) // 2
            
            # Calculate the pixel difference between the center of the bounding box and the center of the frame
            x_difference = center_x - frame_center_x

            if bbox.class_name == "Orange_Flare":
                self.object_difference.object_type = bbox.class_name
                self.object_difference.x_difference = x_difference
                self.x_obstacle = bbox.x_max - bbox.x_min
            if bbox.class_name == "Gate":
                self.object_difference.object_type = bbox.class_name
                self.object_difference.x_difference = x_difference
                self.x_gate = bbox.x_max - bbox.x_min
            if bbox.class_name == "Bucket":
                self.object_difference.object_type = bbox.class_name
                self.object_difference.x_difference = x_difference
                self.x_bucket = bbox.x_max - bbox.x_min

            self.largest = max(self.x_obstacle, self.x_gate, self.x_bucket)
                
            # rospy.loginfo("Center x-coordinate of bounding box: %d", center_x)
            # rospy.loginfo("Pixel difference between bounding box center and frame center: %d", x_difference)
        
        self.pub_object_difference.publish(self.object_difference)

    def spin(self):
        rospy.spin()

def main():
    rospy.init_node('node_accumulator', anonymous=True)

    subscriber = Subscriber()

    subscriber.spin()

if __name__ == '__main__':
    main()
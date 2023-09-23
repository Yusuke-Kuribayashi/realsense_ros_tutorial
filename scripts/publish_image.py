# ROS
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
# python library
import pyrealsense2 as rs
import numpy as np

if __name__ == '__main__':
    rospy.init_node('publish_image_python')
    pub_color_image = rospy.Publisher('color_image_raw', Image, queue_size=10)
    pub_depth_image = rospy.Publisher('depth_image_raw', Image, queue_size=10)
    pub_ir_image    = rospy.Publisher('ir_image_raw', Image, queue_size=10)
    
    # set bridge
    bridge = CvBridge()
    
    # set pipeline
    pipeline = rs.pipeline()
    
    # set config
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, 30)
    
    pipeline.start(config)
    print("start!!")
    
    try:
        while not rospy.is_shutdown():
            # wait until a new image can be received
            frames = pipeline.wait_for_frames()
            
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            ir_frame    = frames.get_infrared_frame(1)
            
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
            ir_image    = np.asanyarray(ir_frame.get_data())
            
            color_msg = bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
            depth_msg = bridge.cv2_to_imgmsg(depth_image, encoding="mono16")
            ir_msg    = bridge.cv2_to_imgmsg(ir_image, encoding="mono8")
            
            pub_color_image.publish(color_msg)
            pub_depth_image.publish(depth_msg)
            pub_ir_image.publish(ir_msg)
            
    finally:
        pipeline.stop()
    
    
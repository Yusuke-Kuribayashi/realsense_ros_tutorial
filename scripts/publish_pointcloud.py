# ROS
import rospy
import sensor_msgs.point_cloud2 as pcd2
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
# python library
import pyrealsense2 as rs
import numpy as np

def create_cloud_xyzrgb(header, points):
    fields = [PointField("x", 0, PointField.FLOAT32, 1),
                PointField("y", 4, PointField.FLOAT32, 1),
                PointField("z", 8, PointField.FLOAT32, 1),
                PointField("rgb", 12, PointField.FLOAT32, 1)]
    return pcd2.create_cloud(header, fields, points)

if __name__ == '__main__':
    rospy.init_node('publish_pointcloud_python')
    pub_pc = rospy.Publisher('pc_data', PointCloud2, queue_size=10)
    
    # set pipeline
    pipeline = rs.pipeline()
    
    # set config
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    
    # match the angle of view of color and depth images
    align = rs.align(rs.stream.color)
    
    # header
    header = Header()
    # header.frame_id = "camera_color_optical_frame"
    header.frame_id = "map"
    
    # pointcloud
    points_data = rs.points()
    pc_data = rs.pointcloud()
    
    # start streaming
    pipeline.start(config)
    print("start!!")
    
    try:
        while not rospy.is_shutdown():
            # wait until a new image can be received
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()
            
            if not color_frame or not color_frame:
                continue
            
            # create pointcloud
            # points coordinate
            points_data = pc_data.calculate(depth_frame)
            np_vtx = np.asanyarray(points_data.get_vertices())
            np_vtx = np_vtx.reshape(-1, 1)
            np_vtx.dtype = np.float32
            
            # points color
            color_image = np.asanyarray(color_frame.get_data())
            color_image = color_image.reshape(-1, 3)
            r,g,b = color_image[:,2].astype(np.uint32), color_image[:,1].astype(np.uint32), color_image[:,0].astype(np.uint32)
            rgb = np.array((r << 16) | (g << 8) | (b << 0), dtype=np.uint32)
            np_tex = rgb.reshape(-1, 1)
            np_tex.dtype = np.float32
            
            header.stamp = rospy.Time.now()
            msg = create_cloud_xyzrgb(header=header, points=np.hstack((np_vtx, np_tex)).astype(np.float32))
            
            pub_pc.publish(msg)
            
    finally:
        pipeline.stop()
        
    print("program has finished")
    
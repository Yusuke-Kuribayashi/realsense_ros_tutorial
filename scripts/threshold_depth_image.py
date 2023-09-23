import pyrealsense2 as rs
import numpy as np
import cv2

if __name__ == '__main__':
    # set pipeline
    pipeline = rs.pipeline()
    
    # set config
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    
    # set threshold
    threshold_filter = rs.threshold_filter()
    threshold_filter.set_option(rs.option.max_distance, 4.0) # upper threshold
    threshold_filter.set_option(rs.option.min_distance, 1.0) # lower threshold
    
    pipeline.start(config)
    print("start!!")
    
    try:
        while(True):
            # wait until a new image can be received
            frames = pipeline.wait_for_frames()
            threshold_frames = threshold_filter.process(frames.get_depth_frame())
            
            depth_frame = threshold_frames.as_frame()
            depth_color_image = np.asanyarray(rs.colorizer().colorize(depth_frame).get_data())
            
            cv2.imshow("threshold depth image", depth_color_image)
            key = cv2.waitKey(1)
            if key == 27:
                break
    finally:
        cv2.destroyAllWindows()
        pipeline.stop()
        
    print("program has finished")
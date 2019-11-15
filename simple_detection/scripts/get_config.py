import pyrealsense2 as rs
import numpy as np

if __name__=="__main__":
    align = rs.align(rs.stream.color)

    width = 1280
    height = 720

    config = rs.config()
    config.enable_stream(rs.stream.depth, width, height, rs.format.z16, 6)
    config.enable_stream(rs.stream.color, width, height, rs.format.rgb8, 6)
    pipeline = rs.pipeline()
    profile = pipeline.start(config)
    depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
    print('depth scale :',depth_scale)

    # # get camera intrinsics
    intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

    print(intr)
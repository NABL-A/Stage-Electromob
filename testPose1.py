# packages
import pyrealsense2 as rs
import numpy as np
import cv2 as cv
import numpy as np

"""
Utilisation de la camera intel realsense
"""
def initialize_device():
    # Create a pipeline
    pipeline = rs.pipeline()
    config = rs.config()

    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()


    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    profile = pipeline.start(config)

    # Get stream profile and camera intrinsics
    color_profile = rs.video_stream_profile(profile.get_stream(rs.stream.color))
    color_intrinsics = color_profile.get_intrinsics()
    color_extrinsics = color_profile.get_extrinsics_to(color_profile)
   
    # Getting the depth sensor's depth scale (see rs-align example for explanation)
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()

    # Create an align object
    # rs.align allows us to perform alignment of depth frames to others frames
    # The "align_to" is the stream type to which we plan to align depth frames.
    align_to = rs.stream.color
    align = rs.align(align_to)

    return pipeline, align, depth_scale,color_intrinsics,color_extrinsics


"""
Estimation de position X, Y, Z par la camera realsense dans le repere camera
"""
def positionXYZ(x, y):
    # x et y en pixel
    pipeline, align, depth_scale, color_intrinsics,color_extrinsics = initialize_device()

    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)

    # Get aligned frames
    aligned_depth_frame = aligned_frames.get_depth_frame()  # aligned_depth_frame is a 640x480 depth image
    color_frame = aligned_frames.get_color_frame()
    depth = aligned_depth_frame.get_distance(x, y)
    # X, Y, Z = rs.rs2_deproject_pixel_to_point(intr, pixel, depth)
    point = rs.rs2_deproject_pixel_to_point(color_intrinsics, [x, y], depth)
    # en mètres m
    point=[point[0]*1000, point[1]*1000, point[2]*1000]
    return point

#-------------------OPEN CAMERA-------------------
cap1 = cv.VideoCapture(4)
cap2 = cv.VideoCapture(6)
if not cap1.isOpened() or not cap2.isOpened():
    print("Cannot open camera 4")
    exit()

#-------------------PRIS UN POINT-------------------

while True:
    # Capture frame-by-frame
    ret1, frame1 = cap1.read()
    ret2, frame2 = cap2.read()

    if not ret1 or not ret2:
        print("Can't receive frame (stream end?). Exiting ...")
        break

    # Display the resulting frame
    cv.imshow('frame1', frame1)
    cv.imshow('frame2', frame2)
'''
im = cv.imread("plank.jpg")
gray = cv.cvtColor(im,cv.COLOR_BGR2GRAY);
gray = cv.GaussianBlur(gray, (5, 5), 0)
_, bin = cv.threshold(gray,120,255,1) # inverted threshold (light obj on dark bg)
bin = cv.dilate(bin, None)  # fill some holes
bin = cv.dilate(bin, None)
bin = cv.erode(bin, None)   # dilate made our shape larger, revert that
bin = cv.erode(bin, None)
bin, contours, hierarchy = cv.findContours(bin, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)

rc = cv.minAreaRect(contours[0])
box = cv.boxPoints(rc)
for p in box:
    pt = (p[0],p[1])
    print pt
    cv.circle(im,pt,5,(200,0,0),2)
cv.imshow("plank", im)
'''
cv.waitKey()


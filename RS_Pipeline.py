#!/usr/bin/env python2.7
import numpy as np
import cv2
import time
import pyrealsense2 as rs
import time
import timeit
import matplotlib.pyplot as plt

class TAKEIMAGECLASS:

	def __init__(self):

		self.pipe = rs.pipeline()#-5.602836608886719e-05s

		CAM_WIDTH, CAM_HEIGHT, CAM_FPS = 848,480,30

		cfg = rs.config()#-3.790855407714844e-05s
		cfg.enable_device('848312070974')
		cfg.enable_stream(rs.stream.depth, CAM_WIDTH, CAM_HEIGHT, rs.format.z16, CAM_FPS)
		cfg.enable_stream(rs.stream.color, CAM_WIDTH, CAM_HEIGHT, rs.format.bgr8, CAM_FPS)

		profile= self.pipe.start(cfg)#0.19227695s

		sensor_dep = profile.get_device().first_depth_sensor() #Depth is the first sensor
		sensor_dep.set_option(rs.option.exposure, 4000)##0.00676s
		sensor_dep.set_option(rs.option.gain, 19)##0.0035s

		sensor_rgb = profile.get_device().query_sensors()[1] #RGB is the second sensor
		sensor_rgb.set_option(rs.option.exposure, 500)
		sensor_rgb.set_option(rs.option.gain, 19)

	def capture_rgb(self):

		try:
			frameset = self.pipe.wait_for_frames()
			RGB_frame = frameset.get_color_frame()
			RGB_frame=np.asanyarray(RGB_frame.get_data())
			# pipe.stop()
		except Exception as e:
			#print (e)
			return False,False
		return RGB_frame,True

	def capture_depth(self):
		try:
			frameset = self.pipe.wait_for_frames()################################## 0.13s
			depth_frame = frameset.get_depth_frame()
			hole_filling = rs.hole_filling_filter()  # hole filling - hole filling
			depth_frame = hole_filling.process(depth_frame)
			# Cleanup:
			# self.pipe.stop()

			depth_frame=np.asanyarray(depth_frame.get_data())
		except Exception as e:
			#print (e)
			return False,False
		return depth_frame,True


if __name__ == "__main__":
	x=TAKEIMAGECLASS()


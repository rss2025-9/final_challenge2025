import cv2
import numpy as np

#################### X-Y CONVENTIONS #########################
# 0,0  X  > > > > >
#
#  Y
#
#  v  This is the image. Y increases downwards, X increases rightwards
#  v  
#  v
#  v
#  v
###############################################################


def image_print(img):
	"""
	Helper function to print out images, for debugging. Pass them in as a list.
	Press any key to continue.
	"""
	cv2.imshow("image", img)
	cv2.waitKey(0)
	cv2.destroyAllWindows()

def cd_color_segmentation(img, template):
	"""
	Implement the object detection using color segmentation algorithm
	Input:
		img: np.3darray; the input image with an object to be detected. BGR.
		template_file_path; Not required, but can optionally be used to automate setting hue filter values.
	Return:
		bbox: ((x1, y1), (x2, y2)); the bounding box of the cone, unit in px
				(x1, y1) is the top left of the bbox and (x2, y2) is the bottom right of the bbox
	"""

	# crop the image to focus on the lower half
	height = img.shape[0]
	crop_y_start = height // 2  # crop down the image (tune this to crop more)
	cropped_img = img[crop_y_start:, :, :]  # y, x, channel

	# convert the image from RGB to HSV
	hsv_object = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2HSV)

	# define lower and upper bound for orange color
	lower_bound = np.array([0, 0, 200])	# hue, saturation (intensity), value (brightness)
	upper_bound = np.array([179, 255, 255])	# value=0 -> black, saturation=0 -> white if value is high enough

	# create mask
	mask = cv2.inRange(hsv_object, lower_bound, upper_bound)

	# Matrix of size 3 as a kernel
	kernel = np.ones((3, 3), np.uint8)

	# Erosion and dilation
	eroded_mask = cv2.erode(mask, kernel, iterations=1)
	dilated_mask = cv2.dilate(eroded_mask, kernel, iterations=2)
	
    # Edge detection
	# takes img, lower threshold, upper threshold, order of the Sobel filter (larger value detect more detailed features)
	edges = cv2.Canny(dilated_mask, 50, 150, apertureSize=3)
	
    # Line detection
	# rho is in pixels, theta is in radians
	lines = cv2.HoughLinesP(edges, rho=1, theta=np.pi/180, threshold=50, minLineLength=20, maxLineGap=10)
	
	# filter out the unwanted color
	# result = cv2.bitwise_and(img, img, mask=dilated_mask)
	
    # draw the line
	# if lines is not None: 
	# 	for line in lines:
	# 		x1, y1, x2, y2 = line[0]
	# 		y1 += crop_y_start
	# 		y2 += crop_y_start
	# 		cv2.line(img, (x1, y1), (x2, y2), (255, 0, 0), 3)

	# cv2.imshow("Segmented Output", result)
	# cv2.imshow("image", img)
	# cv2.waitKey(0)
	# cv2.destroyAllWindows()

	# Return lines and y-coordinate of the cropped image
	return lines, crop_y_start
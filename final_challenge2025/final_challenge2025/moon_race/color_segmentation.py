import cv2
import numpy as np

#################### X-Y CONVENTIONS #########################
# 0,0  X  > > >
#  Y
#  v  This is the image. Y increases downwards, X increases rightwards
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

def cd_color_segmentation(img, template: str = None, node = None):
	"""
	Implement the object detection using color segmentation algorithm
	Input:
		img: np.3darray; the input image with an object to be detected. BGR.
		template_file_path; Not required, but can optionally be used to automate setting hue filter values.
	Return:
		Return lines and y-coordinate of the cropped image
	"""

	# crop the image to focus on the lower half
	height = img.shape[0]
	crop_y_start = 2 * height // 5  # crop down the image (tune this to crop more)
	cropped_img = img[crop_y_start:, :, :]  # y, x, channel

	# convert the image from RGB to HSV
	hsv_object = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2HSV)

	# define lower and upper bound for white color
	lower_bound = np.array([0, 0, 140])	# hue, saturation (intensity), value (brightness)
	upper_bound = np.array([179, 30, 255])	# value=0 -> black, saturation=0 -> white if value is high enough

	# create mask
	mask = cv2.inRange(hsv_object, lower_bound, upper_bound)

	# Matrix of size 3 as a kernel
	kernel = np.ones((3, 3), np.uint8)

	# Erosion and dilation
	eroded_mask = cv2.erode(mask, kernel, iterations=1)
	dilated_mask = cv2.dilate(eroded_mask, kernel, iterations=2)
	if node is not None:
		debug_msg = node.bridge.cv2_to_imgmsg(dilated_mask, "mono8")
		node.debug_pub.publish(debug_msg)

	# filter out the unwanted color
	result = cv2.bitwise_and(cropped_img, cropped_img, mask=dilated_mask)
	
    # Edge detection
	# takes img, lower threshold, upper threshold, order of the Sobel filter (larger value detect more detailed features)
	edges = cv2.Canny(dilated_mask, 50, 150, apertureSize=3)
	
    # Line detection
	# rho is in pixels, theta is in radians
	lines = cv2.HoughLinesP(
		edges, rho=1, theta=np.pi/360, threshold=3, minLineLength=70, maxLineGap=15
	)
	
	filtered_lines = []
	lower_threshold_angle = 17	# in degrees (tune this value if needed)
	upper_threshold_angle = 85  # in degrees (tune this value if needed)
	least_violator = None
	least_violator_delta = float('inf')
	for line in lines:
		x1, y1, x2, y2 = line[0]
		dx = x2 - x1
		dy = y2 - y1
		if dx == 0:		# vertical line
			angle = 90
		else:
			angle = abs(np.degrees(np.arctan2(dy, dx)))	# angle in degrees
		
		if lower_threshold_angle < angle < upper_threshold_angle:
			filtered_lines.append([[x1, y1, x2, y2]])
		elif(delta := min(abs(lower_threshold_angle - angle), abs(upper_threshold_angle - angle))) < least_violator_delta:
			least_violator = [[x1, y1, x2, y2]]
			least_violator_delta = delta

	# Return lines and y-coordinate of the cropped image
	return filtered_lines if filtered_lines else [least_violator], crop_y_start, result
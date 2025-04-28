import cv2
import numpy as np

#################### X-Y CONVENTIONS #########################
# 0,0  X  > > > > >
#
#  Y
#
#  v  This is the image. Y increases downwards, X increases rightwards
#  v  Please return bounding boxes as ((xmin, ymin), (xmax, ymax))
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
	Implement the cone detection using color segmentation algorithm
	Input:
		img: np.3darray; the input image with a cone to be detected. BGR.
		template_file_path; Not required, but can optionally be used to automate setting hue filter values.
	Return:
		bbox: ((x1, y1), (x2, y2)); the bounding box of the cone, unit in px
				(x1, y1) is the top left of the bbox and (x2, y2) is the bottom right of the bbox
	"""
	########## YOUR CODE STARTS HERE ##########
	# convert the image from RGB to HSV
	hsv_cone = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

	# define lower and upper bound for orange color
	lower_bound = np.array([0, 0, 200])	# hue, saturation (intensity), value (brightness)
	upper_bound = np.array([179, 255, 255])	# value=0 -> black, saturation=0 -> white if value is high enough

	# create mask
	mask = cv2.inRange(hsv_cone, lower_bound, upper_bound)

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
	lines = cv2.HoughLinesP(edges, rho=1, theta=np.pi/180, threshold=50, minLineLength=50, maxLineGap=10)

	# filter out the unwanted color
	result = cv2.bitwise_and(img, img, mask=dilated_mask)

	contours, _ = cv2.findContours(dilated_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	bounding_boxes = []
	for contour in contours:
		if cv2.contourArea(contour) > 150:
			x, y, w, h = cv2.boundingRect(contour)
			# For line-following.
			if y < 150:
				continue
			cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)
			bounding_boxes.append(((x, y), (x + w, y + h)))

	if bounding_boxes is None:
		bounding_boxes = [((0, 0), (0, 0))]

	# cv2.imshow("Segmented Output", result)
	cv2.imshow("image", img)
	cv2.waitKey(0)
	cv2.destroyAllWindows()

	########### YOUR CODE ENDS HERE ###########

	# Return bounding box
	return bounding_boxes
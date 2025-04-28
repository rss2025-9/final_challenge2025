import cv2
import os
from color_segmentation import cd_color_segmentation

# Path to the folder of racetrack_images
image_folder = "../../racetrack_images/lane_1"

# Get all image filenames
image_filenames = [f for f in os.listdir(image_folder) if f.endswith(('.png', '.jpg', '.jpeg'))]

# Loop through all images
for filename in image_filenames:
    img_path = os.path.join(image_folder, filename)
    img = cv2.imread(img_path)

    if img is None:
        print(f"Failed to load {filename}")
        continue

    # Run color segmentation
    bounding_boxes = cd_color_segmentation(img, None)

    # Draw all bounding boxes
    for (xmin, ymin), (xmax, ymax) in bounding_boxes:
        if (xmin, ymin) != (0, 0) or (xmax, ymax) != (0, 0):
            cv2.rectangle(img, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)

    # Show the result
    cv2.imshow("Segmented Image", img)
    key = cv2.waitKey(0)  # Press any key to go to the next image
    if key == ord('q'):
        break

cv2.destroyAllWindows()
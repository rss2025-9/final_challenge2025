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
    lines, crop_y_start, result = cd_color_segmentation(img, None)

    # Draw the lines
    if lines is not None:
        for line in lines:
            print(line)
            x1, y1, x2, y2 = line[0]
            y1 += crop_y_start
            y2 += crop_y_start
            cv2.line(img, (x1, y1), (x2, y2), (255, 0, 0), 3)

    # Show the result
    cv2.imshow("Detected Lines", img)
    cv2.imshow("Segmented Output", result)
    key = cv2.waitKey(0)  # Press any key to go to the next image
    if key == ord('q'):
        break

cv2.destroyAllWindows()
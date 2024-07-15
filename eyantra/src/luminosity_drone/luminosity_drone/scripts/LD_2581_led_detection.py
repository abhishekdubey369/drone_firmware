import cv2
import numpy as np

def detect_leds(image_path):
    # Convert the image to the HSV color space for better color segmentation
    image = cv2.imread(image_path)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Apply GaussianBlur to reduce noise
    blurred = cv2.GaussianBlur(gray, (5,5), cv2.BORDER_DEFAULT)
    _, binary_mask = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    # Connected component analysis
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(binary_mask, connectivity=8)

    # Initialize lists to store valid centroid coordinates
    valid_centroids = []

    for i in range(1, num_labels):
        # Exclude small regions (adjust the area threshold as needed)
        if stats[i, cv2.CC_STAT_AREA] > 10:
            cx, cy = centroids[i]
            centroid = (int(cx), int(cy))

            # Exclude non-circular regions based on circularity
            contour_mask = (labels == i).astype(np.uint8)
            contours, _ = cv2.findContours(contour_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if contours:
                # Calculate the perimeter using the first contour found
                perimeter = cv2.arcLength(contours[0], closed=True)

                # Calculate circularity
                circularity = (4 * np.pi * stats[i, cv2.CC_STAT_AREA]) / (perimeter ** 2)

                # Adjust the circularity threshold as needed
                if 0.951439781 <= circularity <= 1.3:
                    # Draw the bright spot on the image
                    cv2.circle(image, centroid, 4, (0, 255, 0), -1)
                    valid_centroids.append(centroid)

    # Display the processed image (for debugging purposes)
    cv2.imshow("LED Detection", image)
    cv2.waitKey(0)
    # cv2.destroyAllWindows()

    print(f"{len(valid_centroids)}")

    return len(valid_centroids), valid_centroids

# Example usage:
image_path = 'image2.jpg'  # Replace with the path to your image
num_light_sources, centroids = detect_leds(image_path)
print(f"No. of light sources detected: {num_light_sources}")
for i, centroid in enumerate(centroids):
    print(f"Centroid #{i + 1}: {centroid}")

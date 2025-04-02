import cv2
import numpy as np

DEBUG = False

def detect_square(mask, output_path):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    output = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    
    for cnt in contours:
        epsilon = 0.02 * cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, epsilon, True)
        if len(approx) == 4 and cv2.contourArea(cnt) > 100:
            if DEBUG:
                cv2.drawContours(output, [approx], -1, (0, 0, 255), 2)
                cv2.imwrite(output_path, output)
            return True
    
    cv2.imwrite(output_path, output)
    return False

def main():
    img = cv2.imread("green.jpg")
    
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    lower_green = np.array([40, 50, 50])
    upper_green = np.array([80, 255, 255])
    mask_green = cv2.inRange(hsv, lower_green, upper_green)
    
    lower_blue = np.array([100, 150, 50])
    upper_blue = np.array([140, 255, 255])
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

    if DEBUG:
        cv2.imwrite("mask_green.jpg", mask_green)
        cv2.imwrite("mask_blue.jpg", mask_blue)

    green_square = detect_square(mask_green, "contours_green.jpg")
    blue_square = detect_square(mask_blue, "contours_blue.jpg")
    
    if green_square:
        print("green square!")
    elif blue_square:
        print("blue square!")

if __name__ == "__main__":
    main()

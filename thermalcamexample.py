import cv2
import numpy as np
import sys

enable_second_open = (True if sys.argv[1] == "y" else False) if len(sys.argv) > 1 else False

extra = sys.argv[2] if len(sys.argv) > 2 else ""
fname = f"tcam{ extra }.png"

while True:
 lower_bound = np.array([0, 0, 100])
 upper_bound = np.array([0, 0, 255])

 image = cv2.imread(fname, cv2.IMREAD_GRAYSCALE)

 bgr = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
 hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
 mask = cv2.inRange(hsv, lower_bound, upper_bound)

 kernel = np.ones((15,15),np.uint8)
 if enable_second_open:
  opened_mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
  closed_mask = cv2.morphologyEx(opened_mask, cv2.MORPH_CLOSE, kernel)
 else:
  closed_mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
 opened_mask = cv2.morphologyEx(closed_mask, cv2.MORPH_OPEN, kernel)
 final = np.zeros((240, 240, 3), np.uint8)

 _, thresh = cv2.threshold(opened_mask, 40, 255, cv2.THRESH_BINARY)
 contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

 largest_area = 0
 largest_contour = None
 for contour in contours:
  area = cv2.contourArea(contour)
  if area > largest_area:
   largest_area = area
   largest_contour = contour

 M = cv2.moments(largest_contour)
 #cv2.circle(final, (round(M['m10'] / M['m00']), round(M['m01'] / M['m00'])), 3, (255, 225, 0), -1)
 
 x_cord = round(M['m10'] / M['m00'])
 y_cord = round(M['m01'] / M['m00'])
 for point in largest_contour:
  if point[0][1] > y_cord:
   y_cord = point[0][1]
  if point[0][0] < x_cord:
   x_cord = point[0][0]
 rad = x_cord if (x_cord - round(M['m10'] / M['m00'])) > (y_cord - round(M['m01'] / M['m00'])) else y_cord
 rad //= 2
 cv2.circle(final, (round(M['m10'] / M['m00']), round(M['m01'] / M['m00'])), rad, (255, 225, 0), 3)

 orange_mask = np.zeros(final.shape, final.dtype)
 orange_mask[:,:] = (255, 128, 0)
 orange_mask = cv2.bitwise_and(orange_mask, orange_mask, mask=opened_mask)
 cv2.addWeighted(orange_mask, 1, final, 1, 0, final)

 cv2.imshow("image", image)
 cv2.imshow("mask", mask)
 cv2.imshow("closed mask", closed_mask)
 cv2.imshow("opened mask", opened_mask)
 cv2.imshow("final", final)

 cv2.moveWindow("image", 5, 0)
 cv2.moveWindow("mask", 255, 0)
 cv2.moveWindow("closed mask", 5, 275)
 cv2.moveWindow("opened mask", 255, 275)
 cv2.moveWindow("final", 5, 550)

 code = cv2.waitKey()
 if code != 32:
  break

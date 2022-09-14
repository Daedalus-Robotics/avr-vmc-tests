import cv2
import numpy
from pupil_apriltags import Detector
import sys

LINE_WIDTH = 5

RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
PURPLE = (214, 48, 255)
LIGHT_BLUE = (0, 255, 255)
ORANGE = (255, 166, 0)

COLORS = [GREEN, RED, BLUE, PURPLE, LIGHT_BLUE, ORANGE]

def get_color(tag_id):
  num = tag_id % len(COLORS)
  return COLORS[num]

def convert_tag_corners(corners):
    new_corners = []
    for corner in corners:
      new_corner = []
      for cord in corner:
        new_corner.append(int(cord.round()))
        #print(cord)
        #print(int(cord.round()))
      new_corners.append(new_corner)
      #print(new_corner)
    #print(new_corners)
    return new_corners

if len(sys.argv) > 2:
  print("Enabled Pose")
  enablePose = True
  tagSize = int(sys.argv[2]) / 100
else:
  enablePose = False
  tagSize = 0.1

at_detector = Detector(families='tag36h11', nthreads=1, quad_decimate=1.0, quad_sigma=0.0, refine_edges=1, decode_sharpening=0.25, debug=0)

cam_id = 0
while True:
  print("Opening camera " + str(cam_id))
  cam = cv2.VideoCapture(cam_id)
  if input("Is this camera correct? (y/n): ") == "y":
    break
  cam.release()
  cam_id += 1

try:
  while True:
    _, frame = cam.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    tags = at_detector.detect(gray, estimate_tag_pose=enablePose, camera_params=[1, 1, 0, 0], tag_size=tagSize)
    
    height = frame.shape[0]
    
    for tag in tags:
      color = get_color(tag.tag_id)
      centerpos = (int(tag.center[0]), int(tag.center[1]))
      centerx = centerpos[0]
      corners = convert_tag_corners(tag.corners)
      highest_corner = 0
      lowest_corner = height
      for corner in corners:
        y = corner[1]
        if y > highest_corner:
          highest_corner = y
        if y < lowest_corner:
          lowest_corner = y
      print("Rotation: ", end="")
      print(tag.pose_R)
      print("Translation: ", end="")
      print(tag.pose_t)
      
      cv2.circle(frame, centerpos, LINE_WIDTH, color, -1)
      
      cv2.line(frame, corners[0], corners[1], color, LINE_WIDTH)
      cv2.line(frame, corners[1], corners[2], color, LINE_WIDTH)
      cv2.line(frame, corners[2], corners[3], color, LINE_WIDTH)
      cv2.line(frame, corners[3], corners[0], color, LINE_WIDTH)
      
      cv2.putText(frame, str(tag.tag_id), (centerx, (lowest_corner - 15) if lowest_corner > 0 else (highest_corner + 15)), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2, cv2.LINE_AA)
    cv2.imshow("apriltag", frame)
    keycode = cv2.waitKey(1)
    if keycode == 32:
      break
finally:
  cam.release()
  cv2.destroyAllWindows()

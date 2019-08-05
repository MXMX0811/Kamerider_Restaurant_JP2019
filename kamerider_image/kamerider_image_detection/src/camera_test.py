import numpy as np
import cv2
cap=cv2.VideoCapture(1)
while True:
    sucess,img=cap.read()
    cv2.imshow("img",img)
    k=cv2.waitKey(10)
    if k == 27:
        cv2.destroyAllWindows()
        break
cap.release()

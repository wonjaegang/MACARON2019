import cv2
import numpy as np

cap = cv2.Videocapture(0)

while True:
    video = cap.read()
    cv2.imshow('video',video)
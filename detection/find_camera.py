import cv2

print("Checking available camera indices:")
for i in range(20):
    cap = cv2.VideoCapture(i)
    if cap.isOpened():
        print(f"Camera found at index {i}")
        cap.release()
import cv2
cap = cv2.VideoCapture(6)  # or 1, 2... depending on camera index

while True:
    ret, frame = cap.read()
    if not ret:
        break
    cv2.imshow("RealSense", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()
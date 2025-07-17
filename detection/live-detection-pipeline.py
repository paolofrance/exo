# Import the InferencePipeline object
from inference import InferencePipeline
# Import the built in render_boxes sink for visualizing results
from inference.core.interfaces.stream.sinks import render_boxes, multi_sink, UDPSink, VideoFileSink
# import VideoFrame for type hinting
from inference.core.interfaces.camera.entities import VideoFrame
from functools import partial

# import opencv to display our annotated images
import cv2
# import supervision to help visualize our predictions
import supervision as sv
import socket
import threading


# Initialize UDP socket
UDP_IP = "127.0.0.1"  # IP address of the receiving laptop
UDP_PORT = 9090         # Port number on the receiving laptop
# nc -ul 9090 to see the messages on the receiving laptop

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

import time
last_time = [time.time()]

# udp_sink = UDPSink.init(ip_address="192.168.1.57",port=9090)

# define sink function
def my_custom_sink(predictions: dict, video_frame: VideoFrame) ->None:
    try:
        # print(f"\n[DEBUG] New frame received: timestamp={video_frame.frame_timestamp}, ID={video_frame.frame_id}")
        current_time = time.time()
        fps = 1.0 / (current_time - last_time[0])
        # print(f"Inference FPS: {fps:.2f}")
        last_time[0] = current_time
        # print the frame ID of the video_frame object
        if predictions["predictions"]:
            num_bounding_boxes = len(predictions["predictions"])
            bounding_boxes_data = []  # Reset the list for each frame
            print(f"Number of boxes: {num_bounding_boxes}")
            for i in predictions["predictions"]:
                confidence = i["confidence"]
                class_name = i["class"]
                x = i["x"]
                y = i["y"]
                width = i["width"]
                height = i["height"]
                class_id = i["class_id"]

                # Print the accessed values
                print(f"Class: {class_name}, Confidence: {confidence}, Class_id: {class_id}")
                print(f"Coordinates: ({x}, {y}), Width: {width}, Height: {height}")
                # Append the bounding box data to the list
                bounding_boxes_data.append({
                    "confidence": confidence,
                    "class_name": class_name,
                    "x": x,
                    "y": y,
                    "width": width,
                    "height": height,
                    "class_id": class_id
                })
            detection_status = "not detected"
            for i in range(num_bounding_boxes):
                if (bounding_boxes_data[i]["class_id"] == 0):  #class_id = 0 >>> grasped    Class_id = 1 >>> not grasped
                    detection_status = "detected"
                    print("detected >>>>>confidence:",{bounding_boxes_data[i]["confidence"]})
            if (detection_status == "not detected"):
                print("not detected")
            message = f"{video_frame.frame_timestamp} : {detection_status}"
            detection_status = "not detected"
        else:
            detection_status = "not detected"
            print("not detected HELLO")
            message = f"{video_frame.frame_timestamp} : {detection_status}"
        sock.sendto(message.encode(), (UDP_IP, UDP_PORT))
        # print(f"[DEBUG] Sending message: {message}")

    except Exception as e:
        print(f"[ERROR] Sink exception: {e}")

    
# initialize a pipeline object
video_sink = VideoFileSink.init(video_file_name="output.avi")


def show_debug(frame_data):
    stream_id, frame = frame_data
    def show():
        print("[DEBUG] Opening cv2 window")
        cv2.imshow("Detection", frame)
        key = cv2.waitKey(1000)  # attendi almeno 1 secondo
        print(f"[DEBUG] Key pressed: {key}")
    threading.Thread(target=show).start()

pipeline = InferencePipeline.init(
    model_id="box-grasp/4",
    video_reference=6,
    # video_reference="http://10.11.65.86:4747/video", # DroidCam stream URL
    # on_prediction=partial(render_boxes, on_frame_rendered=show_debug),
    # on_prediction=partial(video_sink.on_prediction),
    on_prediction=partial(multi_sink, sinks=[my_custom_sink, video_sink.on_prediction]),
    confidence=0.7,
    api_key="HToOs7SUTDuQcOguPjm5",
    iou_threshold=0.7,
)

pipeline.start()
pipeline.join()

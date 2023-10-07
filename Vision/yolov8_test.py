import ultralytics
import cv2
import time
import numpy as np
import imutils
import torch

def generate_boxes_confidences_classids_v8(outs, threshold):
		boxes = []
		confidences = []
		classids = []

		for out in outs:
				for box in out.boxes:
					x1, y1, x2, y2 = [round(x) for x in box.xyxy[0].tolist()]
					class_id = box.cls[0].item()
					prob = round(box.conf[0].item(), 2)
					if prob > threshold:
						# Append to list
						boxes.append([x1, y1, x2-x1, y2-y1])
						confidences.append(float(prob))
						classids.append(class_id)
	
		return boxes, confidences, classids

def yolov8_warmup(model, repetitions=1, verbose=False):
    # Warmup model
    startTime = time.time()
    # create an empty frame to warmup the model
    for i in range(repetitions):
        warmupFrame = np.zeros((360, 640, 3), dtype=np.uint8)
        model.predict(source=warmupFrame, verbose=verbose)
    print(f"Model warmed up in {time.time() - startTime} seconds")
    
#print if cuda is available
print(f"CUDA is available: {torch.cuda.is_available()}")

#model = torch.hub.load("ultralytics/yolov5", "yolov5s")  # or yolov5n - yolov5x6, custom
print("Loading model...")
prevtime = time.time()
model = ultralytics.YOLO("yolov8n.pt")
# check if CUDA is available
yolov8_warmup(model, 10, False)
print(f"Model loaded in {time.time() - prevtime} seconds")

cap = cv2.VideoCapture(0)
ret, frame = cap.read()

while ret:
    prevtime = time.time()
    frame = imutils.resize(frame, width=640)
    try:
        results = model(frame, verbose=False)
    except Exception as e:
        ret, frame = cap.read()
        continue
    print(f"Prediction done in {time.time() - prevtime} seconds")
    # Results
    boxes, confidences, classids = generate_boxes_confidences_classids_v8(results, 0.3)
    # draw results with class names
    for i in range(len(boxes)):
        x, y, w, h = boxes[i]
        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
        class_name = model.names[classids[i]]
        cv2.putText(frame, class_name, (x, y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    #cv2.imshow("frame", frame)
    ret, frame = cap.read()
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
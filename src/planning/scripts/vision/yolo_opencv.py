import cv2
import argparse
import numpy as np
import pyrealsense2 as rs
from realsense import realsense

def get_output_layers(net):
    
    layer_names = net.getLayerNames()
    
    output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]

    return output_layers


def draw_prediction(classes, img, class_id, confidence, x, y, x_plus_w, y_plus_h, COLORS):

    label = str(classes[class_id])

    color = COLORS[class_id]

    cv2.rectangle(img, (x,y), (x_plus_w,y_plus_h), color, 2)

    #cv2.putText(img, label, (x-10,y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

def detect_trash():
	# Create a context object. This object owns the handles to all connected realsense devices
	pipeline = realsense()
	config = "/home/trashfleet/Documents/SwarmUnitROS/src/planning/scripts/vision/yolov3.cfg"
	weights = "/home/trashfleet/Documents/SwarmUnitROS/src/planning/scripts/vision/yolov3.weights"
	classes_path = "/home/trashfleet/Documents/SwarmUnitROS/src/planning/scripts/vision/yolov3.txt"

	for index in range(0,10):
		# Create a pipeline object. This object configures the streaming camera and owns it's handle
		image, depth = pipeline.get_rs_frames()

		image = image.get_data()
		image = np.asanyarray(image)
	
		Width = image.shape[1]
		Height = image.shape[0]
		scale = 0.00392

		classes = None
		with open(classes_path, 'r') as f:
			classes = [line.strip() for line in f.readlines()]

		COLORS = np.random.uniform(0, 255, size=(len(classes), 3))

		net = cv2.dnn.readNet(weights, config)

		blob = cv2.dnn.blobFromImage(image, scale, (416,416), (0,0,0), True, crop=False)

		net.setInput(blob)

		outs = net.forward(get_output_layers(net))

		class_ids = []
		confidences = []
		boxes = []
		conf_threshold = 0.5
		nms_threshold = 0.4


		for out in outs:
			for detection in out:
				scores = detection[5:]
				class_id = np.argmax(scores)
				confidence = scores[class_id]
				if confidence > 0.5:
					center_x = int(detection[0] * Width)
					center_y = int(detection[1] * Height)
					w = int(detection[2] * Width)
					h = int(detection[3] * Height)
					x = center_x - w / 2
					y = center_y - h / 2
					class_ids.append(class_id)
					confidences.append(float(confidence))
					boxes.append([x, y, w, h])


		indices = cv2.dnn.NMSBoxes(boxes, confidences, conf_threshold, nms_threshold)
		center = []
		for i in indices:
			i = i[0]
			box = boxes[i]
			x = box[0]
			y = box[1]
			w = box[2]
			h = box[3]
			if(class_ids[i] == 39):
				draw_prediction(classes, image, class_ids[i], confidences[i], int(x), int(y), int(x+w), int(y+h), COLORS)
			c1 = int(x+w/2)
			c2 = int(y+h/2)
			center.append((c1,c2))

		transformed_3ds = []
		for center_pixel in center:
			pixel_3d = pipeline.cord_3d(list(center_pixel))
			transformed_3ds.append(pixel_3d)
			# transformed_3d = transform(pixel_3d)
			# transformed_3ds.append(transformed_3d)
		
		print(transformed_3ds)
		
		if(len(transformed_3ds) > 0):
			break
		# cv2.resizeWindow("detect", 1280, 720)
		cv2.imshow('detect',image)
		cv2.waitKey()
		cv2.destroyAllWindows()
	return transformed_3ds
		

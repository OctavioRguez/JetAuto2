#!/usr/bin/python3
import cv2 as cv
import numpy as np

class modelPredict:
    # Attributes
    def __init__(self, model:str, class_list:list, colors:list, imgSz:int, conf_thres:float, cuda:bool) -> None:
        self.__model = model
        self.__class_list = class_list
        self.__colors = colors
        self.__imgSize = imgSz
        self.__conf = conf_thres
        self.__buildModel(cuda)

        self.__y = 0.0

    # Define if opencv runs with CUDA or CPU (False = CPU, True = CUDA)
    def __buildModel(self, is_cuda:bool) -> None:
        self.__net = cv.dnn.readNetFromONNX(self.__model)
        if is_cuda:
            print("Attempting to use CUDA")
            self.__net.setPreferableBackend(cv.dnn.DNN_BACKEND_CUDA)
            self.__net.setPreferableTarget(cv.dnn.DNN_TARGET_CUDA_FP16)
        else:
            print("Running on CPU")
            self.__net.setPreferableBackend(cv.dnn.DNN_BACKEND_OPENCV)
            self.__net.setPreferableTarget(cv.dnn.DNN_TARGET_CPU)
    
    # Format image to be used by the model
    def __formatYolov5(self, img:cv.Mat) -> cv.Mat:
        return cv.resize(img, [self.__imgSize, self.__imgSize], interpolation = cv.INTER_AREA)

    # Detect objects in the image
    def __detect(self, img:cv.Mat) -> np.ndarray:
        blob = cv.dnn.blobFromImage(img, 1/255.0, (self.__imgSize, self.__imgSize), swapRB=True, crop=False)
        self.__net.setInput(blob)
        preds = self.__net.forward(self.__net.getUnconnectedOutLayersNames())
        return preds

    # Wrap the detection process
    def __wrapDetection(self, modelOutput:np.ndarray, object:str) -> tuple:
        class_ids, boxes = [], []

        rows = modelOutput[0].shape[1]
        for r in range(rows):
            row = modelOutput[0][r]
            confidence = row[4]

            # Filer the detections
            if confidence > self.__conf:
                classes_scores = row[5:]
                class_id = cv.minMaxLoc(classes_scores)[3][1]

                if (self.__class_list[class_id] == object) and (classes_scores[class_id] > self.__conf):
                    class_ids.append(class_id)
                    x, y, w, h = row[0].item(), row[1].item(), row[2].item(), row[3].item() 
                    left = int((x - 0.5 * w))
                    top = int((y - 0.5 * h))
                    width, height = int(w), int(h)
                    box = np.array([left, top, width, height])
                    boxes.append(box)

        result_class_ids, result_boxes = [], []
        for i in range(len(boxes)):
            result_class_ids.append(class_ids[i])
            result_boxes.append(boxes[i])
        return result_class_ids, result_boxes

    # Start the detection process
    def _startDetection(self, img:cv.Mat, object:str, width:float) -> None:
        formatImg = self.__formatYolov5(img)
        outs = self.__detect(formatImg)
        class_ids, boxes = self.__wrapDetection(outs[0], object)
        
        for (classid, box) in zip(class_ids, boxes):
            color = self.__colors[int(classid) % len(self.__colors)]
            cv.rectangle(formatImg, box, color, 2)
            cv.rectangle(formatImg, (box[0], box[1] - 20), (box[0] + box[2], box[1]), color, -1)
            cv.putText(formatImg, object, (box[0], box[1] - 10), cv.FONT_HERSHEY_SIMPLEX, .5, (0,0,0))

            # Calculate pixels to meters ratio
            PixToMeters = width / box[2]
            self.__y = PixToMeters*(box[0] + box[2] - self.__imgSize)/2
        cv.imshow('Classificator', formatImg)
        cv.waitKey(1)

    # Get the Y coordinate of the object
    def getY(self) -> float:
        return self.__y

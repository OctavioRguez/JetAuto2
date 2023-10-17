#!/usr/bin/python3
import cv2 as cv
import numpy as np

class modelPredict:
    # Variables
    def __init__(self, model:str, class_list:list, colors:list) -> None:
        self.__width = 640
        self.__height = 640

        self.__model = model
        self.__class_list = class_list
        self.__colors = colors
        
        self.__y = 0.0

    def __buildModel(self, is_cuda:bool) -> cv.dnn_Net:
        net = cv.dnn.readNet(self.__model)
        if is_cuda:
            print("Attempting to use CUDA")
            net.setPreferableBackend(cv.dnn.DNN_BACKEND_CUDA)
            net.setPreferableTarget(cv.dnn.DNN_TARGET_CUDA_FP16)
        else:
            print("Running on CPU")
            net.setPreferableBackend(cv.dnn.DNN_BACKEND_OPENCV)
            net.setPreferableTarget(cv.dnn.DNN_TARGET_CPU)
        return net
    
    def __detect(self, image:cv.Mat, net:cv.dnn_Net) -> np.ndarray:
        blob = cv.dnn.blobFromImage(image, 1/255.0, (self.__width, self.__height), swapRB=True, crop=False)
        net.setInput(blob)
        preds = net.forward()
        return preds

    def __wrap_detection(self, image:cv.Mat, output:np.ndarray) -> tuple:
        class_ids, confidences, boxes = [], [], []

        rows = output.shape[0]
        image_width, image_height = image.shape[:2]

        x_factor = image_width / self.__width
        y_factor =  image_height / self.__height

        for r in range(rows):
            row = output[r]
            confidence = row[4]

            if confidence > 0.5:
                classes_scores = row[5:]
                max_indx = cv.minMaxLoc(classes_scores)[3]
                class_id = max_indx[1]

                if (classes_scores[class_id] > 0.25):
                    confidences.append(confidence)
                    class_ids.append(class_id)
                    x, y, w, h = row[0].item(), row[1].item(), row[2].item(), row[3].item() 
                    left = int((x - 0.5 * w) * x_factor)
                    top = int((y - 0.5 * h) * y_factor)
                    width = int(w * x_factor)
                    height = int(h * y_factor)
                    box = np.array([left, top, width, height])
                    boxes.append(box)

        indexes = cv.dnn.NMSBoxes(boxes, confidences, 0.25, 0.45) 

        result_class_ids = []
        result_confidences = []
        result_boxes = []

        for i in indexes:
            result_confidences.append(confidences[i])
            result_class_ids.append(class_ids[i])
            result_boxes.append(boxes[i])

        return result_class_ids, result_confidences, result_boxes

    def __format_yolov5(self, img:cv.Mat) -> cv.Mat:
        return cv.resize(img, [640, 640], interpolation = cv.INTER_AREA)

    def _startDetection(self, object:str, img:cv.Mat, width:float) -> None:
        net = self.__buildModel(True) # Define if opencv runs with CUDA or CPU (False = CPU, True = CUDA)
        formatImg = self.__format_yolov5(img)
        outs = self.__detect(formatImg, net)
        class_ids, confidences, boxes = self.__wrap_detection(formatImg, outs[0])
        
        for (classid, confidence, box) in zip(class_ids, confidences, boxes):
            if object == self.__class_list[classid]:
                color = self.__colors[int(classid) % len(self.__colors)]
                cv.rectangle(formatImg, box, color, 2)
                cv.rectangle(formatImg, (box[0], box[1] - 20), (box[0] + box[2], box[1]), color, -1)
                cv.putText(formatImg, self.__class_list[classid], (box[0], box[1] - 10), cv.FONT_HERSHEY_SIMPLEX, .5, (0,0,0))

                # Calculate pixels to meters ratio
                PixToMeters = width / box[2]
                self.__y = ((box[0] + box[2] / 2) - self.__width/2)*PixToMeters
        cv.imshow('Classificator', formatImg)
        cv.waitKey(1)

    def getY(self) -> float:
        return self.__y

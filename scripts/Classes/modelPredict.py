#!/usr/bin/python3
import cv2 as cv
import numpy as np
import onnxruntime as ort

# Classificator class
class modelPredict:
    def __init__(self, model:str, class_list:list, conf_thres:float, cuda:bool) -> None:
        self.__model = model
        self.__class_list = class_list
        self.__colors = np.random.uniform(0, 255, size=(len(self.__class_list), 3))
        self.__conf = conf_thres
        self.__buildModel(cuda)

        self.__y = 0.0

    # Define if opencv runs with CUDA or CPU (False = CPU, True = CUDA)
    def __buildModel(self, is_cuda:bool) -> None:
        if is_cuda:
            print("Attempting to use CUDA")
            self.__session = ort.InferenceSession(self.__model, providers=['CUDAExecutionProvider'])
        else:
            print("Running on CPU")
            self.__session = ort.InferenceSession(self.__model, providers=['CPUExecutionProvider'])
        shape = self.__session.get_inputs()[0].shape
        self.__inputWidth, self.__inputHeight = shape[2:4]

    # Format image to be used by the model
    def __formatImg(self, img:cv.Mat) -> np.ndarray:
        image = cv.cvtColor(img, cv.COLOR_BGR2RGB)
        image = np.array(cv.resize(image, [self.__inputWidth, self.__inputHeight])) / 255.0
        image = np.transpose(image, (2, 0, 1))
        return np.expand_dims(image, axis=0).astype(np.float32)

    # Detect objects in the image
    def __detect(self, img:cv.Mat) -> np.ndarray:
        inputs = {self.__session.get_inputs()[0].name: img}
        preds = self.__session.run(None, inputs)
        return np.squeeze(preds[0])

    # Wrap the detection process
    def __wrapDetection(self, modelOutput:np.ndarray, object:str) -> tuple:
        class_ids, boxes, scores = [], [], []

        x_factor = self.__imgWidth / self.__inputWidth
        y_factor = self.__imgHeight / self.__inputHeight

        rows = modelOutput.shape[0]
        for r in range(rows):
            row = modelOutput[r]
            
            if row[4] > self.__conf:
                classes_scores = row[5:]
                class_id = np.argmax(classes_scores)
                max_score = classes_scores[class_id]

                if (max_score > self.__conf) and (self.__class_list[class_id] == object):
                    x, y, w, h = row[0].item(), row[1].item(), row[2].item(), row[3].item()

                    left = (x - 0.5 * w) * x_factor
                    top = (y - 0.5 * h) * y_factor
                    width, height = w*x_factor, h*y_factor

                    class_ids.append(class_id)
                    scores.append(max_score)
                    boxes.append(np.array([left, top, width, height]))

        indices = cv.dnn.NMSBoxes(boxes, scores, self.__conf, 0.5)

        result_class_ids, result_boxes, result_scores = [], [], []
        for i in indices:
            result_class_ids.append(class_ids[i])
            result_boxes.append(boxes[i])
            result_scores.append(scores[i])
        return result_class_ids, result_boxes, result_scores

    # Start the detection process
    def _startDetection(self, img:cv.Mat, object:str, width:float) -> None:
        self.__imgHeight, self.__imgWidth = img.shape[:2]
        formatImg = self.__formatImg(img)
        outs = self.__detect(formatImg)
        class_ids, boxes, scores = self.__wrapDetection(outs, object)
        
        for (classid, box, score) in zip(class_ids, boxes, scores):
            x, y, w, h = box
            color = self.__colors[classid]
            cv.rectangle(img, (int(x), int(y)), (int(x + w), int(y + h)), color, 2)
            cv.rectangle(img, (int(x), int(y) - 20), (int(x) + int(w), int(y) + 20), color, -1)
            cv.putText(img, f'{object}: {score:.2f}', (x, y - 10), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 1, cv.LINE_AA)

            # Calculate pixels to meters ratio
            PixToMeters = width / w
            self.__y = PixToMeters*(x + w - self.__imgWidth)/2
        cv.imshow('Classificator', img)
        cv.waitKey(1)

    # Get the Y coordinate of the object
    def getY(self) -> float:
        return self.__y

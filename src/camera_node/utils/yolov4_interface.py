import onnxruntime as ort
import numpy as np
import cv2

class YOLOv4:
    def __init__(self, model_path):
        self.session = ort.InferenceSession(model_path)
        self.input_name = self.session.get_inputs()[0].name

    def preprocess(self, image):
        # Resize and normalize to 608,608 BGR -> RGB
        img = cv2.resize(image, (608, 608))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        img = np.transpose(img, (2, 0, 1))  # HWC to CHW
        img = np.expand_dims(img, axis=0)  # Add batch dim
        return img

    def postprocess(self, output, original_shape):
        # This will depend on your specific ONNX model’s output shape.
        # Simplified: just returns raw predictions.
        return output

    def infer(self, image):
        input_tensor = self.preprocess(image)
        outputs = self.session.run(None, {self.input_name: input_tensor})
        return self.postprocess(outputs, image.shape)

if __name__ == "__main__":
    # Example usage
    model_path = "/home/computer/ros2-vision/models/yolov4_416_416.onnx"
    detector = YOLOv4(model_path)
    print("Model loaded successfully.")
    # Load an image

    image = cv2.imread("/home/computer/ros2-vision/dog.jpg")
    print("Image shape:", image.shape)
    # Perform inference
    results = detector.infer(image)
    
    print("Detections:", results)
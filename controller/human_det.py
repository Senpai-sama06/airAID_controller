from ultralytics import YOLO
import cv2
import matplotlib.pyplot as plt

def detect_objects(image_path):
    model = YOLO('yolov8n.pt')  
    results = model(image_path)

    for result in results:
        boxes = result.boxes
        img = result.plot()  
        cv2.imshow("YOLOv8 Detection", img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        result.save(filename='output.jpg')
        print("Saved output as output.jpg")



image_path = 'Drone_Project\pic1.png'  # Replace with image path
detect_objects(image_path)


'''
Add functionality to geocode the humans detected through the model.
'''



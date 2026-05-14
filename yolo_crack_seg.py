from ultralytics import YOLO

# Load a model
# Using a pretrained model like yolo26n-seg.pt is recommended for faster convergence
model = YOLO("yolo26n-seg.pt")

# Train the model on the Crack Segmentation dataset
# Ensure 'crack-seg.yaml' is accessible or provide the full path
results = model.train(data="crack-seg.yaml", epochs=100, imgsz=640)

# After training, the model can be used for prediction or exported
# results = model.predict(source='path/to/your/images')
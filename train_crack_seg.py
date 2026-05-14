import os
import sys
from ultralytics import YOLO

def main():
    print("==========================================================")
    print(" CRACK SEGMENTATION MODEL TRAINING (ultralytics)")
    print("==========================================================")
    print("Downloading the 'crack-seg' semantic segmentation dataset...")
    print("Initializing YOLOv8 Nano Segment architecture...\n")
    
    try:
        # Load a pretrained YOLOv8 Nano segmentation model to act as the architecture base
        model = YOLO("yolov8n-seg.pt")
        
        # Initiate the automated training routine
        # `crack-seg.yaml` is baked into the Ultralytics ecosystem so it automatically downloads
        results = model.train(
            data="crack-seg.yaml", 
            epochs=50,          # Reduced slightly from 100 to ensure faster local testing
            imgsz=640,          # Standard input resolution
            device=0,           # Forces execution on your CUDA GPU
            patience=10         # Enable early stopping if accuracy stops improving
        )
        
        print("\n==========================================================")
        print(" TRAINING SUCCESSFUL! ")
        print("==========================================================")
        print("The customized weights have been saved. We will integrate these into the GUI next.")
    except Exception as e:
        print(f"\n[ERROR] An issue occurred during training: {e}")
        sys.exit(1)

if __name__ == '__main__':
    main()

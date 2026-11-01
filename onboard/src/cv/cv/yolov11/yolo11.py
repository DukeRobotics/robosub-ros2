from ultralytics import YOLO

# Load a YOLO11n PyTorch model
model = YOLO('yolo11n.pt')

# Export the model to TensorRT with DLA enabled (only works with FP16 or INT8)
model.export(format='engine', device='dla:0', half=True)  # dla:0 or dla:1 corresponds to the DLA cores

# Load the exported TensorRT model
trt_model = YOLO('yolo11n.engine')

# Run inference
results = trt_model('https://ultralytics.com/images/bus.jpg')

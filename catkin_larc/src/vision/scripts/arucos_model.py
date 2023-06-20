import torch
import torchvision.transforms as transforms
import cv2

# Load the pre-trained model
model = torch.load('best.pt')
model.eval()

# Define the transformation to apply to the input image
transform = transforms.Compose([
    transforms.ToTensor(),
    transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
])

# Open the camera
camera = cv2.VideoCapture(0)

while True:
    # Read a frame from the camera
    ret, frame = camera.read()
    
    # Preprocess the frame
    input_tensor = transform(frame)
    input_batch = input_tensor.unsqueeze(0)
    
    # Run the model inference
    with torch.no_grad():
        output = model(input_batch)
    
    # Process the model output (e.g., draw bounding boxes)
    # ...
    
    # Display the resulting frame
    cv2.imshow('Object Detection', frame)
    
    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close all windows
camera.release()
cv2.destroyAllWindows()

import os
import cv2
import imutils

def resize_images_in_directory(input_dir, output_dir, target_height=720):
    # Create the output directory if it doesn't exist
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    # Get a list of all files in the input directory
    file_list = os.listdir(input_dir)

    for file_name in file_list:
        # Check if the file is an image (you can add more image extensions as needed)
        if file_name.lower().endswith(('.jpg', '.jpeg', '.png', '.bmp', '.gif')):
            input_path = os.path.join(input_dir, file_name)
            output_path = os.path.join(output_dir, file_name)

            # Read the image
            image = cv2.imread(input_path)

            if image is not None:
                # Resize the image to the target height while maintaining the aspect ratio
                resized = imutils.resize(image, height=target_height)

                # Save the resized image
                cv2.imwrite(output_path, resized)
                print(f"Resized and saved: {output_path}")
            else:
                print(f"Failed to read image: {input_path}")

if __name__ == "__main__":
    input_directory = "/Volumes/HDD/Dataset_final_yolov8_png/"  # Replace with the path to your input directory
    output_directory = "/Volumes/HDD/Dataset_final_yolo_resize_png/"  # Replace with the desired output directory
    target_height = 720  # Specify the target height in pixels

    resize_images_in_directory(input_directory, output_directory, target_height)

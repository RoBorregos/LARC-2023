from PIL import Image
import os

datasetpath = "/home/jabv/Desktop/Dataset_final_yolov8_png_no_bg/"
resultspath = "/home/jabv/Desktop/cropped_imgs/"

fg_folders = [
    ("A/"),
    ("B/"),
    ("C/"),
    ("D/"),
    ("E/"),
    ("F/"),
    ("G/"),
    ("H/"),
    ("I/")
]

for folder in fg_folders:
    for filename in os.listdir(f"{datasetpath}{folder}"):
        try:
            print(f"{filename} started")
            myImage = Image.open(datasetpath+folder+filename)
            black = Image.new('RGBA', myImage.size)
            myImage = Image.composite(myImage, black, myImage)
            print("aqui")
            myCroppedImage = myImage.crop(myImage.getbbox())
            myCroppedImage.save(f"{resultspath}{folder}{filename}")
            print(f"{filename} done")
        except:
            print(f"{filename} failed")
            continue
print("all done")
from PIL import Image, ImageDraw, ImageFont
import random

# Tamaño de la imagen
for i in range(10000):
    width = 500
    height = 500

    # Crear una nueva imagen
    img = Image.new('RGB', (width, height), color = (255, 255, 255))

    # Crear objeto de dibujo
    d = ImageDraw.Draw(img)

    # Elegir un tamaño y posición aleatoria para la letra
    font_size = random.randint(50, 200)
    x = random.randint(0, width - font_size)
    y = random.randint(0, height - font_size)

    # Crear objeto de fuente con Arial y el tamaño aleatorio
    font = ImageFont.truetype("/Arial.ttf", font_size)

    # Dibujar la letra "A" en la posición aleatoria
    d.text((x, y), "A", fill=(0, 0, 0), font=font)

    # Guardar la imagen
    img.save("/Users/josebenvenuto/Desktop/Letras_dataset/A/imagen_A" + str(i) + ".png")

    # Dibujar la letra "A" en la posición aleatoria
    d.text((x, y), "B", fill=(0, 0, 0), font=font)

    # Guardar la imagen
    img.save("/Users/josebenvenuto/Desktop/Letras_dataset/B/imagen_B" + str(i) + ".png")
    
    # Dibujar la letra "A" en la posición aleatoria
    d.text((x, y), "C", fill=(0, 0, 0), font=font)

    # Guardar la imagen
    img.save("/Users/josebenvenuto/Desktop/Letras_dataset/C/imagen_C" + str(i) + ".png")
    
    # Dibujar la letra "A" en la posición aleatoria
    d.text((x, y), "D", fill=(0, 0, 0), font=font)

    # Guardar la imagen
    img.save("/Users/josebenvenuto/Desktop/Letras_dataset/D/imagen_D" + str(i) + ".png")
    
      # Dibujar la letra "A" en la posición aleatoria
    d.text((x, y), "E", fill=(0, 0, 0), font=font)

    # Guardar la imagen
    img.save("/Users/josebenvenuto/Desktop/Letras_dataset/E/imagen_E" + str(i) + ".png")
    
      # Dibujar la letra "A" en la posición aleatoria
    d.text((x, y), "F", fill=(0, 0, 0), font=font)

    # Guardar la imagen
    img.save("/Users/josebenvenuto/Desktop/Letras_dataset/F/imagen_F" + str(i) + ".png")
    
      # Dibujar la letra "A" en la posición aleatoria
    d.text((x, y), "G", fill=(0, 0, 0), font=font)

    # Guardar la imagen
    img.save("/Users/josebenvenuto/Desktop/Letras_dataset/G/imagen_G" + str(i) + ".png")
    
      # Dibujar la letra "A" en la posición aleatoria
    d.text((x, y), "H", fill=(0, 0, 0), font=font)

    # Guardar la imagen
    img.save("/Users/josebenvenuto/Desktop/Letras_dataset/H/imagen_H" + str(i) + ".png")
    
      # Dibujar la letra "A" en la posición aleatoria
    d.text((x, y), "I", fill=(0, 0, 0), font=font)

    # Guardar la imagen
    img.save("/Users/josebenvenuto/Desktop/Letras_dataset/I/imagen_I" + str(i) + ".png")
    
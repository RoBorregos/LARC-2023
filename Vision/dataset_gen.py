
from PIL import Image, ImageDraw, ImageFont
import random

# Tamaño de la imagen
for i in range(10):
    print(i)
    width = 500
    height = 500

    # Crear una nueva imagen
    #img = Image.new('RGB', (width, height), color = (255, 255, 255))

    # Crear objeto de dibujo
    #d = ImageDraw.Draw(img)

    # Elegir un tamaño y posición aleatoria para la letra
    font_size = random.randint(50, 200)

    # Crear objeto de fuente con Arial y el tamaño aleatorio
    #print("aaa")
    font = ImageFont.truetype("/usr/share/fonts/truetype/msttcorefonts/Arial.ttf", font_size)
    img = Image.new('RGB', (width, height), color = (255, 255, 255))
    
    d = ImageDraw.Draw(img)

    # Elegir un tamaño y posición aleatoria para la letra
    text_width, text_height = d.textsize("A", font=font)
    x = random.randint(0, width - text_width)
    y = random.randint(0, height - text_width)

    # Dibujar la letra "A" en la posición aleatoria
    d.text((x, y), "A", fill=(0, 0, 0), font=font)

    # Generar un ángulo de rotación aleatorio
    angle = random.randint(-30, 30)

    # Rotar la letra dibujada por el ángulo generado
    img = img.rotate(angle, expand=True)

    # Guardar la imagen
    img.save("/home/jabv/Desktop/tempo/A" + str(i) + ".png")
    
    img = Image.new('RGB', (width, height), color = (255, 255, 255))
    
    d = ImageDraw.Draw(img)
    text_width, text_height = d.textsize("B", font=font)
    x = random.randint(0, width - text_width)
    y = random.randint(0, height - text_width)
    # Elegir un tamaño y posición aleatoria para la letra


    # Dibujar la letra "A" en la posición aleatoria
    d.text((x, y), "B", fill=(0, 0, 0), font=font)

    # Generar un ángulo de rotación aleatorio
    angle = random.randint(-30, 30)

    # Rotar la letra dibujada por el ángulo generado
    img = img.rotate(angle, expand=True)

    # Guardar la imagen
    img.save("/home/jabv/Desktop/tempo/B" + str(i) + ".png")
    
    img = Image.new('RGB', (width, height), color = (255, 255, 255))
    
    d = ImageDraw.Draw(img)
    text_width, text_height = d.textsize("C", font=font)
    x = random.randint(0, width - text_width)
    y = random.randint(0, height - text_width)
    # Elegir un tamaño y posición aleatoria para la letra
    # Dibujar la letra "A" en la posición aleatoria
    d.text((x, y), "C", fill=(0, 0, 0), font=font)

    # Generar un ángulo de rotación aleatorio
    angle = random.randint(-30, 30)

    # Rotar la letra dibujada por el ángulo generado
    img = img.rotate(angle, expand=True)

    # Guardar la imagen
    img.save("/home/jabv/Desktop/tempo/C" + str(i) + ".png")
    
    img = Image.new('RGB', (width, height), color = (255, 255, 255))
    
    d = ImageDraw.Draw(img)
    text_width, text_height = d.textsize("D", font=font)
    x = random.randint(0, width - text_width)
    y = random.randint(0, height - text_width)
    # Elegir un tamaño y posición aleatoria para la letra
    # Dibujar la letra "A" en la posición aleatoria
    d.text((x, y), "D", fill=(0, 0, 0), font=font)

    # Generar un ángulo de rotación aleatorio
    angle = random.randint(-30, 30)

    # Rotar la letra dibujada por el ángulo generado
    img = img.rotate(angle, expand=True)

    # Guardar la imagen
    img.save("/home/jabv/Desktop/tempo/D" + str(i) + ".png")
    
    img = Image.new('RGB', (width, height), color = (255, 255, 255))
    
    d = ImageDraw.Draw(img)
    text_width, text_height = d.textsize("E", font=font)
    x = random.randint(0, width - text_width)
    y = random.randint(0, height - text_width)

    
      # Dibujar la letra "A" en la posición aleatoria
    d.text((x, y), "E", fill=(0, 0, 0), font=font)

    # Generar un ángulo de rotación aleatorio
    angle = random.randint(-30, 30)

    # Rotar la letra dibujada por el ángulo generado
    img = img.rotate(angle, expand=True)

    # Guardar la imagen
    img.save("/home/jabv/Desktop/tempo/E" + str(i) + ".png")
    
    img = Image.new('RGB', (width, height), color = (255, 255, 255))
    
    d = ImageDraw.Draw(img)
    text_width, text_height = d.textsize("F", font=font)
    x = random.randint(0, width - text_width)
    y = random.randint(0, height - text_width)
    
      # Dibujar la letra "A" en la posición aleatoria
    d.text((x, y), "F", fill=(0, 0, 0), font=font)

    # Generar un ángulo de rotación aleatorio
    angle = random.randint(-30, 30)

    # Rotar la letra dibujada por el ángulo generado
    img = img.rotate(angle, expand=True)

    # Guardar la imagen
    img.save("/home/jabv/Desktop/tempo/F" + str(i) + ".png")
    
    img = Image.new('RGB', (width, height), color = (255, 255, 255))
    
    d = ImageDraw.Draw(img)
    text_width, text_height = d.textsize("G", font=font)
    x = random.randint(0, width - text_width)
    y = random.randint(0, height - text_width)

    
      # Dibujar la letra "A" en la posición aleatoria
    d.text((x, y), "G", fill=(0, 0, 0), font=font)

    # Generar un ángulo de rotación aleatorio
    angle = random.randint(-30, 30)

    # Rotar la letra dibujada por el ángulo generado
    img = img.rotate(angle, expand=True)

    # Guardar la imagen
    img.save("/home/jabv/Desktop/tempo/G" + str(i) + ".png")
    
    img = Image.new('RGB', (width, height), color = (255, 255, 255))
    
    d = ImageDraw.Draw(img)

    text_width, text_height = d.textsize("H", font=font)
    x = random.randint(0, width - text_width)
    y = random.randint(0, height - text_width)

    
      # Dibujar la letra "A" en la posición aleatoria
    d.text((x, y), "H", fill=(0, 0, 0), font=font)

    # Generar un ángulo de rotación aleatorio
    angle = random.randint(-30, 30)

    # Rotar la letra dibujada por el ángulo generado
    img = img.rotate(angle, expand=True)

    # Guardar la imagen
    img.save("/home/jabv/Desktop/tempo/H" + str(i) + ".png")
    
    img = Image.new('RGB', (width, height), color = (255, 255, 255))
    
    d = ImageDraw.Draw(img)

    text_width, text_height = d.textsize("I", font=font)
    x = random.randint(0, width - text_width)
    y = random.randint(0, height - text_width)
    
      # Dibujar la letra "A" en la posición aleatoria
    d.text((x, y), "I", fill=(0, 0, 0), font=font)

    # Generar un ángulo de rotación aleatorio
    angle = random.randint(-30, 30)

    # Rotar la letra dibujada por el ángulo generado
    img = img.rotate(angle, expand=True)

    # Guardar la imagen
    img.save("/home/jabv/Desktop/tempo/I" + str(i) + ".png")
    
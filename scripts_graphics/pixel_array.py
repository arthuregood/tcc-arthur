from PIL import Image
import numpy as np
import matplotlib.pyplot as plt


# Caminho da imagem
image_path = 'src/tcc/worlds/map_2.png'

# Carregar a imagem
image = Image.open(image_path)

# Converter a imagem para escala de cinza para trabalhar com um único canal
image_gray = image.convert('L')

# Transformar a imagem em um array de pixels
pixels_array = np.array(image_gray)

# Criar um array booleano: True para pixels preenchidos (paredes) e False para não preenchidos (espaço livre)
# Consideramos um pixel como "preenchido" se ele for significativamente mais escuro que o branco puro (255)
threshold = 254
filled_pixels = pixels_array < threshold

# Vamos ver o tamanho do array e um preview dos valores
print(filled_pixels.shape)
print(filled_pixels)  # Mostrar o tamanho e os primeiros 5x5 pixels

plt.imshow(filled_pixels, cmap='gray')
plt.title('Mapa binário do labirinto')
plt.show()
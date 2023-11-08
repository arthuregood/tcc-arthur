from PIL import Image
import numpy as np
import matplotlib.pyplot as plt
from scipy.ndimage import gaussian_filter


image_path = 'src/tcc/worlds/map_1.png'
maze_image = Image.open(image_path)

# Converter a imagem para escala de cinza
maze_gray = maze_image.convert('L')

# Binarizar a imagem: pixels brancos (chão) serão 0, e tudo mais será 1
threshold = 1 
maze_binary = np.where(np.array(maze_gray) > threshold, 0, 1)

plt.imshow(maze_binary, cmap='gray')
plt.title('Mapa binário do labirinto')
plt.show()

# Definir valores de potencial alto e baixo
wall_potential = 100
free_space_potential = 0

# Criar um mapa de potenciais inicial baseado na imagem binarizada
potential_map = np.where(maze_binary == 1, wall_potential, free_space_potential)

# Suavizar o mapa de potenciais para criar um gradiente usando filtro gaussiano
sigma = 1.5
smoothed_potential_map = gaussian_filter(potential_map, sigma=sigma)

# Visualização do mapa de campos potenciais
plt.imshow(smoothed_potential_map, cmap='hot')
plt.colorbar(label='Potencial')
plt.title('Mapa de campos potenciais')
plt.show()

# Definir a posição do objetivo (x, y)
goal_position = (50, 10)

# Criar um campo potencial atrativo que diminui com a distância do objetivo
def attractive_potential_field(potential_map, goal, strength=1):

    x_indices, y_indices = np.indices(potential_map.shape)
    distance_x = (x_indices - goal[0]) ** 2
    distance_y = (y_indices - goal[1]) ** 2
    distance = np.sqrt(distance_x + distance_y)
    potential_field = strength * distance
    potential_field -= potential_field.min()  # Normalizar para ter o mínimo no objetivo
    return potential_field

# Calcular o campo potencial atrativo
attr_potential_map = attractive_potential_field(smoothed_potential_map, goal_position)

# Somar o campo potencial atrativo ao mapa de campos potenciais já existente,
# invertendo o campo atrativo para que o objetivo seja uma região de baixo potencial
combined_potential_map = smoothed_potential_map - attr_potential_map
combined_potential_map -= combined_potential_map.min()  # Normalizar

# Visualizar o resultado
plt.imshow(combined_potential_map, cmap='hot')
plt.colorbar(label='Potencial')
plt.plot(goal_position[0], goal_position[1], 'go')  # Marcar o objetivo com um ponto verde
plt.title('Mapa de campos potenciais com objetivo atrativo')
plt.show()
from PIL import Image
import numpy as np
import matplotlib.pyplot as plt

image = Image.open("src/tcc/worlds/map_1.png")

image_gray = image.convert('L')

pixels_array = np.array(image_gray)

threshold = 254
filled_pixels = pixels_array < threshold
print(filled_pixels)


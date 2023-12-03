import open3d as o3d
import cv2
import numpy as np
import matplotlib.pyplot as plt
import io

depth_raw = o3d.io.read_image("/Users/filippoferrari/Desktop/University/Smart Robotics/PROJECT/Open3D/examples/python/reconstruction_system/dataset/test/depth/00000.png")
color_raw = o3d.io.read_image("/Users/filippoferrari/Desktop/University/Smart Robotics/PROJECT/Open3D/examples/python/reconstruction_system/dataset/test/image/00000.jpg")
# depth_raw = o3d.io.read_image("/Users/filippoferrari/Desktop/SRproject/dataset/beer_depth/depth/00003.png")
# color_raw = o3d.io.read_image("/Users/filippoferrari/Desktop/SRproject/dataset/beer_depth/image/00003.jpg")

rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
    color_raw, depth_raw)

print(rgbd_image)

plt.subplot(1, 2, 1)
plt.title('Grayscale image')
plt.imshow(rgbd_image.color)
plt.subplot(1, 2, 2)
plt.title('Depth image')
plt.imshow(rgbd_image.depth)
plt.show()


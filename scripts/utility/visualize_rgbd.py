import open3d as o3d
import matplotlib.pyplot as plt

depth_raw = o3d.io.read_image("/Users/filippoferrari/Desktop/SRproject/dataset/beer/image/0.jpg")
color_raw = o3d.io.read_image("/Users/filippoferrari/Desktop/SRproject/dataset/beer/depth/0.png")

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


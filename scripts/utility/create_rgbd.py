import open3d as o3d
import numpy as np
from PIL import Image

def create_rgbd_image(rgb_path, depth_path):
    # Carica le immagini RGB e di profondità
    rgb_image = Image.open(rgb_path)
    depth_image = Image.open(depth_path)

    # Converte le immagini in array NumPy
    rgb_array = np.asarray(rgb_image)
    depth_array = np.asarray(depth_image)

    # Crea il cloud di punti a partire dall'immagine di profondità
    depth_o3d = o3d.geometry.Image(depth_array)
    rgb_o3d = o3d.geometry.Image(rgb_array)

    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
        rgb_o3d, depth_o3d, depth_scale=1000.0, depth_trunc=3.0, convert_rgb_to_intensity=False
    )

    return rgbd_image

def save_rgbd_image(rgbd_image, output_path):
    # Salva l'immagine RGBD in formato .png
    o3d.io.write_rgbd_image(output_path, rgbd_image)

def main():
    # Specifica i percorsi delle immagini RGB e di profondità
    rgb_path = ""
    depth_path = ""

    # Crea l'immagine RGBD
    rgbd_image = create_rgbd_image(rgb_path, depth_path)

    # Salva l'immagine RGBD
    output_path = ""
    save_rgbd_image(rgbd_image, output_path)


    # Visualizza l'immagine RGBD
    o3d.visualization.draw_geometries([o3d.geometry.TriangleMesh.create_coordinate_frame(), o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image)])

if __name__ == "__main__":
    main()

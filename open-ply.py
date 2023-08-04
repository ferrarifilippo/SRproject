import os
import open3d as o3d
import numpy as np

def main():

    pcd = o3d.io.read_point_cloud("/Users/filippoferrari/open3d_data/extract/SampleRedwoodRGBDImages/scene/integrated.ply")

    o3d.visualization.draw_geometries([pcd])

if __name__ == "__main__":
    main()

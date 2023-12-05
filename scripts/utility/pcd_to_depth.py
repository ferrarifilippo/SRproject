import open3d as o3d

# Project into images
depth_list = ['/Users/filippoferrari/Desktop/SRproject/beer/depth/%d.png' % i for i in range(4)]
jpg_list = ['/Users/filippoferrari/Desktop/SRproject/beer/image/%d.jpg' % i for i in range(4)]
pcd_list = ['/Users/filippoferrari/Desktop/SRproject/beer/conversion_result_%d.pcd' % i for i in range(4)]

for i in range(4):
   print(i)
   pcd_temp = o3d.io.read_point_cloud(pcd_list[i], format="pcd")
   pcd_temp.transform([[1.0, 0.0, 0.0, 0.0], [0.0, -1.0, 0.0, 0.0], [0.0, 0.0, -1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
   vis = o3d.visualization.Visualizer()
   vis.create_window()
   vis.get_render_option().point_color_option = o3d.visualization.PointColorOption.Color
   vis.get_render_option().point_size = 3.0
   vis.add_geometry(pcd_temp)

   # Capture rgb image
   vis.capture_screen_image(jpg_list[i], do_render=True)
   # Capture depth image
   vis.capture_depth_image(depth_list[i], do_render=True)
   vis.destroy_window()


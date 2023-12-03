import open3d as o3d

# Project into images
# pcd_list = ["/Users/filippoferrari/Desktop/SRproject/dataset/beer_move/conversion_result_0.pcd", "/Users/filippoferrari/Desktop/SRproject/dataset/beer_move/conversion_result_1.pcd"]#, "/Users/filippoferrari/Desktop/SRproject/dataset/beer1/conversion_result_2.pcd"]
# jpg_list = ["/Users/filippoferrari/Desktop/SRproject/dataset/beer_move/image/0.jpg", "/Users/filippoferrari/Desktop/SRproject/dataset/beer_move/image/1.jpg"]#, "/Users/filippoferrari/Desktop/SRproject/dataset/test/image/2.jpg"]
# depth_list = ["/Users/filippoferrari/Desktop/SRproject/dataset/beer_move/depth/0.png", "/Users/filippoferrari/Desktop/SRproject/dataset/beer_move/depth/1.png"]#, "/Users/filippoferrari/Desktop/SRproject/dataset/test/depth/2.png"]
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


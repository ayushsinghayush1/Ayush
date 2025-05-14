import numpy as np
import open3d as o3d

# Define the file paths
intrinsics_file = 'intrinsics.npy'
depth_file = 'one-box.depth.npdata.npy'
color_file = 'one-box.color.npdata.npy'
# extrinsics_file = 'extrinsics.npy' 

# Load the data
try:
    intrinsics = np.load(intrinsics_file)
    depth_map = np.load(depth_file)
    color_image = np.load(color_file)
    # extrinsics = np.load(extrinsics_file)
    print("Successfully loaded data.")
except FileNotFoundError as e:
    print(f"Error loading file: {e}. Make sure the files are in the correct directory.")
    exit()
except Exception as e:
    print(f"An error occurred while loading data: {e}")
    exit()

height, width = depth_map.shape

depth_o3d = o3d.geometry.Image(depth_map.astype(np.float32))

if len(color_image.shape) == 2:
    color_image_rgb = np.stack([color_image, color_image, color_image], axis=-1)
else:
    color_image_rgb = color_image[:, :, :3]

color_o3d = o3d.geometry.Image(color_image_rgb.astype(np.uint8))

fx = intrinsics[0, 0]
fy = intrinsics[1, 1]
cx = intrinsics[0, 2]
cy = intrinsics[1, 2]

pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(width, height, fx, fy, cx, cy)

depth_scale = 1000.0

rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
    color_o3d,
    depth_o3d,
    depth_scale=depth_scale,
    convert_rgb_to_intensity=False
)

point_cloud = o3d.geometry.PointCloud.create_from_rgbd_image(
    rgbd_image,
    pinhole_camera_intrinsic
)

# Geometric Segmentation: Plane Fitting and Filtering Above Plane
print("Segmenting points above the dominant plane...")

# Adjusting thresholds based on tuning
distance_threshold_plane = 0.005
ransac_n_plane = 3
num_iterations_plane = 1000
plane_model, inliers = point_cloud.segment_plane(distance_threshold=distance_threshold_plane,
                                                 ransac_n=ransac_n_plane,
                                                 num_iterations=num_iterations_plane)

if len(inliers) == 0:
    print("Could not find a dominant plane for segmentation. Cannot estimate pose.")
    segmented_point_cloud = o3d.geometry.PointCloud()
    cleaned_segmented_point_cloud = o3d.geometry.PointCloud()
else:
    [a, b, c, d] = plane_model
    plane_normal = np.array([a, b, c])
    plane_normal = plane_normal / np.linalg.norm(plane_normal)

    points_array = np.asarray(point_cloud.points)
    distances = np.abs(a * points_array[:, 0] + b * points_array[:, 1] + c * points_array[:, 2] + d) / np.linalg.norm(plane_normal)

    # Adjust threshold based on tuning
    height_above_plane_threshold = 0.0003

    above_plane_mask = distances > height_above_plane_threshold

    segmented_points_array = points_array[above_plane_mask]
    segmented_colors_array = np.asarray(point_cloud.colors)[above_plane_mask]

    segmented_point_cloud = o3d.geometry.PointCloud()
    segmented_point_cloud.points = o3d.utility.Vector3dVector(segmented_points_array)
    if segmented_colors_array.size > 0:
         segmented_point_cloud.colors = o3d.utility.Vector3dVector(segmented_colors_array)


    print(f"Segmented point cloud has {len(segmented_point_cloud.points)} points.")


    # Outlier Removal
    print("Applying statistical outlier removal...")
    if len(segmented_point_cloud.points) > 0:
        cl, ind = segmented_point_cloud.remove_statistical_outlier(nb_neighbors=70,
                                                                    std_ratio=0.3)
        cleaned_segmented_point_cloud = cl
        print(f"Cleaned segmented point cloud has {len(cleaned_segmented_point_cloud.points)} points.")
    else:
        cleaned_segmented_point_cloud = segmented_point_cloud


# Pose Estimation
if len(cleaned_segmented_point_cloud.points) > 0:
    oriented_bounding_box = cleaned_segmented_point_cloud.get_oriented_bounding_box()

    rotation_matrix = oriented_bounding_box.R
    translation_vector = oriented_bounding_box.center

    object_to_camera_transform = np.identity(4)
    object_to_camera_transform[:3, :3] = rotation_matrix
    object_to_camera_transform[:3, 3] = translation_vector

    camera_to_object_transform = np.linalg.inv(object_to_camera_transform)

    estimated_pose_matrix = camera_to_object_transform

    print("Estimated Camera-to-Object Pose Matrix (4x4):\n", estimated_pose_matrix)

    oriented_bounding_box.color = (1, 0, 0) # Red bounding box

    # Final Visualization
    print("Visualizing original point cloud, cleaned segmented (Blue), and bounding box (Red).")

    # Paint cleaned_segmented_point_cloud blue for final visualization
    cleaned_segmented_point_cloud_viz = o3d.geometry.PointCloud()
    cleaned_segmented_point_cloud_viz.points = cleaned_segmented_point_cloud.points
    if cleaned_segmented_point_cloud.has_colors():
         cleaned_segmented_point_cloud_viz.colors = cleaned_segmented_point_cloud.colors
    cleaned_segmented_point_cloud_viz.paint_uniform_color([0, 0, 1]) # Blue

    # Include the original point_cloud in the visualization list
    o3d.visualization.draw_geometries([point_cloud, cleaned_segmented_point_cloud_viz, oriented_bounding_box])

else:
    print("No points remaining after segmentation and cleaning. Cannot estimate pose.")

print("Script finished.")
[C-matrix]: ./images/C-matrix.png
[C-matrix_2]: ./images/C-matrix_2.png
[C-matrix_3]: ./images/C-matrix_3.png
[ObjectRecognition]: ./images/ObjectRecognition.png
[ObjectRecognition_2]: ./images/ObjectRecognition_2.png
[ObjectRecognition_3]: ./images/ObjectRecognition_3.png
[extracted_inliers]: ./images/extracted_inliers.png
[extracted_outliers]: ./images/extracted_outliers.png
[exercise_2_pcl_cluster]: ./images/exercise_2_pcl_cluster.png
[exercise_2_point_cloud]: ./images/exercise_2_point_cloud.png
[exercise_2_point_object]: ./images/exercise_2_point_object.png
[exercise_2_point_table_new]: ./images/exercise_2_point_table_new.png
[exercise_3_object_taging]: ./images/exercise_3_object_taging.png
[exercise_3_tarin_confusion_matrix]: ./images/exercise_3_tarin_confusion_matrix.png

## Project: Perception Pick & Place
# Required Steps for a Passing Submission:
1. Extract features and train an SVM model on new objects (see `pick_list_*.yaml` in `/pr2_robot/config/` for the list of models you'll be trying to identify). 
2. Write a ROS node and subscribe to `/pr2/world/points` topic. This topic contains noisy point cloud data that you must work with.
3. Use filtering and RANSAC plane fitting to isolate the objects of interest from the rest of the scene.
4. Apply Euclidean clustering to create separate clusters for individual items.
5. Perform object recognition on these objects and assign them labels (markers in RViz).
6. Calculate the centroid (average in x, y and z) of the set of points belonging to that each object.
7. Create ROS messages containing the details of each object (name, pick_pose, etc.) and write these messages out to `.yaml` files, one for each of the 3 scenarios (`test1-3.world` in `/pr2_robot/worlds/`).  [See the example `output.yaml` for details on what the output should look like.](https://github.com/udacity/RoboND-Perception-Project/blob/master/pr2_robot/config/output.yaml)  
8. Submit a link to your GitHub repo for the project or the Python code for your perception pipeline and your output `.yaml` files (3 `.yaml` files, one for each test world).  You must have correctly identified 100% of objects from `pick_list_1.yaml` for `test1.world`, 80% of items from `pick_list_2.yaml` for `test2.world` and 75% of items from `pick_list_3.yaml` in `test3.world`.
9. Congratulations!  Your Done!

# Extra Challenges: Complete the Pick & Place
7. To create a collision map, publish a point cloud to the `/pr2/3d_map/points` topic and make sure you change the `point_cloud_topic` to `/pr2/3d_map/points` in `sensors.yaml` in the `/pr2_robot/config/` directory. This topic is read by Moveit!, which uses this point cloud input to generate a collision map, allowing the robot to plan its trajectory.  Keep in mind that later when you go to pick up an object, you must first remove it from this point cloud so it is removed from the collision map!
8. Rotate the robot to generate collision map of table sides. This can be accomplished by publishing joint angle value(in radians) to `/pr2/world_joint_controller/command`
9. Rotate the robot back to its original state.
10. Create a ROS Client for the “pick_place_routine” rosservice.  In the required steps above, you already created the messages you need to use this service. Checkout the [PickPlace.srv](https://github.com/udacity/RoboND-Perception-Project/tree/master/pr2_robot/srv) file to find out what arguments you must pass to this service.
11. If everything was done correctly, when you pass the appropriate messages to the `pick_place_routine` service, the selected arm will perform pick and place operation and display trajectory in the RViz window
12. Place all the objects from your pick list in their respective dropoff box and you have completed the challenge!
13. Looking for a bigger challenge?  Load up the `challenge.world` scenario and see if you can get your perception pipeline working there!

### Exercise 1, 2 and 3 pipeline implemented
#### 1. Complete Exercise 1 steps. Pipeline for filtering and RANSAC plane fitting implemented. [Exercise 1 Code](https://github.com/gauravsaxena1983/RoboND-Perception-Exercises/tree/master/Exercise-1)

First we import the pcl library.
```
# Import PCL module
import pcl
```

Next we load the pcd file tabletop.pcd using pcl.load_XYZRGB function
```
# Load Point Cloud file
cloud = pcl.load_XYZRGB('tabletop.pcd')
```

Next we do statistical outlier filter to remove the noice.
```
# statistical_outlier_filter
statistical_outlier_filter = cloud.make_statistical_outlier_filter()
statistical_outlier_filter.set_mean_k(50)
statistical_outlier_filter.set_std_dev_mul_thresh(1.0)

cloud = statistical_outlier_filter.filter()
```

Next we apply Voxel Grid filter.
```
# Voxel Grid filter

# Create a VoxelGrid filter object for our input point cloud
vox = cloud.make_voxel_grid_filter()

# Choose a voxel (also known as leaf) size
# Note: this (1) is a poor choice of leaf size   
# Experiment and find the appropriate size!
LEAF_SIZE =   0.01

# Set the voxel (or leaf) size  
vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)

# Call the filter function to obtain the resultant downsampled point cloud
cloud_filtered = vox.filter()
```

Save the point cloud data to file voxel_downsampled.pcd after applying the vox filter.
```
filename = 'voxel_downsampled.pcd'
pcl.save(cloud_filtered, filename)

```

Make a pass through filter on z and y axis to remove the table from the pcd.
```
# PassThrough filter
# Create a PassThrough filter object.
passthrough = cloud_filtered.make_passthrough_filter()

# Assign axis and range to the passthrough filter object.
filter_axis = 'z'
passthrough.set_filter_field_name(filter_axis)
axis_min = 0.6
axis_max = 1.1
passthrough.set_filter_limits(axis_min, axis_max)
cloud_filtered = passthrough.filter()

# Assign y axis and range to the passthrough filter object. Needed as i see front of the table in the pcd file
passthrough = cloud_filtered.make_passthrough_filter()
filter_axis = 'y'
passthrough.set_filter_field_name(filter_axis)
axis_min = -2
axis_max = -1.4
passthrough.set_filter_limits(axis_min, axis_max)
cloud_filtered = passthrough.filter()

```

Save the final pcd to file pass_through_filtered.pcd
```
# Finally use the filter function to obtain the resultant point cloud. 
filename = 'pass_through_filtered.pcd'
pcl.save(cloud_filtered, filename)
```

Now apply the RANSAC plane segmentation
```
# RANSAC plane segmentation
# Create the segmentation object
seg = cloud_filtered.make_segmenter()

# Set the model you wish to fit 
seg.set_model_type(pcl.SACMODEL_PLANE)
seg.set_method_type(pcl.SAC_RANSAC)

# Max distance for a point to be considered fitting the model
# Experiment with different values for max_distance 
# for segmenting the table
max_distance = 0.01
seg.set_distance_threshold(max_distance)

# Call the segment function to obtain set of inlier indices and model coefficients
inliers, coefficients = seg.segment()
```


Extract the inliners and save the pcd in file extracted_inliers.pcd.
![extracted_inliers][extracted_inliers]
```
# Extract inliers
extracted_inliers = cloud_filtered.extract(inliers, negative=False)
filename = 'extracted_inliers.pcd'

# Save pcd for table
pcl.save(extracted_inliers, filename)
```


Extract the outliners and save the pcd in file extracted_outliers.pcd.
![extracted_outliers][extracted_outliers]
```
# Extract outliers
extracted_outliers = cloud_filtered.extract(inliers, negative=True)
filename = 'extracted_outliers.pcd'

# Save pcd for tabletop objects
pcl.save(extracted_outliers, filename)
```
#### 2. Complete Exercise 2 steps: Pipeline including clustering for segmentation implemented.  [Exercise 2 Code](https://github.com/gauravsaxena1983/RoboND-Perception-Exercises/tree/master/Exercise-2)
First import pcl helper library
```
#!/usr/bin/env python
# Import modules
from pcl_helper import *
```

Declare the callback function. 
```
def pcl_callback(pcl_msg):
```

Converting ros message to pcl using mehtods defined in pcl_helper.py file.
![exercise_2_point_cloud][exercise_2_point_cloud]
```
    cloud = ros_to_pcl(pcl_msg)
```

Applying  Voxel Grid Downsampling.
```
    vox = cloud.make_voxel_grid_filter()
    LEAF_SIZE = 0.01   
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    cloud_filtered = vox.filter()
```

Applying PassThrough Filter. 
```
    passthrough = cloud_filtered.make_passthrough_filter()
    filter_axis = 'z'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = 0.6
    axis_max = 1.1
    passthrough.set_filter_limits(axis_min, axis_max)
    cloud_filtered = passthrough.filter()
```

Applying RANSAC Plane Segmentation.
```
    seg = cloud_filtered.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    max_distance = 0.01
    seg.set_distance_threshold(max_distance)
    inliers, coefficients = seg.segment()
```

Extracting inliers and outliers
![exercise_2_point_object][exercise_2_point_object]
![exercise_2_point_table_new][exercise_2_point_table_new]
```
    cloud_objects =  cloud_filtered.extract(inliers, negative=True)
    cloud_table = cloud_filtered.extract(inliers, negative=False)
```

Applying Euclidean Clustering.

```
    white_cloud = XYZRGB_to_XYZ(cloud_objects)
    tree = white_cloud.make_kdtree()
    ec = white_cloud.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(0.015)
    ec.set_MinClusterSize(200)
    ec.set_MaxClusterSize(2500)
    ec.set_SearchMethod(tree)
    cluster_indices = ec.Extract()
```

Creating seprate random colours for each cluster.
```
    cluster_color = get_color_list(len(cluster_indices))
```

Creating colour cluster cloud from white colud using above colour list. 
![exercise_2_pcl_cluster][exercise_2_pcl_cluster]
```
    color_cluster_point_list = []

    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                            white_cloud[indice][1],
                                            white_cloud[indice][2],
                                            rgb_to_float(cluster_color[j])])
```

Create new cloud containing all clusters, each with unique color.
```
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)
```

Convert PCL data to ROS messages.
```
    ros_cloud_objects = pcl_to_ros(cloud_objects)
    ros_cloud_table = pcl_to_ros(cloud_table)
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)
```


Publish ROS messages
```
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)
    pcl_cluster_pub.publish(ros_cluster_cloud)
```

The main input
```
if __name__ == '__main__':
```

ROS node initialization
```
    rospy.init_node('clustering', anonymous=True)
```

Creating Subscribers
```
    pcl_sub = rospy.Subscriber("/sensor_stick/point_cloud", pc2.PointCloud2, pcl_callback, queue_size=1)
```    

Creating Publishers
```
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
    pcl_cluster_pub = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size=1)
```

Initializing color_list
```
    get_color_list.color_list = []
```

Spining while node is not shutdown
```
    while not rospy.is_shutdown():
        rospy.spin()
```
#### 3. Complete Exercise 3 Steps.  Features extracted and SVM trained.  Object recognition implemented. [Exercise 3 Code](https://github.com/gauravsaxena1983/RoboND-Perception-Exercises/tree/master/Exercise-3)

In exercise 3 we follow all the steps present in Exercise 1 and 2 to find the features from the models and train the data. 
![exercise_3_tarin_confusion_matrix][exercise_3_tarin_confusion_matrix]

Later we added the code to predict the object and label them and publish it back into the perception pipeline. 
![exercise_3_object_taging][exercise_3_object_taging]
```
# Exercise-3: 

    # Classify the clusters! (loop through each detected cluster one at a time)
    detected_objects_labels = []
    detected_objects = []

    for index, pts_list in enumerate(cluster_indices):
        # Grab the points for the cluster from the extracted outliers (cloud_objects)
        pcl_cluster = cloud_objects.extract(pts_list)
        # TODO: convert the cluster from pcl to ROS using helper function
        ros_cluster = pcl_to_ros(pcl_cluster)

        # Extract histogram features
        # TODO: complete this step just as is covered in capture_features.py
        chists = compute_color_histograms(ros_cluster, using_hsv=True)
        normals = get_normals(ros_cluster)
        nhists = compute_normal_histograms(normals)
        feature = np.concatenate((chists, nhists))
        #labeled_features.append([feature, model_name])

        # Make the prediction, retrieve the label for the result
        # and add it to detected_objects_labels list
        prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .4
        object_markers_pub.publish(make_label(label,label_pos, index))

        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cluster
        detected_objects.append(do)
```

### Pick and Place Setup

#### 1. For all three tabletop setups (`test*.world`), perform object recognition, then read in respective pick list (`pick_list_*.yaml`). Next construct the messages that would comprise a valid `PickPlace` request output them to `.yaml` format.

And here's another image! 
![demo-2](https://user-images.githubusercontent.com/20687560/28748286-9f65680e-7468-11e7-83dc-f1a32380b89c.png)

Spend some time at the end to discuss your code, what techniques you used, what worked and why, where the implementation might fail and how you might improve it if you were going to pursue this project further.  




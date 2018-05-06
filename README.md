# Object-detection-in-Point-Cloud-road-boundary

Object detection in Point Cloud is popular in HD Map and sensor-based autonomous driving. There  basically four types of object you can obtain in daily scenario: road surface - contains painted lane  marking and pavement area, support facility - contains road boundary (guardrail and curb), road  sign, light pole, etc., uncorrelated object - for example, sidewalk, building, etc., and moving object -  such like pedestrian, vehicle, bicycle, etc.  

In this project, please search references, design and prototype your road  boundary (guardrail) detection algorithm.


## Run Instructions

1. Open a Terminal at the project directory, run:
```
$ python3 CoordTransform.py
```
This python script will read original point cloud data, do coordinate transformation (LLA -> ECEF -> ENU -> Camera), 
and generate a `point_cloud_camera_coord.csv` file, which has 4 columns (X-coord, Y-coord, Z-coord, Intensity) for each data item.

2. Build the project "cloud_viewer" with the `CMakeLists.txt`

3. Run the project solution



### Notes:
1. Make sure the file `final_project_point_cloud.fuse` is in the directory `./final_project_data/`

2. Make sure the `point_cloud_camera_coord.csv` is under the correct directory

3. If you already have `point_cloud_camera_coord.pcd` file, you can commemt the line 335 to line 339

## Required Environment
  Python 3.6
  C++
  PCL 1.8.1 for C++
  opencv for C++

## Notes

1. It is recommended that you use cmake to build the project

2. We only test the project under windows system



## Project Files
```
  CoordTransform.py
  point_cloud_camera_coord.csv
  cloud_viewer.cpp
  CMakeLists.txt
```


## Reference
1. http://www.jeffdelmerico.com/wp-content/uploads/2014/03/pcl_tutorial.pdf
2. http://pointclouds.org/



















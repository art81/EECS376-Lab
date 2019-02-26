# fetch_vision
package to find position and orientation of parts on tables using Fetch robot head camera.
Several assumptions: torso fully lifted, head tilted 1.0 rad, table height = 0.7m
Does box filtering to find points above the table.  These points are converted to white pixels in
a corresponding 2D image, with fixed resolution (currently 5mm/pixel).

2D image is processed by OpenCV to find region labelling (connected regions).
Subsequently, need to find centroid and orientation of segmented objects. (not done yet)

## Example usage
`roscore`
`rosrun fetch_vision fetch_vision`
 then respond to prompe w/ PCD file name, e.g. three_cylinders.pcd
Results will be displayed in OpenCV windows.
Can also view point-cloud filtering results in rviz, using:
   frame head_camera_rgb_optical_frame,  topic pcd and topic /box_filtered_pcd



## Running tests/demos
    

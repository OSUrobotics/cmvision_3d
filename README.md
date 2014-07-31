3d_cmvision
==================

This is a package using the /blobs topic produced by cmvision, in addition to 3D data from a depth image to produce real world color tracking in the tf tree. Basically, we are combining color tracking with localization (or odometry). This way you can keep long term track of where your colors are.

The package itself uses a Bayesian filter to update the existence of color locations given new depth/size/etc information. 

## Usage:
You must be using a camera with a camera_info topic. This allows us to transform from pixels to 3d rays. You additionally need some depth image. These
correspond to the camera_topic and depth_image parameters for color_controller, respectively. Finally, you set parent_frame to the frame you want your colors to have as a parent. For example, if I'm just working on a robot I may set this to just /odom. Working woth AMCL localization, I'd set this to /map. 
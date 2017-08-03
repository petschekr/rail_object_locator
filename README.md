# RAIL Object Locator

## Overview

Combines the abilities of `rail_object_detector` and `handle_detector` to locate objects in the current scene as seen by the Kinect sensor and apply the correct translations and orientations to them.

With [dependencies](#dependencies) installed simply run `roslaunch object_location object_location_engine.launch` to launch the locator and its dependent nodes

The locator can be started by calling the `/location_query` service which will process and return a standard header and a list of detected objects with associated label, detection probability, pose, and cropped point cloud.

**Note:** The service call can take upwards of a minute to return. Performace is mostly bound by the `YOLO` object detection library used by `rail_object_detector` and can be sped up by enabling CUDA.

If the `publish_as_tf` option is set to `true` (default) the following will be continuously published:
- Detected objects under `/object_location`
- Object cropped point clouds under `/object_location_cloud/<label>`
- Transforms relative to the global frame (default: `map`) by object label

***

![Positioning and cropped point clouds](http://i.imgur.com/ypbCWpW.png)
> Bottle detected, labeled, and located by this package as shown in RViz. The grayscale point cloud is the feed coming directly from the Kinect sensor while the false color area is the processed and cropped point cloud containing just the recognized object.

![Orientation](http://i.imgur.com/tGMaoTR.png)
> Combining `rail_object_detector` and `handle_detector`'s capabilities, this package finds the objects in the scene and uses handle data to apply the proper orientations to them. Here, `rail_object_detector` found the bottle and monkey wrench (labeled as toothbrush) while `handle_detector` found handle orientations for all three of the wrenches. The results were combined to form this output with the z-axis of each transform parallel to the axis of the handle (if one exists)

![Orientation 2](http://i.imgur.com/kg2kLuV.png)
> A closer look at the calculated transforms of the monkey wrenches in the scene.

## Dependencies

* ROS Indigo
* [`rail_object_detector`](https://github.com/GT-RAIL/rail_object_detector)
* [`handle_detector`](https://github.com/petschekr/handle_detector) (`petschekr` fork)

## License

Copyright &copy; 2017 Ryan Petschek. Released under the MIT license. See [LICENSE](LICENSE.md) for more information.
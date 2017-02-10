# A FAST SCREEN-CAMERA CALIBRATION METHOD USING A RGB-D SENSOR

##### Authors
Nicola Piccinelli, Marco Ruzzenente, Giacomo De Rossi

## Abstract
Screen-camera calibration means localizing the screen position in the camera’s cartesian space. This allows to develop more endearing interactions with a user, such as holographic display applications. This article presents a novel approach to the screen-camera calibration which does not require any change in the positioning of either the screen or the sensor, and it is both fast and practical for the user to execute. Thanks to a RGB-D sensor and both a skeleton and face tracking system, it is possible to use the hand to point to each corner of the screen from different angles; the lines connecting the pointing eye and fingertip will have their fulcrum roughly on the corners of the screen. By using a mean squared error reduction, it is possible to determine the intersection point in the 3D space for each corner. Such approach is both fast and inexpensive.

## Requirements
* Unity 3D (5.3.3)
* Kinect 2 
* Kinect 2 SDK
* Microsoft Windows

## Usage
1. Clone the repository :)
2. Open project in Unity (the build is not required) and open Scene/Calibration
3. Setup the calibration parameters
4. Run the application and follow the instructions
5. The results of the calibration is placed in Assets/config.ini 

## Calibration Parameters
In *KinectCalibrator* game object you must define six parameters in order to achive a good results in your screen-camera calibration.

1. **Screen Height (in meters)**: for best results use the factory dimensions
2. **Screen Width (in meters)**: for best results use the factory dimensions
3. **Lines per corner**: the amount of lines requested for each screen corner (total samples: lines_per_corner * 4)
4. **Ransac Threshold**: the maximum distance between computed intersection point and inlier lines
5. **Ransac Probability**: default 0.95
6. **Upper Left Corner Position Guess**: the position of upper left corner w.r.t kinect depth frame

## Result file Nomenclature
The calibration results is subdivided as follow:

* **CORNERS_POSITION_WITHOUT_RANSAC**: screen corners position estimated with SSD (Sum of Square Differences) without outliers rejection
* **OPTIMIZE_CORNERS_POSITION_WITHOUT_RANSAC (User Guess)**: screen corners position estimated after Levenberg–Marquardt algorithm initialized with user UpperLeft Corner guess. The set of corner lines should contains outliers
* **OPTIMIZE_CORNERS_POSITION_WITHOUT_RANSAC (Estimated Guess)**: screen corners position estimated after Levenberg–Marquardt algorithm initialized with automatic best guess estimation. The set of corner lines should contains outliers
* **CORNERS_POSITION_RANSAC**: screen corners position estimated with RANSAC algorithm with outliers rejection
* **OPTIMIZE_CORNERS_POSITION_RANSAC (User Guess)**: screen corners position estimated after Levenberg–Marquardt algorithm initialized with user UpperLeft Corner guess. The set of corner lines doesn't contain outliers
* **OPTIMIZE_CORNERS_POSITION_RANSAC (Estimated Guess)**: screen corners position estimated after Levenberg–Marquardt algorithm initialized with automatic best guess estimation. The set of corner lines doesn't contain outliers

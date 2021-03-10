# Self-Driving Car Beta Testing Nanodegree 

## Sensor Fusion and Object detection

We used the Waymo Open Dataset's real-world data and applied an extended Kalman fusion filter to 
map and track several vehicles in this project. The major tasks accomplished to complete the project: 
1. Implement a Kalman filter to track an object 
2. Track management, initialize, update and delete tracks to manage to track 
3. Data association
4. Camera sensor fusion, based on lidar fusion, add camera measurement fusion 

To run the project, simply run the script loop_over_dataset.py


## Step-1: Extended Kalman Filter

In the filter.py file, EKF is used.

* Design system sate [x, y, z, vx, vy, vz], process model, and constant velocity model.
* We have to calculate the systemmatrix for the 3D process models with constant velocity and noise covariances based on the current time phase. Although the time phase could vary in general.
* For current state calculation, h(x), and the Jacobian H function are evaluated. 

The following graph was plotted.

![step1](https://user-images.githubusercontent.com/67744017/110574471-5ade0580-8187-11eb-992f-2eecf2721d95.png)
Fig: Single target tracking results

## Step-2: Track Management

In trackmanagement.py, track management is carried out.

One object is shown as a track in our device architecture, and tracklist is used to handle several objects. 
We transfer the track item and measurement information to the EKF to anticipate and update the 
worker on a track.

The key track control points are below:

* Following unassigned lidar calculation, the track would be initialized.

* There are scores on a track. If it is correlated with measurement, the score would be increased and otherwise lowered.

* A track includes the condition that is changed according to the track ranking.

* If the score is below one certain three-point and the state balance is greater than a threshold, a track is 
eliminated.

The following graph is plotted.

![step2](https://user-images.githubusercontent.com/67744017/110575141-9cbb7b80-8188-11eb-9a6d-de500eefa671.png)
Fig: Single target tracking results

## Step-3: Data Association

In this step, the closest neighbor association correctly matches several measurements to several tracks. In association.py, data association is introduced.

Here are the important steps in the association of results,

* Build a matrix with all tracks and overviews open.
* For each measurement pair, calculate the distance of Mahalanobis (MHD).
* To exclude unlikely track pairs, use the hypothesis test Chi-Square.
* Choose the pair with the smallest MHD, update ekf, and delete the relation matrix with the appropriate row and column.
* Return to phase 4 before all applicable pairs are processed.

The following graph is plotted.

![step3](https://user-images.githubusercontent.com/67744017/110577108-2456b980-818c-11eb-8fa5-1ac12046721a.png)
Fig: Tracking result with three tracks

## Step-4: Camera Sensor fusion

In this phase, we add to ekf fusion cameras. The main assumption is the center of the 3d space bounding box for a car which is following the center of the 2d imagery of the vehicle. This assertion is approximately correct, however, for a front camera does not always be accurate.

Implementing the fusion measuring camera consists of a projection matrix, which converts the points from 3d in space into 2d in the picture, in the main aspects below.

The partial device status derivative(x,y,z) for measuring model (u,v) measuring noise R for measuring camera is Jacobian.

To reject the wrong measurement-track pair, check whether the tracking status is in the camera field of view.

A video of “my_tracking_results.avi” for the tracking of vehicles using lidar and camera measurements is recorded inside the project directory. Uploaded the file in the video folder.

Tracking result with three tracks

![step3_graph](https://user-images.githubusercontent.com/67744017/110577461-e312d980-818c-11eb-961e-5f9f51ebb8f0.png)
Fig: RMSE for the three valid tracks

## Most Difficult Part

The implementation of ekf, track management, data association, and camera-lidar fusion are all well guided in the lectures, so I believe that completing this project is straightforward.
In my opinion, it was difficult to implement the camera measuring model. When projecting a 3d point into a 2d point, there are transformations in the camera axis. However, the coding of the project was discovered and the problem was solved.

For the project, a pre-computed result is needed. However, the pre-computed result files do not correspond to the load filename of the loop_over_dataset.py file. For using the files, I modified the filenames according to the pre-computed result.

![image](https://user-images.githubusercontent.com/67744017/110577644-43a21680-818d-11eb-9287-05bfd6807ebc.png)
Fig: modified loop_over_dataset for pre-computed result

## Benefits in Camera-Lidar Fusion tracking over Lidar-only tracking

Different sensors for object detection have their advantages and disadvantages. For more precise and stable tracking, sensor fusion combines multiple sensors. Cameras, for example, may offer rich textured and color-based information in object detection that LiDAR usually does not provide.

On the other side, LiDAR can operate at low visibility, for example at night, or moderate rain or fog.

Besides, the LiDAR can offer a far more precise spatial coordinate estimate than a camera for the identification of target orientation relative to the sensor.

The optimal algorithm should thoroughly take advantage of their benefits since both the camera and LiDAR have their inconveniences and drawbacks in their fusion. This could result in more precise tracking results than relying solely on LiDAR-only tracking.

By including camera fusion tracking, we can produce a more precise geometric scene understanding and cover a broader range and thus detect more objects. 

## Real-life challenges:

A sensor-fusion systemcould be confronted with a variety of real-world issues. This project has its own set of challenges.

Multiple tracks and measurements provide a precise correlation. Setting good gating thresholds for both scenarios (rejecting wrong measurement and track pair) can be problematic at times.

The measurement noise configuration is insufficient to provide a precise project result. In reality, rather than setting a standardized noise variance for a sensor, it's best if each measurement has its noise variance.

This project eliminates the issue of extrinsic parameter tuning, which is one method for camera and 
LiDAR fusion. These extrinsic parameters are defined since we are using a public dataset for this 
experiment.

## Improvement opportunity:

As already stated, the project should carry out the Camera-LiDAR Fusion Monitoring. And also, A 3D measuring model of the real camera sound can assist with the fusion effect, we can fit actual 3d points in the lidar point cloud to the vehicle target pixels.

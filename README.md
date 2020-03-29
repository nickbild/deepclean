# Deep Clean

Deep Clean watches a room and flags all surfaces as they are touched for special attention on the next cleaning to help prevent the spread of contagious disease.

One use case would be in a hospital room.  When a patient checks out, the cleaning service could view all surfaces that need special attention.

<p align="center">
<img src="https://raw.githubusercontent.com/nickbild/deepclean/master/media/teaser.gif">
</p>

## How it Works

Deep Clean uses a stereo camera to detect the depth (z-coordinate) of an object of interest (e.g. a hand) in the video frame.  OpenPose is used to detect hand location (x,y-coordinates).  When a hand is at the same position and depth as another object in view (i.e. touching), that location is tracked.  An image can be generated showing all surfaces that have been touched ([script](https://github.com/nickbild/deepclean/blob/master/3dcam.py)).

### Camera

Camera options for the Xavier tend to be on the expensive side for a hobbyist, so I built my own from a pair of Raspberry Pi cameras.  To keep the cameras in the proper relative positions, I designed and 3D printed a [base](https://github.com/nickbild/deepclean/blob/master/3d_models/stero_cam_base.stl) and a [stabilizer](https://github.com/nickbild/deepclean/blob/master/3d_models/stereo_stabilizer.stl).

The Xavier does not have CSI connections for this type of camera, so I used a pair of Raspberry Pis to capture images ([script](https://github.com/nickbild/deepclean/blob/master/remote_cam.py)) when triggered via GPIO from the Xavier.  This is far from optimal, and shows itself in the low frame rate, but it works adequately on a budget.

The cameras were [calibrated](https://github.com/nickbild/deepclean/blob/master/camera_calibration/stereo_calibration.py) (individually, and in pair) in OpenCV using a printed chessboard pattern:

![chessboard](https://raw.githubusercontent.com/nickbild/deepclean/master/camera_calibration/img/good/pi1_3_good.jpg)

## Media

YouTube Video:  
https://www.youtube.com/watch?v=Qy8Ks7UTtrA

Full setup:  
![Full Setup](https://raw.githubusercontent.com/nickbild/deepclean/master/media/full_setup_sm.jpg)

Stereo Camera:  
![camera](https://raw.githubusercontent.com/nickbild/deepclean/master/media/cameras_sm.jpg)

## Bill of Materials

- 1 x NVIDIA Jetson AGX Xavier
- 2 x Raspberry Pi 3B+
- 2 x Raspberry Pi Camera v2
- 3D printer
- Miscellaneous wire

## Future Direction

With a higher budget, a more optimal version could be developed with an off the shelf stereo camera that can directly interface with the processing unit.  Using a custom-built stereo camera makes the system a little bit buggy at times, and capturing images on separate computers slows the frame rate substantially.

There is also room for adding additional detection techniques.  While I believe it's useful as is, it currently can't detect a cough or sneeze, for example, which can also contaminate surfaces.

## About the Author

[Nick A. Bild, MS](https://nickbild79.firebaseapp.com/#!/)

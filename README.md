# KCF_ROS

Wraps the KCF tracker into a ROS package and publishes the tracked region as a custom message.

This tracking method is an implementation of [1] which is extended to KFC with color-names features [2].
The implementation is based on OpenCVs abstract class cv::Tracker.

The package tracker provides a Service node that starts the tracking process upon receiving an initialization request. 
The image frame is received from the usb_cam package.


# Dependencies

1. ROS
2. OpenCV 3.0
3. Cmake

### How to run

[1] J. F. Henriques, R. Caseiro, P. Martins, and J. Batista. Exploiting the circulant structure of tracking-by-detection with kernels. In proceedings of the European Conference on Computer Vision, 2012.

[2] M. Danelljan, F.S. Khan, M. Felsberg, and J. van de Weijer. Adaptive color attributes for real-time visual tracking. In Computer Vision and Pattern Recognition (CVPR), 2014 IEEE Conference on, pages 1090–1097, June 2014.







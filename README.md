## mobile_3D_Reconstruction
## Title: Sparse 3D reconstruction of a textured object on an iOS device

**Team:** Aishanou Rait, Astha Prasad

**Abstract:** The goal of this project was to implement the entire Bundle Adjustment pipeline for obtaining the N-view 3D reconstruction of an object with sufficient features. The user is asked to take a video of the object of interest using an iOS device, which is then processed to generate the 3D reconstructed points. .

**Introduction:**Since 3D reconstruction requires computationally heavy non-linear optimization, the feasibility of realistically performing this pipeline on a mobile device is rather untested. Current mobile solutions lean on mobile devices for the sole purpose of capturing object images with ease. The collected images of the object are then uploaded to the cloud where the 3D reconstruction is performed, thus taking the mobile’s processing capabilities out of the equation. This project outlines our attempts to perform the 3D reconstruction pipeline on a mobile device with the help of available open source non-linear optimizers such as the Ceres solver for iOS devices. 

**Background:** Some of the popular open source Structure for Motion packages available are:
Bundler (Windows/Linux)
VisualSFM (Windows/Linux/Mac)
SFMedu (MATLAB)

While all the above offer solutions for Bundle Adjustment, none of the above are developed for iOS devices and the speed ups used by them cannot be easily ported onto a mobile device. Thus we decided to implement our own basic pipeline using openCV’s functions on an iOS device and try to optimize at an algorithmic level. 

**Results:** The details of the results are shown in the video below. For more details please refer to the report in the repository.
[![Alt text for your video](http://img.youtube.com/vi/T-D1KVIuvjA/0.jpg)](http://www.youtube.com/watch?v=T-D1KVIuvjA)





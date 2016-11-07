## mobile_3D_Reconstruction
## Title: Sparse 3D reconstruction of a textured object on an iOS device

**Team:** Aishanou Rait, Astha Prasad

**Summary:** Implement the entire Bundle Adjustment pipeline for obtaining the N-view 3D reconstruction of an object with sufficient features. The user will take video of the object using an iOS device and after processing 3D reconstruction of the object would be displayed.

**Background:** The Bundle Adjustment part would be implemented in MATLAB or C++ for another class project to understand the nitty-gritty's of the various steps involved in the pipeline. This implementation will not focus on real-time performance. However, it is always better to obtain 3D reconstruction of an object in real time using just a mobile device. For Example, if a 3D reconstruction of a room is available then one can project models of furniture to see how it fits. Thus through this project we aim to make the reconstruction work on a mobile device with a stretch goal of making it real time. 

**The Challenge:** A naive implementation of Bundle Adjustment will not run fast on an embedded system platform. The challenge is to figure out what optimizations can be performed for the system to run in real time: such as using the Ceres solver or the G2O. The compatibility of these libraries and the performance enhancement has to be discovered.

The aim of the project is to learn the performance bottlenecks of a basic implementation of Bundle Adjustment and what can be done to remove them. The following are the major steps which will be implemented:

<img src="https://github.com/aorait/mobile_3D_Reconstruction/blob/master/images/Flowchart.png" width="500">

**Goals and Deliverables:**

PLAN TO ACHIEVE:Sparse 3D reconstruction of an object  using a video recorded on a mobile device. 

HOPE TO ACHIEVE:Real time sparse reconstruction

The success of the project will be validated by providing a live recording of the device screen exhibiting a user capturing a video of a 3D textured object followed by 3D sparse reconstruction of the object.  

Realistically, we plan successfully execute sparse 3D reconstruction within the allotted time. Achieving semi-dense reconstruction in real time would be our stretch goal. 

**Schedule:**

<img src="https://github.com/aorait/mobile_3D_Reconstruction/blob/master/images/Schedule.PNG" width="600">





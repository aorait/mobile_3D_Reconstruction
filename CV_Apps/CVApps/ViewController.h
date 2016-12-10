//
//  ViewController.h
//  CVApps
//
//  Created by Aishanou Osha Rait on 11/30/16.
//  Copyright © 2016 Aishanou Osha Rait. All rights reserved.
//


////////////Trial2.cpp

#include "Trial2.hpp"
#include <iostream>
#include <vector>
#include <stdlib.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
//#include <ceres.h>

// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl/io/openni_grabber.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl_conversions/pcl_conversions.h>



#import <UIKit/UIKit.h>
#import <MobileCoreServices/UTCoreTypes.h>
#import <MediaPlayer/MediaPlayer.h>


using namespace cv;
using namespace std;



//#define DATA_LOC "/home/aishanou/Documents/GeometryProject/data/"
//#define MIN_KP_DIST (0.1)
////#define FOCAL_LEN_X (719.5459)
////#define FOCAL_LEN_Y (719.5459)
////#define FOCAL_LEN_X (3310.4)
////#define FOCAL_LEN_Y (3325.5)
//#define FOCAL_LEN_X (1502.4)
//#define FOCAL_LEN_Y (1529.5)
////cv::Point2d principle_point=cv::Point2d(302.32, 246.87);
////cv::Point2d principle_point=cv::Point2d(240, 320);
////double cx=316.73; double cy=200.55;
//double cx=302.32; double cy=246.87;
//cv::Point2d principle_point=cv::Point2d(cx, cy);




@interface ViewController : UIViewController <UIImagePickerControllerDelegate, UINavigationControllerDelegate>

- (IBAction)SelectVideo:(id)sender;
// For opening UIImagePickerController
-(BOOL)startMediaBrowserFromViewController:(UIViewController*)controller usingDelegate:(id )delegate;
@property(nonatomic) AVURLAsset *movie;

@end


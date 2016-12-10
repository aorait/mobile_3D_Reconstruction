//
//  GetFramesViewController.h
//  CVApps
//
//  Created by Aishanou Rait on 12/7/16.
//  Copyright © 2016 Aishanou Osha Rait. All rights reserved.
//



#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include "armadillo"


#import "opencv2/imgcodecs/ios.h"
#import <opencv2/core/core.hpp>
#import <opencv2/features2d/features2d.hpp>

#import <UIKit/UIKit.h>
#import <MediaPlayer/MediaPlayer.h>
#import <MobileCoreServices/UTCoreTypes.h>
#import <AssetsLibrary/AssetsLibrary.h>
#import <AVFoundation/AVFoundation.h>

#define DATA_LOC "/home/aishanou/Documents/GeometryProject/data/"
#define MIN_KP_DIST (0.1)
//#define FOCAL_LEN_X (719.5459)
//#define FOCAL_LEN_Y (719.5459)
//#define FOCAL_LEN_X (3310.4)
//#define FOCAL_LEN_Y (3325.5)
#define FOCAL_LEN_X (1502.4)
#define FOCAL_LEN_Y (1529.5)

@interface GetFramesViewController : UIViewController

@property(nonatomic) AVURLAsset *MyMovie;
@property (strong) AVAssetImageGenerator *imageGenerator;
@property (strong, nonatomic) NSMutableArray *Frames;
@property (strong, nonatomic) UIActivityIndicatorView *spinner;


@end

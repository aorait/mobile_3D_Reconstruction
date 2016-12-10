//
//  GetFramesViewController.m
//  CVApps
//
//  Created by Aishanou Rait on 12/7/16.
//  Copyright © 2016 Aishanou Osha Rait. All rights reserved.
//

#import "GetFramesViewController.h"
#include <iostream>
#include <stdio.h>
#include <fstream>
#include <stdlib.h> // Include the standard library
#include <ctime>



using namespace std;
using namespace cv;

@interface GetFramesViewController (){
    UIImageView *imageView00_;
    UIImageView *imageView01_;
    UIImageView *imageView10_;
    UIImageView *imageView11_;
//    UIImageView *imageViewtest_;
    
}

@end

@implementation GetFramesViewController

//-(void)mymethods:(NSString *)aCont withsecond:(NSString *)a-second { }
//[mymethod:self.contoCorrente withsecond:self.asecond];


void convertToHomo(cv::Mat &src, cv::Mat &dst){
    cv::Mat lastRow = src.row( 3 );
    cv::Mat tmp;
    cv::repeat(lastRow, 4, 1,tmp );
    dst=src/tmp;
}


void computeMatches(vector<cv::Mat> &images,vector<vector<cv::KeyPoint> > &keypoints, vector<cv::Mat> &descriptors,vector<vector<cv::DMatch>> &total_matches){
    cv::FlannBasedMatcher matcher;
    vector<cv::DMatch> matches;
    for(int i=0;i<descriptors.size()-1;++i){
        cv::Mat descp1,descp2;
        descriptors[i].copyTo(descp1);
        descriptors[i+1].copyTo(descp2);
        matcher.match(descp2, descp1, matches);
        double min_kp_dist = std::numeric_limits<double>::max();
        double max_kp_dist = std::numeric_limits<double>::min();
        for (int i = 0; i < matches.size(); ++i) {
            double cur_dist = matches[i].distance;
            if (min_kp_dist > cur_dist) {   min_kp_dist = cur_dist; }
            if (max_kp_dist < cur_dist) {   max_kp_dist = cur_dist; }
        }
        
        cout << "-- Min keypoint distance = " << min_kp_dist << std::endl;
        cout << "-- Max keypoint distance = " << max_kp_dist << std::endl;
        // Compute the "good" set of matches
        vector<cv::DMatch> good_matches;
        for (int i = 0; i < matches.size(); ++i) {
            if (matches[i].distance < std::max(0.01, 3 * min_kp_dist)) {
                good_matches.push_back(matches[i]);
            }
        }
        total_matches.push_back(good_matches);
    }
    cout<<"size of total_matches "<<total_matches.size()<<endl;
}

void showMatches(const vector<cv::Mat> &images, const vector<vector<cv::KeyPoint> > &keypoints,const vector<vector<cv::DMatch>> &total_matches){
    cout<<"size of total_matches "<<total_matches.size()<<endl;
    string imName;
    for(int i=0;i<total_matches.size();++i){
        cv::Mat matchedImage;
        cv::drawMatches(images[i+1], keypoints[i+1], images[i], keypoints[i], total_matches[i], matchedImage, cv::Scalar::all(-1),
                        cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
//        imName="matches"+to_string(i);
//        imshow(imName,matchedImage);
    }
    
}

void detectFeatures(vector<cv::Mat> &images, vector<vector<cv::KeyPoint> > &keypoints,vector<cv::Mat> &descriptors){
    int min_hessian = 400;
    cv::Ptr<cv::xfeatures2d::SURF> surf_detector = cv::xfeatures2d::SURF::create(min_hessian);
    vector<cv::KeyPoint> kp;
    cv::Mat descp;
    
    for(int i=0;i<images.size();++i){
        cv::Mat im;
        images[i].copyTo(im);
        surf_detector->detectAndCompute(im, cv::Mat(), kp, descp);
        keypoints.push_back(kp);
        descriptors.push_back(descp);
    }
    for(int i=0;i<images.size();++i){
        cout << "im" <<i<< " detected "<< keypoints[i].size() << " keypoints" << std::endl;
    }
    //computeMatches(images,keypoints,descriptors);
}

cv::Mat readIntrinsics(const string &strSettingPath){
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    double fx = fSettings["Camera.fx"];
    double fy = fSettings["Camera.fy"];
    double cx = fSettings["Camera.cx"];
    double cy = fSettings["Camera.cy"];
    
    cv::Mat K = cv::Mat::eye(3,3,CV_64F);
    K.at<double>(0,0) = fx;
    K.at<double>(1,1) = fy;
    K.at<double>(0,2) = cx;
    K.at<double>(1,2) = cy;
    return K;
}


void computePoses(vector<vector<cv::Point2f>> &pts2D,cv::Mat &K, vector<cv::Mat> &projMats){
    double focal_len=K.at<double>(0, 0);
    double cx=K.at<double>(0, 2);
    double cy=K.at<double>(1, 2);
    cv::Mat R1 = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat t1= cv::Mat::zeros(3,1, CV_64F);
    cv::Mat M1;
    cv::hconcat(R1,t1,M1);
    projMats.push_back(M1);
    cv::Point2d principle_point=cv::Point2d(cx, cy);
    for(int i=0;i<pts2D.size()-1;i=i+2){
        cv::Mat E = cv::findEssentialMat(pts2D[i], pts2D[i+1], focal_len, principle_point);
        cout << "Essential matrix: \n" << E << endl;
        cv::Mat R2,t2;
        int inliersE=recoverPose(E, pts2D[i], pts2D[i+1], R2, t2, focal_len,principle_point);
        //Form projection matrices
        cv::Mat M2;
        cv::hconcat(R2,t2,M2);
        cout<<"M1 "<<endl<<M1<<endl;
        cout<<"M2 "<<endl<<M2<<endl;
        projMats.push_back(M2);
        
    }
}

void convertKeypointstoCVpointsSingle(vector<vector<cv::KeyPoint>> &keypoints,vector<vector<cv::DMatch>> &common_matches,vector<vector<cv::Point2f>> &pts2D){
    for (int i = 0; i < common_matches.size(); ++i) {
        vector<cv::Point2f> pt_temp1,pt_temp2;
        for(int j=0;j<common_matches[i].size();++j){
            if(i==0){
                pt_temp1.push_back(keypoints[i][common_matches[i][j].trainIdx].pt);
            }
            pt_temp2.push_back(keypoints[i][common_matches[i][j].queryIdx].pt);
            //kp_loc_2.push_back(keypoints_2[good_matches[i].queryIdx].pt);
        }
        if(i==0){
            pts2D.push_back(pt_temp1);
        }
        pts2D.push_back(pt_temp2);
    }
}

// Quick function to convert to Armadillo matrix header
arma::fmat Cv2Arma(cv::Mat &cvX)
{
    arma::fmat X(cvX.ptr<float>(0), cvX.cols, cvX.rows, false); // This is the transpose of the OpenCV X_
    return X; // Return the new matrix (no new memory allocated)
}

void compute3DPoints(vector<vector<cv::Point2f>> &pts2D,cv::Mat &K, vector<cv::Mat> &projMats){
    cout<<"Entered compute3Dpoints"<<endl;
    //cout<<"K "<<K<<endl;
    cv::Mat emp;
    cv::Mat M1;
    projMats[0].copyTo(M1);
    vector<cv::Point2f> kp1;
    undistortPoints(pts2D[0], kp1, K,emp);
    for(int i=1;i<pts2D.size();++i){
        vector<cv::Point2f> kp2;
        undistortPoints(pts2D[i], kp2, K,emp);
        //cout<<"undistorted points1 "<<kp1<<endl;
        //cout<<"undistorted points2 "<<kp2<<endl;
        cv::Mat points4D,points4D_trans,points3D,points3D_trans,final3D;
        cv::Mat M2;
        projMats[i].copyTo(M2);
        triangulatePoints(M1,M2, kp1,kp2,points4D);
        cv::transpose(points4D,points4D_trans);
        //cout<<points4D_trans<<endl;
        convertToHomo(points4D,points3D);
        cv::Mat temp_trans;
        //cv::transpose(points3D,temp_trans);
        points3D(cv::Rect(0,0,points3D.cols,points3D.rows-1)).copyTo(temp_trans);
        // pcl::PointCloud<pcl::PointXYZ>::Ptr pclPtr= MatToPointXYZ(temp_trans);
        // string filename="test_pcd.pcd"+to_string(i);
        // pcl::io::savePCDFileASCII (filename, *pclPtr);
        cv::transpose(temp_trans,final3D);
        
        
        //WRITE TO FILE
        NSArray *paths = NSSearchPathForDirectoriesInDomains
        (NSDocumentDirectory, NSUserDomainMask, YES);
        NSString *documentsDirectory = [paths objectAtIndex:0];
        
        //make a file name to write the data to using the documents directory:
        NSString *fileName = [NSString stringWithFormat:@"%@/XYZ.txt",
                              documentsDirectory];
        
        // Get the full path and name of the file
        const char *fname = [fileName UTF8String];
        
        // Randomly initialize a variable and then save it
        arma::fmat final3Darma(final3D.rows, final3D.rows);
        final3Darma = Cv2Arma(final3D);
        final3Darma.save(fname, arma::raw_ascii);
        cout << "ARMA FINAL: " << final3Darma.n_rows << " x " << final3Darma.n_cols << endl;
        cout << "writing out XYZ.txt to the Documents directory!!!" << endl;
        //cout<<final3D<<endl;
    }
}

void appendOnes(cv::Mat &mat,cv::Mat &res){
    cv::Mat lastRow=cv::Mat::zeros(1,4, CV_64F);
    lastRow.at<double>(0,3) = 1;
    vconcat(mat,lastRow,res);
}

void transFormProjectionMats(vector<cv::Mat> &projMats,vector<cv::Mat> &transformedMats){
    vector<cv::Mat> tempMats;
    for(int i=0;i<projMats.size();++i){
        cv::Mat temp;
        appendOnes(projMats[i],temp);
        cout<<temp<<endl;
        tempMats.push_back(temp);
    }
    for(int i=2;i<tempMats.size();++i){
        cv::Mat mult=tempMats[i-1]*tempMats[i];
        mult.copyTo(tempMats[i]);
    }
    for(int i=0;i<tempMats.size();++i){
        cv::Mat trimmed;
        tempMats[i](cv::Rect(0,0,tempMats[i].cols,tempMats[i].rows-1)).copyTo(trimmed);
        transformedMats.push_back(trimmed);
    }
}


void convertKeypointstoCVpoints(vector<vector<cv::KeyPoint>> &keypoints,vector<vector<cv::DMatch>> &total_matches,vector<vector<cv::Point2f>> &pts2D){
    vector<cv::Point2f> pt_temp1,pt_temp2;
    for (int i = 0; i < total_matches.size(); ++i) {
        for(int j=0;j<total_matches[i].size();++j){
            pt_temp1.push_back(keypoints[i][total_matches[i][j].trainIdx].pt);
            pt_temp2.push_back(keypoints[i][total_matches[i][j].queryIdx].pt);
            //kp_loc_2.push_back(keypoints_2[good_matches[i].queryIdx].pt);
        }
        pts2D.push_back(pt_temp1);
        pts2D.push_back(pt_temp2);
    }
}

void computeCommonMatches(vector<cv::Mat> &images,vector<vector<cv::KeyPoint> > &keypoints,vector<cv::Mat> &descriptors,vector<vector<cv::DMatch>> &total_matches,vector<vector<cv::DMatch>> &common_matches){
    vector<cv::KeyPoint> keypoints_trimmed;
    for(int j=0;j<total_matches[0].size();++j){
        keypoints_trimmed.push_back(keypoints[0][total_matches[0][j].trainIdx]);
    }
    cout<<"keypoints_trimmed size "<<keypoints_trimmed.size()<<endl;
    cv::Mat descriptor_base;
    cv::Ptr<cv::DescriptorExtractor> extractor = cv::ORB::create();
    extractor->compute(images[0], keypoints_trimmed, descriptor_base);
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
    std::vector<cv::DMatch> matches;
    for(int i=1;i<descriptors.size();++i){
        cv::Mat descp2;
        extractor->compute(images[i], keypoints[i], descp2);
        matcher->match(descriptor_base,descp2,matches);
        cout<<"size of matches "<<matches.size()<<endl;
        common_matches.push_back(matches);
    }
}


- (void)viewDidLoad {
    [super viewDidLoad];
    
    // Do any additional setup after loading the view.
    
    //SEVERAL IMAGES
    AVAsset *myAsset = self.MyMovie;
    Float64 durationSeconds = CMTimeGetSeconds([myAsset duration]);
    CMTime firstThird = CMTimeMakeWithSeconds(durationSeconds/3.0, 600);
    CMTime secondThird = CMTimeMakeWithSeconds(durationSeconds*2.0/3.0, 600);
    CMTime end = CMTimeMakeWithSeconds(durationSeconds, 600);
    NSValue *t1 = [NSValue valueWithCMTime:kCMTimeZero];
    NSValue *t2 = [NSValue valueWithCMTime:firstThird];
    NSValue *t3 = [NSValue valueWithCMTime:secondThird];
    NSValue *t4 = [NSValue valueWithCMTime:end];
    NSArray *times = [NSArray arrayWithObjects: t1, t2, t3, t4, nil];
    
    self.Frames = [[NSMutableArray alloc] init];
    vector <cv::Mat> MatFrames;
    cv::Mat im;
    self.imageGenerator = [[AVAssetImageGenerator alloc] initWithAsset:myAsset];
    self.imageGenerator.appliesPreferredTrackTransform = YES;
    for (int i = 0; i < times.count; i++) {
        NSError *error = NULL;
        CMTime time = CMTimeMake([[times objectAtIndex:i] intValue], 1);
        CGImageRef refImg = [self.imageGenerator copyCGImageAtTime:time actualTime:NULL error:&error];
        [self.Frames addObject:[[UIImage alloc] initWithCGImage:refImg]];
        cout << "UIImage " << i << self.Frames[i] << endl;
        cout << "Converting Image to cv::Mat" << endl;
        UIImageToMat(self.Frames[i], im);
        MatFrames.push_back(im);
    }
    
    
    //CONVERT
    cout << "Size of self.Frames :" << MatFrames.size() << endl;
    
    float Width = self.view.frame.size.width;
    float Height = self.view.frame.size.height;
    
    imageView00_ = [[UIImageView alloc] initWithFrame:CGRectMake(0.0, 0.0, Width/2, Height/2)];
    imageView01_ = [[UIImageView alloc] initWithFrame:CGRectMake(Width/2, 0.0, Width/2, Height/2)];
    imageView10_ = [[UIImageView alloc] initWithFrame:CGRectMake(0.0, Height/2, Width/2, Height/2)];
    imageView11_ = [[UIImageView alloc] initWithFrame:CGRectMake(Width/2, Height/2, Width/2, Height/2)];
    
    [self.view addSubview:imageView00_];
    imageView00_.contentMode = UIViewContentModeScaleAspectFit;
    imageView00_.clipsToBounds = YES;
    [imageView00_ setImage:self.Frames[0]];
    
    [self.view addSubview:imageView01_];
    imageView01_.contentMode = UIViewContentModeScaleAspectFit;
    imageView01_.clipsToBounds = YES;
    [imageView01_ setImage:self.Frames[1]];
    
    [self.view addSubview:imageView10_];
    imageView10_.contentMode = UIViewContentModeScaleAspectFit;
    imageView10_.clipsToBounds = YES;
    [imageView10_ setImage:self.Frames[2]];
    
    [self.view addSubview: imageView11_];
    imageView11_.contentMode = UIViewContentModeScaleAspectFit;
    imageView11_.clipsToBounds = YES;
    [imageView11_ setImage:self.Frames[3]];
    
    self.spinner = [[UIActivityIndicatorView alloc] initWithActivityIndicatorStyle:UIActivityIndicatorViewStyleWhiteLarge];
    [self.spinner setCenter:CGPointMake(Width/2.0, Height/2.0)];
    [self.view addSubview:self.spinner];
    [self.spinner startAnimating];
    
    cout << endl << "Executing TWO VIEW TRIANGULATION" << endl;
    
    cv::Mat color_im1, color_im2, color_im3, color_im4;
    MatFrames[0].copyTo(color_im1);
    MatFrames[1].copyTo(color_im2);
    MatFrames[2].copyTo(color_im3);
    MatFrames[3].copyTo(color_im4);
    cout<<"size of im1 "<<color_im1.size()<<endl;
    cout<<"size of im2 "<<color_im2.size()<<endl;
    cout<<"size of im3 "<<color_im3.size()<<endl;
    cout<<"size of im4 "<<color_im4.size()<<endl;
    
    std::clock_t start;
    double duration;
    
    start = std::clock();

    
    cv::Mat im1,im2, im3, im4;
    cv::cvtColor(color_im1, im1, cv::COLOR_BGR2GRAY);
    cv::cvtColor(color_im2, im2, cv::COLOR_BGR2GRAY);
    cv::cvtColor(color_im3, im3, cv::COLOR_BGR2GRAY);
    cv::cvtColor(color_im4, im4, cv::COLOR_BGR2GRAY);

    vector<cv::Mat> inputImages;
    inputImages.push_back(im1);
    inputImages.push_back(im2);
    inputImages.push_back(im3);
    inputImages.push_back(im4);
    
    vector<vector<cv::KeyPoint> >keypoints;
    vector<cv::Mat> descriptors;
    vector<vector<cv::DMatch>> total_matches,common_matches;
    vector<vector<cv::Point2f>> pts2D, pts2D_final;
    vector<cv::Mat> projMats,transformedMats;
    detectFeatures(inputImages,keypoints,descriptors);
    computeMatches(inputImages,keypoints,descriptors,total_matches);

    double fx = 818;
    double fy = 818;
    double cx = 270;
    double cy = 480;
    cv::Mat mK = cv::Mat::eye(3,3,CV_64F);
    mK.at<double>(0,0) = fx;
    mK.at<double>(1,1) = fy;
    mK.at<double>(0,2) = cx;
    mK.at<double>(1,2) = cy;
    //std::cout << "--Intrinsic matrix: \n" << mK << std::endl;
    convertKeypointstoCVpoints(keypoints,total_matches,pts2D);
    //cout<<"size of 2d points "<<pts2D.size()<<endl;
    computePoses(pts2D,mK, projMats);
    transFormProjectionMats(projMats,transformedMats);
    /*cout<<"BACK IN MAIN"<<endl;
     transFormProjectionMats(projMats,transformedMats);
     for(int i=0;i<transformedMats.size();++i){
     cout<<transformedMats[i]<<endl;
     }*/
    computeCommonMatches(inputImages,keypoints,descriptors,total_matches,common_matches);
    convertKeypointstoCVpointsSingle(keypoints,common_matches,pts2D_final);
    //cout<<"size of pts2D_final "<<pts2D_final.size()<<endl;
    for(int i=0;i<pts2D_final.size();++i){
        cout<<pts2D_final[i].size()<<endl;
    }
    compute3DPoints(pts2D_final,mK,transformedMats);
    
    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    
    std::cout<<"printf: "<< duration <<'\n';
    
    ofstream xyzPts;
    xyzPts.open("xyzPts.txt");
    
   // [self.spinner stopAnimating];

    
    
}



- (void)didReceiveMemoryWarning {
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}


/*
#pragma mark - Navigation

// In a storyboard-based application, you will often want to do a little preparation before navigation
- (void)prepareForSegue:(UIStoryboardSegue *)segue sender:(id)sender {
    // Get the new view controller using [segue destinationViewController].
    // Pass the selected object to the new view controller.
}
*/

@end

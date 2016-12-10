#include <iostream>
#include <vector>
#include <stdlib.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_io.h>
//#include <pcl_conversions/pcl_conversions.h>

using namespace std;

pcl::PointCloud<pcl::PointXYZ>::Ptr MatToPointXYZ(cv::Mat OpencVPointCloud)
         {
             /*
             *  Function: Get from a Mat to pcl pointcloud datatype
             *  In: cv::Mat
             *  Out: pcl::PointCloud
             */

             //char pr=100, pg=100, pb=100;
             pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);//(new pcl::pointcloud<pcl::pointXYZ>);

             for(int i=0;i<OpencVPointCloud.cols;i++)
             {
                //std::cout<<i<<endl;

                pcl::PointXYZ point;
                point.x = OpencVPointCloud.at<float>(0,i);
                point.y = OpencVPointCloud.at<float>(1,i);
                point.z = OpencVPointCloud.at<float>(2,i);

                // when color needs to be added:
                //uint32_t rgb = (static_cast<uint32_t>(pr) << 16 | static_cast<uint32_t>(pg) << 8 | static_cast<uint32_t>(pb));
                //point.rgb = *reinterpret_cast<float*>(&rgb);

                point_cloud_ptr -> points.push_back(point);


             }
             point_cloud_ptr->width = (int)point_cloud_ptr->points.size();
             point_cloud_ptr->height = 1;

             return point_cloud_ptr;

         }

void display_image(cv::Mat im, std::string window_name="Image",
                   bool user_wait=false, int wait_key=0,
                   bool destroy_window=false) {
    cv::namedWindow(window_name, CV_WINDOW_AUTOSIZE);
    cv::imshow(window_name, im);
    if (user_wait) {
        cv::waitKey(wait_key);
    }
    if (destroy_window) {
        cv::destroyWindow(window_name);
    }
}

void convertToHomo(cv::Mat &src, cv::Mat &dst){
    cv::Mat lastRow = src.row( 3 );
    cv::Mat tmp;
    cv::repeat(lastRow, 4, 1,tmp ); 
    dst=src/tmp;
}

void readImages(const string &path, vector<cv::Mat> &images, int num,const string &extension){
    string im_name;
    for(int i=1;i<=num;++i){
        im_name=path+"/im"+to_string(i)+extension;
        cv::Mat im=imread(im_name,cv::IMREAD_GRAYSCALE);
        images.push_back(im);
    }
}

void displayImages(vector<cv::Mat> &images){
    string imName;
    for(int i=0;i<images.size();++i){
        imName="image"+to_string(i);
        imshow(imName,images[i]);
    }
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
            if (matches[i].distance < std::max(0.01, 1.3 * min_kp_dist)) {
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
        imName="matches"+to_string(i);
        imshow(imName,matchedImage);        
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

void compute3DPoints(vector<vector<cv::Point2f>> &pts2D,cv::Mat &K, vector<cv::Mat> &projMats){
        cv::Mat emp;
        cv::Mat M1=projMats[0];
        vector<cv::Point2f> kp1;
        undistortPoints(pts2D[0], kp1, K,emp);
        for(int i=1;i<pts2D.size();++i){
            vector<cv::Point2f> kp2;
            undistortPoints(pts2D[i], kp2, K,emp);

            //cout<<"undistorted points1 "<<kp1<<endl;
            //cout<<"undistorted points2 "<<kp2<<endl;
            cv::Mat points4D,points4D_trans,points3D,points3D_trans,final3D;
            cv::Mat M2=projMats[i];
            triangulatePoints(M1,M2, kp1,kp2,points4D);
            cv::transpose(points4D,points4D_trans);
            //cout<<points4D_trans<<endl;
            convertToHomo(points4D,points3D);
            cv::Mat temp_trans;
            //cv::transpose(points3D,temp_trans);        
            points3D(cv::Rect(0,0,points3D.cols,points3D.rows-1)).copyTo(temp_trans); 
            pcl::PointCloud<pcl::PointXYZ>::Ptr pclPtr= MatToPointXYZ(temp_trans);
            string filename="test_pcd.pcd"+to_string(i);
            pcl::io::savePCDFileASCII (filename, *pclPtr);
            cv::transpose(temp_trans,final3D);
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

int main(int argc, char* argv[]) {
    if (argc < 5)
    {
        cout<<"Usage: ./bundle_trial <path to images> <number of images> <extension type> <path to settings file>"<<endl;
        return -1;
    }
    // Image filenames
    vector<cv::Mat> inputImages;
    vector<vector<cv::KeyPoint> >keypoints;
    vector<cv::Mat> descriptors;
    vector<vector<cv::DMatch>> total_matches,common_matches;
    vector<vector<cv::Point2f>> pts2D, pts2D_final;
    vector<cv::Mat> projMats,transformedMats;
    string num=argv[2];
    int numImages=atoi(num.c_str());;
    readImages(argv[1],inputImages,numImages,argv[3]);
    //displayImages(inputImages);
    detectFeatures(inputImages,keypoints,descriptors);
    computeMatches(inputImages,keypoints,descriptors,total_matches);
    showMatches(inputImages,keypoints,total_matches);
    cv::Mat mK=readIntrinsics(argv[4]);
    //std::cout << "--Intrinsic matrix: \n" << mK << std::endl;
    convertKeypointstoCVpoints(keypoints,total_matches,pts2D);
    //cout<<"size of 2d points "<<pts2D.size()<<endl;
    computePoses(pts2D,mK, projMats);
    cout<<"BACK IN MAIN"<<endl;
    transFormProjectionMats(projMats,transformedMats);
    for(int i=0;i<transformedMats.size();++i){
        cout<<transformedMats[i]<<endl;
    }
    computeCommonMatches(inputImages,keypoints,descriptors,total_matches,common_matches);
    convertKeypointstoCVpointsSingle(keypoints,common_matches,pts2D_final);
    //cout<<"size of pts2D_final "<<pts2D_final.size()<<endl;
    for(int i=0;i<pts2D_final.size();++i){
        cout<<pts2D_final[i].size()<<endl;
    }
    compute3DPoints(pts2D_final,mK,transformedMats);
    cv::waitKey();
    return 0;   
}

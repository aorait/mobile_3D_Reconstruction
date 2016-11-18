#include "include.h"

int main(int argc, char*argv[])
{
	cv::Mat frame1 = imread("image1.JPG");
	cv::Mat frame2 = imread("image2.JPG");

	int minHessian = 400;
	SurfFeatureDetector detector(minHessian);
	SurfDescriptorExtractor extractor;
	BFMatcher BF;
	vector <DMatch> matches;

	vector <KeyPoint> keypoints1;
	vector <KeyPoint> keypoints2;
	Mat descriptors1;
	Mat descriptors2;

	detector.detect(frame1, keypoints1);
	detector.detect(frame2, keypoints2);

	extractor.compute(frame1, keypoints1, descriptors1);
	extractor.compute(frame2, keypoints2, descriptors2);

	BF.match(descriptors1, descriptors2, matches);
	Mat final;
	drawMatches(frame1, keypoints1, frame2, keypoints2, matches, final);

	double max_dist = 0; double min_dist = 100;

  	//-- Quick calculation of max and min distances between keypoints
  	for( int i = 0; i < descriptors1.rows; i++ )
  	{ double dist = matches[i].distance;
    	if( dist < min_dist ) min_dist = dist;
    	if( dist > max_dist ) max_dist = dist;
 	 }

 	printf("-- Max dist : %f \n", max_dist );
  	printf("-- Min dist : %f \n", min_dist );

  	//-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
  	vector <DMatch> good_matches;

  	for( int i = 0; i < descriptors1.rows; i++ )
  	{ if( matches[i].distance < 3*min_dist )
    	 { good_matches.push_back( matches[i]); }
  	}	

  	cout << good_matches.size() << endl;

	vector <Point2f> F1;
	vector <Point2f> F2;

	for (int i = 0; i < good_matches.size(); i++)
	{
		F1.push_back(keypoints1[good_matches[i].queryIdx].pt);
		F2.push_back(keypoints2[good_matches[i].trainIdx].pt);
	}

	Mat H = findHomography(F1, F2, CV_RANSAC);
	Mat F = findFundamentalMat(F1, F2, FM_RANSAC, 3, 0.99);




	// //Display corresponding points

	// int a = 60;
	// vector <Point2f> frame1_corners(4);
	// frame1_corners[0] = cvPoint(a,a); 
	// frame1_corners[1] = cvPoint( frame1.cols-a, a );
  	// frame1_corners[2] = cvPoint( frame1.cols-a, frame1.rows-a ); 
  	// frame1_corners[3] = cvPoint( a, frame1.rows-a );
  	// std::vector<Point2f> frame2_corners(4);

  	// perspectiveTransform(frame1_corners, frame2_corners, H);

  	// line(frame1, frame1_corners[0], frame1_corners[1], Scalar(0, 255, 0), 4 );
  	// line(frame1, frame1_corners[1], frame1_corners[2], Scalar( 0, 255, 0), 4 );
  	// line(frame1, frame1_corners[2], frame1_corners[3], Scalar( 0, 255, 0), 4 );
  	// line(frame1, frame1_corners[3], frame1_corners[0], Scalar( 0, 255, 0), 4 );

  	// line(frame2, frame2_corners[0], frame2_corners[1], Scalar(0, 255, 0), 4 );
  	// line(frame2, frame2_corners[1], frame2_corners[2], Scalar( 0, 255, 0), 4 );
  	// line(frame2, frame2_corners[2], frame2_corners[3], Scalar( 0, 255, 0), 4 );
  	// line(frame2, frame2_corners[3], frame2_corners[0], Scalar( 0, 255, 0), 4 );


   //  //Show detected matches
  	// imshow( "Good Matches & Object detection", final );
  	// imshow("Frame1", frame1);
  	// imshow("Frame2", frame2);

  	// waitKey(0);
  	// return 0;




	
}

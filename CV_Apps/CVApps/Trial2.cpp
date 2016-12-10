//-(void)TwoViewTriangulation: (vector<cv::Mat>*)MatFrames {
//    //perform two view triangulation
//    
//    cout << endl << "Executing TWO VIEW TRIANGULATION" << endl;
//    //    // Image filenames
//    //    //    UIImage *image = [UIImage imageNamed:@"lena.png"];
//    //    //    cv::Mat im;
//    //    //    UIImageToMat(resImage, im);
//    //    std::string im_name1 = "dino0001.png";//DATA_LOC"dino0001.png";
//    //    std::string im_name2 = "dino0003.png"; //DATA_LOC"dino0003.png";
//    //    NSString* filePath1 = [[NSBundle mainBundle]
//    //                           //pathForResource:@"abhi" ofType:@"jpg"];
//    //                           pathForResource:@"dino0001" ofType:@"png"];
//    //    //pathForResource:@"abhi_new" ofType:@"jpg"];
//    //    UIImage* resImage1 = [UIImage imageWithContentsOfFile:filePath1];
//    //
//    //    NSString* filePath2 = [[NSBundle mainBundle]
//    //                           //pathForResource:@"abhi" ofType:@"jpg"];
//    //                           pathForResource:@"dino0003" ofType:@"png"];
//    //    //pathForResource:@"abhi_new" ofType:@"jpg"];
//    //    UIImage* resImage2 = [UIImage imageWithContentsOfFile:filePath2];
//    //
//    //    // Next convert to a cv::Mat
//    //    cv::Mat color_im1,color_im2;
//    //    UIImageToMat(resImage1, color_im1);
//    //    UIImageToMat(resImage2, color_im2);
//    
//    vector<cv::Mat>& Frames = *MatFrames;
//    cv::Mat color_im1, color_im2;
//    cout<<MatFrames->size()<<endl;
//    Frames[0].copyTo(color_im1);
//    Frames[1].copyTo(color_im2);
//    cout<<"size of im1 "<<color_im1.size()<<endl;
//    cout<<"size of im2 "<<color_im2.size()<<endl;
//    
//    cv::Mat im1,im2;
//    cv::cvtColor(color_im1, im1, cv::COLOR_BGR2GRAY);
//    cv::cvtColor(color_im2, im2, cv::COLOR_BGR2GRAY);
//    
//    /*  cout << im_name1 << endl;
//     cout << im_name2 << endl;
//     
//     UIImage *image_1 = [UIImage imageNamed:@"dino0001.png"];
//     cv::Mat im1;
//     //cv::UIImageToMat(image_1, im1);
//     
//     UIImage *image_2 = [UIImage imageNamed:@"dino0003.png"];
//     cv::Mat im2;
//     cv::UIImageToMat(image_2, im2);*/
//    
//    
//    
//    
//    
//    
//    //    // Read in images
//    //    cv::Mat im1 = cv::imread(im_name1, cv::IMREAD_GRAYSCALE);
//    //    cv::Mat im2 = cv::imread(im_name2, cv::IMREAD_GRAYSCALE);
//    
//    //display_image(im1, "image1");
//    //display_image(im2, "image2");
//    
//    // Detect SURF keypoints and compute feature descriptors at those keypoints
//    int min_hessian = 400;
//    cv::Ptr<cv::xfeatures2d::SURF> surf_detector = cv::xfeatures2d::SURF::create(min_hessian);
//    
//    std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
//    cv::Mat descriptors_1, descriptors_2;
//    
//    surf_detector->detectAndCompute(im1, cv::Mat(), keypoints_1, descriptors_1);
//    surf_detector->detectAndCompute(im2, cv::Mat(), keypoints_2, descriptors_2);
//    
//    std::cout << "im1 detected " << keypoints_1.size() << " keypoints" << std::endl;
//    std::cout << "im2 detected " << keypoints_2.size() << " keypoints" << std::endl;
//    
//    // Match descriptors using FLANN
//    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce");
//    
//    std::vector<cv::DMatch> matches;
//    matcher->match(descriptors_2, descriptors_1, matches);
//    
//    double min_kp_dist = std::numeric_limits<double>::max();
//    double max_kp_dist = std::numeric_limits<double>::min();
//    
//    for (int i = 0; i < matches.size(); ++i) {
//        double cur_dist = matches[i].distance;
//        if (min_kp_dist > cur_dist) {   min_kp_dist = cur_dist; }
//        if (max_kp_dist < cur_dist) {   max_kp_dist = cur_dist; }
//    }
//    
//    std::cout << "-- Min keypoint distance = " << min_kp_dist << std::endl;
//    std::cout << "-- Max keypoint distance = " << max_kp_dist << std::endl;
//    
//    // Compute the "good" set of matches
//    std::vector<cv::DMatch> good_matches;
//    for (int i = 0; i < matches.size(); ++i) {
//        if (matches[i].distance < std::max(MIN_KP_DIST, 2 * min_kp_dist)) {
//            good_matches.push_back(matches[i]);
//        }
//    }
//    std::cout << "-- #good matches = " << good_matches.size() << std::endl;
//    
//    // Get keypoint locations
//    std::vector<cv::Point2f> kp_loc_1, kp_loc_2;
//    for (int i = 0; i < good_matches.size(); ++i) {
//        kp_loc_1.push_back(keypoints_1[good_matches[i].trainIdx].pt);
//        kp_loc_2.push_back(keypoints_2[good_matches[i].queryIdx].pt);
//    }
//    
//    
//    // Draw keypoints
//    cv::Mat img_keypoints_1, img_keypoints_2;
//    
//    cv::drawKeypoints(im1, keypoints_1, img_keypoints_1, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
//    cv::drawKeypoints(im2, keypoints_2, img_keypoints_2, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
//    
//    // Show detected keypoints
//    cout<<"reached here 1"<<endl;
//    
//    // Show good matches
//    cv::Mat im_matches;
//    cv::drawMatches(im2, keypoints_2, im1, keypoints_1, good_matches, im_matches, cv::Scalar::all(-1),
//                    cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
//    cout<<"reached here 2"<<endl;
//    
//    //cv::imshow(im_matches, "good_matches");
//    double cx=302.32; double cy=246.87;
//    cv::Point2d principle_point=cv::Point2d(cx, cy);
//    // Intrinsic matrix
//    cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
//    K.at<double>(0, 0) = FOCAL_LEN_X;
//    K.at<double>(1, 1) = FOCAL_LEN_Y;
//    K.at<double>(0, 2) = cx;
//    K.at<double>(1, 2) = cy;
//    std::cout << "--Intrinsic matrix: \n" << K << std::endl;
//    
//    // Compute essential matrix
//    cv::Mat F = cv::findFundamentalMat(kp_loc_1, kp_loc_2, cv::FM_RANSAC, 1, 0.99);
//    std::cout << "-- Fundamental matrix: \n" << F << std::endl;
//    
//    cv::Mat E_fromF = K.t() * F * K;
//    std::cout << "-- Essential matrix from F: \n" << E_fromF << std::endl;
//    
//    // Draw epipolar lines
//    /*std::vector<cv::Vec3f> epi_lines_left;
//     cv::computeCorrespondEpilines(kp_loc_2, 2, F, epi_lines_left);
//     visualize_epipolar_lines(im1, kp_loc_1, im2, kp_loc_2, epi_lines_left);
//     
//     std::vector<cv::Vec3f> epi_lines_right;
//     cv::computeCorrespondEpilines(kp_loc_1, 1, F, epi_lines_right);
//     visualize_epipolar_lines(im2, kp_loc_2, im1, kp_loc_1, epi_lines_right, "right");*/
//    
//    
//    // Compute essential matrix
//    cv::Mat E = cv::findEssentialMat(kp_loc_1, kp_loc_2, FOCAL_LEN_X, principle_point);
//    std::cout << "-- Essential matrix: \n" << E << std::endl;
//    
//    //using E of homework
//    cv::Mat E_homework = (cv::Mat_<double>(3,3) << 0.0047, 0.4448, -3.3384,
//                          0.4302, -0.0082, 0.0868,
//                          3.3480, 0.0263, 0.0012);
//    
//    cout<<"E from homework "<<endl<<E_homework<<endl;
//    
//    //Decompose Essential matrix to R and t
//    cv::Mat R2,t2,R2_1,R2_2,t2_1,t2_2;
//    cv::decomposeEssentialMat(E, R2_1, R2_2, t2_1);
//    t2_2=(-1)*t2_1;
//    //int inliersE=recoverPose(E, kp_loc_1, kp_loc_2, R2, t2, FOCAL_LEN_X,principle_point);
//    
//    //Form projection matrices
//    cv::Mat R1 = cv::Mat::eye(3, 3, CV_64F);
//    cv::Mat t1= cv::Mat::zeros(3,1, CV_64F);
//    cv::Mat M1,M2_1,M2_2,M2_3,M2_4;
//    cv::hconcat(R1,t1,M1);
//    cv::hconcat(R2_1,t2_1,M2_1);
//    cv::hconcat(R2_1,t2_2,M2_2);
//    cv::hconcat(R2_2,t2_1,M2_3);
//    cv::hconcat(R2_2,t2_2,M2_4);
//    cout<<"M1 "<<endl<<M1<<endl;
//    cout<<"M2_1 "<<endl<<M2_1<<endl;
//    cout<<"M2_2 "<<endl<<M2_2<<endl;
//    cout<<"M2_3 "<<endl<<M2_3<<endl;
//    cout<<"M2_4 "<<endl<<M2_4<<endl;
//    
//    //Undistort points before passing to triangulate since it uses only extrinsics.
//    double min_val, max_val;
//    cv::Mat emp;
//    undistortPoints(kp_loc_1, kp_loc_1, K,emp);
//    undistortPoints(kp_loc_2, kp_loc_2, K,emp);
//    
//    //Triangulate points 4 combinations (should be converted into a function)
//    cv::Mat points4D_1,points4D_trans_1,points3D_1,points3D_trans_1;
//    triangulatePoints(M1,M2_1, kp_loc_1,kp_loc_2,points4D_1);
//    convertToHomo(points4D_1,points3D_1);
//    cv::Mat temp_1,temp_trans_1;
//    cv::transpose(points3D_1,temp_trans_1);
//    cout<<"3D points 1"<<endl<<temp_trans_1<<endl;
//    
//    cv::Mat temp_trans;
//    cv::minMaxLoc(temp_trans_1.col(2), &min_val, &max_val);
//    cout<<"min_val "<<min_val<<endl;
//    cout<<"max_val "<<max_val<<endl;
//    if(min_val>=0){
//        temp_trans_1.copyTo(temp_trans);
//    }
//    
//    cv::Mat points4D_2,points4D_trans_2,points3D_2,points3D_trans_2;
//    triangulatePoints(M1,M2_2, kp_loc_1,kp_loc_2,points4D_2);
//    convertToHomo(points4D_2,points3D_2);
//    cv::Mat temp_2,temp_trans_2;
//    cv::transpose(points3D_2,temp_trans_2);
//    cout<<"3D points 2"<<endl<<temp_trans_2<<endl;
//    
//    cv::minMaxLoc(temp_trans_2.col(2), &min_val, &max_val);
//    cout<<"min_val "<<min_val<<endl;
//    cout<<"max_val "<<max_val<<endl;
//    if(min_val>=0){
//        temp_trans_2.copyTo(temp_trans);
//    }
//    
//    cv::Mat points4D_3,points4D_trans_3,points3D_3,points3D_trans_3;
//    triangulatePoints(M1,M2_3, kp_loc_1,kp_loc_2,points4D_3);
//    convertToHomo(points4D_3,points3D_3);
//    cv::Mat temp_3,temp_trans_3;
//    cv::transpose(points3D_3,temp_trans_3);
//    cout<<"3D points 3"<<endl<<temp_trans_3<<endl;
//    
//    cv::minMaxLoc(temp_trans_3.col(2), &min_val, &max_val);
//    cout<<"min_val "<<min_val<<endl;
//    cout<<"max_val "<<max_val<<endl;
//    if(min_val>=0){
//        temp_trans_3.copyTo(temp_trans);
//    }
//    
//    cv::Mat points4D_4,points4D_trans_4,points3D_4,points3D_trans_4;
//    triangulatePoints(M1,M2_4, kp_loc_1,kp_loc_2,points4D_4);
//    convertToHomo(points4D_4,points3D_4);
//    cv::Mat temp_4,temp_trans_4;
//    cv::transpose(points3D_4,temp_trans_4);
//    cout<<"3D points 4"<<endl<<temp_trans_4<<endl;
//    
//    cv::minMaxLoc(temp_trans_4.col(2), &min_val, &max_val);
//    cout<<"min_val "<<min_val<<endl;
//    cout<<"max_val "<<max_val<<endl;
//    
//    if(min_val>=0){
//        temp_trans_4.copyTo(temp_trans);
//    }
//    
//    //for writing into points cloud file
//    cv::Mat temp,final3D;
//    temp_trans(cv::Rect(0,0,temp_trans.cols-1,temp_trans.rows)).copyTo(temp);
//    cv::transpose(temp,final3D);
//    
//    cout << "temp :" << endl << temp << endl;
//    
//    ofstream xyzPts;
//    xyzPts.open("xyzPts.txt");
//    xyzPts << temp << endl;
//    
//    
//    }

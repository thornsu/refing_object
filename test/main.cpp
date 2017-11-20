#include <iostream>
#include "my_refind/config.h"

Point2f pixel2cam(const Point2d& p, const Mat& K);

int main(int argc, char **argv) 
{
	if(argc!=5)
	{
		cerr<<"usage: ./exe config_file detect_file"<<endl;
		return -1;
	}
	
	Mat img_left,img_right;
	img_left=imread(argv[1]);
	img_right=imread(argv[2]);
	
	Config stereo(argv[3]);
	
	CascadeClassifier cascade;
    cascade.load(argv[4]);
	
	Mat gray_left,gray_right;
	
	cvtColor(img_left,gray_left,COLOR_BGR2GRAY);
	equalizeHist(gray_left,gray_left);
	cvtColor(img_right,gray_right,COLOR_BGR2GRAY);
	equalizeHist(gray_right,gray_right);
	
	vector<Rect>face_left;
	cascade.detectMultiScale(gray_left,face_left,1.1,2,0| CASCADE_SCALE_IMAGE, Size(30,30));
	
	vector<Rect>face_right;
	cascade.detectMultiScale(gray_right,face_right,1.1,2,0| CASCADE_SCALE_IMAGE, Size(30,30));
	
	/*
	for(size_t i=0; i<face_left.size(); i++)
    {
        Rect r = face_left[i];
         rectangle(img_left, r, Scalar(255,255,0),3,3,0);
     }
     */
	Mat search_left = img_left(face_left[0]).clone();

	
	vector<KeyPoint>keypoints_left,keypoints_right;
	
	Mat desciptors_left,descriptors_right;
	
	Ptr<ORB> orb=ORB::create();
	orb->detect(search_left,keypoints_left);
	orb->detect(img_right(face_right[0]),keypoints_right);
	
	for(size_t i=0; i<keypoints_left.size(); i++)
    {
        keypoints_left[i].pt=keypoints_left[i].pt + Point2f(face_left[0].x,face_left[0].y);
    }
    for(size_t i=0; i<keypoints_right.size(); i++)
    {
        keypoints_right[i].pt=keypoints_right[i].pt + Point2f(face_right[0].x,face_right[0].y);
    }
	
	orb->compute(img_left,keypoints_left,desciptors_left);
	orb->compute(img_right,keypoints_right,descriptors_right);
	
	vector<DMatch> matches;
    BFMatcher matcher (NORM_HAMMING);
    matcher.match(desciptors_left,descriptors_right,matches);
	
	double max_dist=0;
    for(size_t i=0; i<desciptors_left.rows; i++)
    {
        double dist = matches[i].distance;
        if(dist>max_dist)
            max_dist=dist;
    }
    
    vector<DMatch> good_matches;
    for(size_t i=0; i<desciptors_left.rows; i++)
    {
        if(matches[i].distance <= 0.5*max_dist)
            good_matches.push_back(matches[i]);
    }
	
	
	Mat matches_show;
    drawMatches(img_left,keypoints_left,img_right,keypoints_right,good_matches,matches_show);
    imshow("show_matches",matches_show);
     
	Mat T1 = (Mat_<double> (3,4) <<
        1,0,0,0,
        0,1,0,0,
        0,0,1,0);
	Mat T2 = (Mat_<double> (3,4) <<
        stereo.L2R_R.at<double>(0,0),stereo.L2R_R.at<double>(0,1),stereo.L2R_R.at<double>(0,2),stereo.L2R_T.at<double>(0,0),
        stereo.L2R_R.at<double>(1,0),stereo.L2R_R.at<double>(1,1),stereo.L2R_R.at<double>(1,2),stereo.L2R_T.at<double>(1,0),
        stereo.L2R_R.at<double>(2,0),stereo.L2R_R.at<double>(2,1),stereo.L2R_R.at<double>(2,2),stereo.L2R_T.at<double>(2,0));
	
	Mat R = (Mat_<double>(3,3)<<
		stereo.L2R_R.at<double>(0,0),stereo.L2R_R.at<double>(0,1),stereo.L2R_R.at<double>(0,2),
        stereo.L2R_R.at<double>(1,0),stereo.L2R_R.at<double>(1,1),stereo.L2R_R.at<double>(1,2),
        stereo.L2R_R.at<double>(2,0),stereo.L2R_R.at<double>(2,1),stereo.L2R_R.at<double>(2,2));
	
	Mat T = (Mat_<double>(3,1)<<
		stereo.L2R_T.at<double>(0,0),stereo.L2R_T.at<double>(1,0),stereo.L2R_T.at<double>(2,0)
	);
	
	
	vector<Point2d> pts_1,pts_2;
	for(DMatch m:good_matches)
	{
		pts_1.push_back(pixel2cam(keypoints_left[m.queryIdx].pt,stereo.K_l));
		pts_2.push_back(pixel2cam(keypoints_right[m.trainIdx].pt,stereo.K_r));
	}
	
	/*
	for(size_t i=0;i<good_matches.size();i++)
	{
		cout<<keypoints_left[good_matches[i].queryIdx].pt<<endl;
		cout<<pts_1[i]<<endl;
		
		cout<<keypoints_right[good_matches[i].trainIdx].pt<<endl
						<<pts_2[i]<<endl<<endl;
	}
	*/
	
	Mat pts_4d;
	triangulatePoints(T1,T2,pts_1,pts_2,pts_4d);
	
	vector<Point3d> points;
	
	
	cout<<"3d marks"<<endl;
	
	for(int i=0; i<pts_4d.cols;i++)
	{
		Mat x=pts_4d.col(i);
		x/=x.at<double>(3,0);
		Point3d p(
			x.at<double>(0,0),
			x.at<double>(1,0),
			x.at<double>(2,0)
		);
		points.push_back(p);
	}
	
	for(int i=0;i<points.size();i++)
	{
		
		Point2d pts1_cam(points[i].x/points[i].z,
										points[i].y/points[i].z
		);
		
		cout<<"point in left camera: "<<pts_1[i]<<endl; 
		cout<<"point projected from 3d"<<pts1_cam<<"  d="<<points[i].z<<endl;
	}
	
	
	cout<<endl;
	
	vector<Point3d> points_right;
	
	for(int i=0;i<points.size();i++)
	{
		Mat x = R*(Mat_<double>(3,1)<<points[i].x,points[i].y,points[i].z)+T;
		Point3d p(
			x.at<double>(0,0),
			x.at<double>(1,0),
			x.at<double>(2,0)
		);
		points_right.push_back(p);
	}
	
	for(int i=0;i<points_right.size();i++)
	{
		
		Point2d pts2_cam(points_right[i].x/points_right[i].z,
										points_right[i].y/points_right[i].z
		);
		
		cout<<"point in right camera: "<<pts_2[i]<<endl; 
		cout<<"point projected from 3d"<<pts2_cam<<"  d="<<points_right[i].z<<endl;
	}
	
	
	
	 waitKey();
	
    std::cout << "Hello, world!" << std::endl;
    return 0;
}

Point2f pixel2cam ( const Point2d& p, const Mat& K )
{
    return Point2f
    (
        ( p.x - K.at<double>(0,2) ) / K.at<double>(0,0), 
        ( p.y - K.at<double>(1,2) ) / K.at<double>(1,1) 
    );
}

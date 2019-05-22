#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iomanip>
#include <boost/thread/mutex.hpp>
#include <knu_ros_lecture/finalproject.h>

using namespace cv;
using namespace std;


boost::mutex mutex;
double g_avg_x=0;
double g_avg_size=0;
bool onoff = false;

double angle(Point pt1, Point pt2, Point pt0) {
	double dx1 = pt1.x - pt0.x;
	double dy1 = pt1.y - pt0.y;
	double dx2 = pt2.x - pt0.x;
	double dy2 = pt2.y - pt0.y;
	return (dx1*dx2 + dy1*dy2) / sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

void find_squares(Mat& image, vector< vector< Point> >& squares)
{
	// blur will enhance edge detection
	GaussianBlur(image, image, Size(5, 5), 1, 1);
	Mat gray0;
	cvtColor(image, gray0, CV_BGR2GRAY);
	Mat gray(image.size(), CV_8U);
	vector< vector< Point> > contours;

	gray = gray0 >= 100;

	// Find contours and store them in a list
	findContours(gray, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
	// Test contours
	vector< Point> approx;
	for (size_t i = 0; i < contours.size(); i++)
	{
		// approximate contour with accuracy proportional
		// to the contour perimeter
		approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.02, true);

		// Note: absolute value of an area is used because
		// area may be positive or negative - in accordance with the
		// contour orientation
		if (approx.size() == 4 &&
			fabs(contourArea(Mat(approx))) > 1000 &&
			isContourConvex(Mat(approx)))
		{
			double maxCosine = 0;

			for (int j = 2; j < 5; j++)
			{
				double cosine = fabs(angle(approx[j % 4], approx[j - 2], approx[j - 1]));
				maxCosine = MAX(maxCosine, cosine);
			}

			if (maxCosine < 0.3)
				squares.push_back(approx);
		}
	}
}


void find_largest_square(const vector<vector <Point> >& squares, vector<Point>& biggest_square0,vector<Point>& biggest_square1) {
	if (!squares.size()) {
		return;
	}

	int max_width = 0;
	int max_height = 0;
	int max_square_idx0 = 0;
	int max_square_idx1 = 0;
	const int n_points = 4;

	for (size_t i = 0; i < squares.size(); i++) {
		Rect rectangle = boundingRect(Mat(squares[i]));
		if ((rectangle.width >= max_width) && (rectangle.height >= max_height)) {
			max_width = rectangle.width;
			max_height = rectangle.height;
			max_square_idx1 = max_square_idx0;
			max_square_idx0 = i;
		}
	}
	biggest_square0 = squares[max_square_idx0];
	biggest_square1 = squares[max_square_idx1];

}


// A callback function. Executed eack time a new pose message arrives.
void poseMessageReceivedRGB(const sensor_msgs::ImageConstPtr& msg) {
	int squarenum = 0;
	double min_x = 9999, max_x = -9999;
	double avg_x=0;
	double avg_size=0;

	Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;

	Mat img_origin = img.clone();

	vector< vector< Point> > squares;
	find_squares(img, squares);
	vector<Point> largest_square0;
	vector<Point> largest_square1;
	find_largest_square(squares, largest_square0, largest_square1);

	if (largest_square0.size() > 0) {
		for (int i = 0; i < 4; i++) {
			line(img, largest_square0[i], largest_square0[(i + 1) % 4], Scalar(0, 0, 255), 3, CV_AA);
			avg_x += largest_square0[i].x;	
			if (min_x > largest_square0[i].x)
				min_x = largest_square0[i].x;
			if (max_x < largest_square0[i].x)
				max_x = largest_square0[i].x;
		}
		avg_size += max_x - min_x;
		squarenum++;
	}
	min_x = 9999; max_x = -9999;
	if (largest_square1.size() > 0) {
		for (int i = 0; i < 4; i++) {
			line(img, largest_square1[i], largest_square1[(i + 1) % 4], Scalar(0, 0, 255), 3, CV_AA);
			avg_x += largest_square0[i].x;	
			if (min_x > largest_square0[i].x)
				min_x = largest_square0[i].x;
			if (max_x < largest_square0[i].x)
				max_x = largest_square0[i].x;
		}
		avg_size += max_x - min_x;
		squarenum++;
	}

	avg_x /= (squarenum*4);
	avg_size /= squarenum;

	//가장 큰 두 사각형의 x좌표 평균
	//가장 큰 두 사각형의 크기 평균
	mutex.lock(); {
		g_avg_x = avg_x;
		g_avg_size = avg_size;
		if (squarenum > 0)
			onoff = true;
		else
			onoff = false;
	} mutex.unlock();
	
	

	imshow("img_origin", img_origin);
	imshow("squares", img);
	waitKey(30);
}


int main(int argc, char **argv)
{
	// Initialize the ROS system
	ros::init(argc, argv, "turtle_rpiCamera_detection_sample");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);

	// Create a subscriber object
	image_transport::Subscriber subRGB = it.subscribe("/raspicam_node/image", 1, &poseMessageReceivedRGB, ros::VoidPtr(), image_transport::TransportHints("compressed"));
	ros::Publisher pub = nh.advertise<knu_ros_lecture::finalproject>("finalproject", 100);

	// Let ROS take over
	while(ros::ok()){
		ros::spinOnce();

		mutex.lock(); {
			if (onoff)
			{
				//x좌표에 대한거 전송
				knu_ros_lecture::finalproject msg;
				msg.avg_x = g_avg_x;
				msg.avg_size = g_avg_size;
				pub.publish(msg);
			}
		} mutex.unlock();
	}
	return 0;
}

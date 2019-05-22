#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <boost/thread/mutex.hpp>
#include <tf/tf.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/LaserScan.h>
#include <iomanip>
#include <unistd.h>
#include <stdlib.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/CompressedImage.h>
#include <image_transport/image_transport.h>
#include <knu_ros_lecture/finalproject.h>

using namespace cv;
using namespace std;

#define toRadian(degree)	((degree) * (M_PI / 180.))
#define toDegree(radian)	((radian) * (180. / M_PI))

boost::mutex mutex[3];
nav_msgs::Odometry g_odom;
sensor_msgs::LaserScan g_scan;
knu_ros_lecture::finalproject g_msg1;

// A template method to check 'nan'
template<typename T>
inline bool isnan(T value)
{
    return value != value;
}

// callback function
void odomMsgCallback(const nav_msgs::Odometry &msg)
{
    // receive a '/odom' message with the mutex
    mutex[0].lock(); {
        g_odom = msg;
    } mutex[0].unlock();
}

// callback function
void scanMsgCallback(const sensor_msgs::LaserScan& msg)
{
    // receive a '/odom' message with the mutex
    mutex[1].lock(); {
        g_scan = msg;
    } mutex[1].unlock();
}

// callback function
void fMsgCallback(const knu_ros_lecture::finalproject& msg)
{
    // receive a '/odom' message with the mutex
    mutex[2].lock(); {
        g_msg1 = msg;
    } mutex[2].unlock();
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void convertOdom2XYZRPY(nav_msgs::Odometry &odom, Vec3d &xyz, Vec3d &rpy)
{
    // 이동 저장
    xyz[0] = odom.pose.pose.position.x;
    xyz[1] = odom.pose.pose.position.y;
    xyz[2] = odom.pose.pose.position.z;

    // 회전 저장
    tf::Quaternion rotationQuat = tf::Quaternion(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
    tf::Matrix3x3(rotationQuat).getEulerYPR(rpy[2], rpy[1], rpy[0]);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void convertScan2XYZs(sensor_msgs::LaserScan& lrfScan, vector<Vec3d> &XYZs)
{
    int nRangeSize = (int)lrfScan.ranges.size();
    XYZs.clear();
    XYZs.resize(nRangeSize);

    for(int i=0; i<nRangeSize; i++) {
        double dRange = lrfScan.ranges[i];

        if(isnan(dRange)) {
            XYZs[i] = Vec3d(0., 0., 0.);
        } else {
            double dAngle = lrfScan.angle_min + i*lrfScan.angle_increment;
            XYZs[i] = Vec3d(dRange*cos(dAngle), dRange*sin(dAngle), 0.);
        }
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void initGrid(Mat &display, int nImageSize)
{
    const int nImageHalfSize = nImageSize/2;
    const int nAxisSize = nImageSize/16;
    const Vec2i imageCenterCooord = Vec2i(nImageHalfSize, nImageHalfSize);
    display = Mat::zeros(nImageSize, nImageSize, CV_8UC3);
    line(display, Point(imageCenterCooord[0], imageCenterCooord[1]), Point(imageCenterCooord[0]+nAxisSize, imageCenterCooord[1]), Scalar(0, 0, 255), 2);
    line(display, Point(imageCenterCooord[0], imageCenterCooord[1]), Point(imageCenterCooord[0], imageCenterCooord[1]+nAxisSize), Scalar(0, 255, 0), 2);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// A callback function. Executed eack time a new pose message arrives.
void drawLRFScan(Mat &display, vector<Vec3d> &laserScanXY, double dMaxDist)
{
    Vec2i imageHalfSize = Vec2i(display.cols/2, display.rows/2);
    int nRangeSize = (int)laserScanXY.size();

    for(int i=0; i<nRangeSize; i++) {
        int x = imageHalfSize[0] + cvRound((laserScanXY[i][0]/dMaxDist)*imageHalfSize[0]);
        int y = imageHalfSize[1] + cvRound((laserScanXY[i][1]/dMaxDist)*imageHalfSize[1]);

        if(x >= 0 && x < display.cols && y >= 0 && y < display.rows) {
            display.at<Vec3b>(y, x) = Vec3b(255, 255, 0);
        }
    }
}

//Twist 메세지 
geometry_msgs::Twist movetopeople(sensor_msgs::LaserScan& lrfScan)
{
	int i,j; double dRangesum; double dRange; double minrange = 99999, minrangeidx = -9999;
	int nRangeSize = (int)lrfScan.ranges.size();
	double angle = 0, linear = 0;
	int sumnum = 0;

	geometry_msgs::Twist msgT;

	for (i = 0; i < (nRangeSize); i++)
	{
		dRangesum = 0;
		sumnum = 0;

		//점 21(노이즈 대비)개의 평균값 
		for (j = -10; j <= 10; j++)
		{
			if (i+j < nRangeSize && i+j > 0)
				dRange = lrfScan.ranges[i+j];
			else if (i+j > nRangeSize)
				dRange = lrfScan.ranges[i+j-nRangeSize];
			else if (i+j < 0)
				dRange = lrfScan.ranges[i+j+nRangeSize];

			if(!isnan(dRange)&&dRange>0)
			{	
				dRangesum += dRange;
				sumnum++;
			}
		}

		if (sumnum != 0)
			dRangesum /= sumnum;

		//평균값의 최소값
		if(!isnan(dRangesum)&&dRangesum>0)
		{
			if (minrange > dRangesum)
			{
				minrange = dRangesum;
				minrangeidx = i;
			}
		}
	}

	//최소값의 인덱스가 터틀봇 왼쪽에 있으면 좌회전, 오른쪽에 있으면 우회전
	if (minrangeidx == -9999)
		angle = 0;
	else if ( 0 <= minrangeidx && minrangeidx < nRangeSize/8 )
		angle = 0.25;
	else if ( nRangeSize/8 <= minrangeidx && minrangeidx < nRangeSize/2 )
		angle = 0.5;
	else if ( nRangeSize/2 <= minrangeidx && minrangeidx < nRangeSize*7/8 )
		angle = -0.5;
	else if ( nRangeSize*7/8 <= minrangeidx && minrangeidx < nRangeSize )
		angle = -0.25;

	//전방 60도 각도에 가장 가까운게 있으면 25cm 유지
	if ( minrangeidx > 0 && (minrangeidx < nRangeSize/12 || minrangeidx > nRangeSize*11/12))
	{
		if (minrange > 0.5)
			linear = 0.15;
		else if (minrange > 0.25)
			linear = 0.1;
		else if (minrange < 0.15)
			linear = -0.1;
	}
	else//직진x
	{
		linear = 0;
	}

	msgT.linear.x = linear;
	msgT.angular.z = angle;
	printf("linear = %lf\n",linear);
	printf("angle = %lf\n",angle);
	printf("minrange = %lf\n",minrange);
	printf("minrangeidx = %lf\n",minrangeidx);
	printf("\n");
	return msgT;
}

int main (int argc, char **argv)
{
	ros::init(argc, argv, "project");
	ros::NodeHandle nh;

	ros::Subscriber subOdom = nh.subscribe("/odom", 100, &odomMsgCallback);
	ros::Subscriber subScan = nh.subscribe("/scan", 10, &scanMsgCallback);
	ros::Subscriber subf = nh.subscribe("finalproject", 100, &fMsgCallback);
	ros::Publisher pubT = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

	Mat display;
	initGrid(display, 801);

	nav_msgs::Odometry odom;
	sensor_msgs::LaserScan scan;
	knu_ros_lecture::finalproject msg1;
	geometry_msgs::Twist msgT;

	bool start = false;
	
	// 이동 및 회전 정보
	Vec3d xyz, rpy;

	// LRF scan 정보
	vector<Vec3d> laserScanXY;

	// Mat distance for grid
	const double dGridMaxDist = 4.0;

	while(ros::ok()) {
		//현재 position 읽기
		// callback 함수을 call!
		ros::spinOnce();

		mutex[0].lock(); {
			odom = g_odom;
		} mutex[0].unlock();

		// odom으로부터 이동 및 회전정보 획득
		convertOdom2XYZRPY(odom, xyz, rpy);

		// receive the global '/scan' message with the mutex
		mutex[1].lock(); {
			scan = g_scan;
		} mutex[1].unlock();

		mutex[2].lock(); {
			msg1 = g_msg1;
		} mutex[2].unlock();
		// scan으로부터 Cartesian X-Y scan 획득
		convertScan2XYZs(scan, laserScanXY);
		
		if (!start)
		{
			sleep(2);
			start = true;
		}
		
		msgT = movetopeople(scan);
		pubT.publish(msgT);
		
		//위치 및 경로 출력
		initGrid(display, 801);
		drawLRFScan(display, laserScanXY, dGridMaxDist);
		transpose(display, display);  // X-Y축 교환
		flip(display, display, 0);  // 수평방향 반전
		flip(display, display, 1);  // 수직방향 반전

		// 영상 출력!
		imshow("turtle_map", display);

		// 사용자의 키보드 입력을 받음!
		int nKey = waitKey(30) % 255;

		if(nKey == 27) {
			// 종료
			break;
		} else if(nKey == ' ') {
          
		} else if(nKey == 'c' || nKey == 'C') {
			initGrid(display, 801);
		}
	}
}

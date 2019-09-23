#include<opencv2/opencv.hpp>
//#include<image_transport/image_transport.h>
#include<iostream>
#include<string>
#include<cmath>
#include<stdlib.h>
#include<stdio.h>
#include"ros/ros.h"
#include"send_msg/rt.h"
//#include<cv_bridge/cv_bridge.h>
using namespace cv;
using namespace std;

static void on_trackbar( int, void* );
void find_corners(vector<Point2f> &, vector<Point2f> &, Rect &, RotatedRect &, Mat &, vector<vector<Point>> & , int);
void rotated_matrix(vector<Point2f> &, vector<Point2f> &, Mat &, Mat &,
                    vector<Point3f> &, Mat &, Mat &);
Point center_point(vector<Point2f> &, vector<Point2f> &);
Vec3f angles;
int max_y(vector<Point2f> & corner);
int min_y(vector<Point2f> &);

double theta_x;
double theta_y;
double theta_z;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "send_msg");
    ros::NodeHandle n;
    // ros::NodeHandle nh;
    ros::Publisher send = n.advertise<send_msg::rt>("joker", 100);
    // image_transport::ImageTransport it(nh);
    // image_transport::Publisher pub = it.advertise("image", 1);
    Mat src_img;
    Mat gray_img;
    Mat hsv_img;
    Mat output_img;
    Mat inRange_img;
    Mat canny_img;
    Mat erode_img;	
    Mat dilate_img;
    Mat contours_img;
    Mat struct_element1 = getStructuringElement(MORPH_RECT, Size(7,7), Point(-1,-1));
    Mat mask;
    Mat camMatrix;
    Mat distCoeff;
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    vector<Point2f> corners1;
    vector<Point2f> corners2;
    vector<Point2f> light_bar1(2);
    vector<Point2f> light_bar2(2);
    Mat rvec;
    Mat tvec;
    RotatedRect ellipse0;
    RotatedRect ellipse1;
    Rect rect;
    int hmax = 40, hmin = 0;
    int vmax = 255, vmin = 40;
    int smax = 255, smin = 10;
    int bigrect_x ;
    int bigrect_y ;
    int macth_num;
    int min_num = -1;
    int width;//装甲片的长和宽//
    int height;
    int x_diff;//两个轮廓的水平距离//
    double scale3;
    float current_scale;
    float min_scale= 1.2;
    float rect_scale0;
    float rect_scale1;
    float rect_scale3 ;

    VideoCapture cap("/home/joker/water.avi");
    vector<Point3f> world_cor;
    world_cor.push_back(Point3f(-6.75, -2.75, 0));
    world_cor.push_back(Point3f(-6.75, 2.75, 0));
    world_cor.push_back(Point3f(6.75, 2.75, 0));
    world_cor.push_back(Point3f(6.75, -2.75, 0));
    camMatrix = (Mat_<double>(3, 3) << 1128.048344, 0, 339.421769, 0, 1127.052190, 236.535242, 0, 0, 1);
    distCoeff = (Mat_<double>(5, 1) << -0.568429,0.514592,-0.000126,0.00500,0.00000);

    while(1)
    {
        send_msg::rt para;
        if(!cap.isOpened())
        {
            cout << "No video is opened!";
        }

        cap >> src_img;
        cap >> output_img;
        

        if(src_img.empty() || output_img.empty())
            {
                return 0;
            }
        
        cvtColor(output_img, hsv_img, COLOR_BGR2HSV);
        inRange(hsv_img, Scalar(hmin, smin, vmin), Scalar(hmax, smax, vmax), inRange_img);
        erode(inRange_img, inRange_img, struct_element1);
        findContours(inRange_img, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE);
        
        vector<Rect> boundRect(contours.size());//矩形逼近每个轮廓//
        for(int i=0; i < contours.size(); i++)
        {	
            boundRect[i] = boundingRect(Mat(contours[i]));
        }

        for(int i = 0; i < contours.size() ; i++)//遍历每个轮廓
        {
            min_scale = 1.9;
            min_num = -1;
           for(int j = 0; j < contours.size(); j++)//通过筛选挑出符合要求的轮廓//
           { 
               x_diff = fabs(boundRect[i].x - boundRect[j].x);
               rect_scale0 = (float)boundRect[i].height / boundRect[i].width;//每个轮廓的长宽比//
               rect_scale1 = (float)boundRect[j].height / boundRect[j].width;
               rect_scale3 = (float)boundRect[i].height / boundRect[j].height;//两个轮廓之间的长度比//
               cout << "rect_scale0 = " << rect_scale0 << endl;
               cout << "rect_scale1 = " << rect_scale1 << endl;                                 
               if(rect_scale3 <= 1)//小于1时取倒数//
               {
                   rect_scale3 = 1.0 / rect_scale3;
               }
               cout << "rect_scale3 = " << rect_scale3 << "############################" << endl;

               if( i!= j && x_diff >= 30 && rect_scale0 >= 1.0 && rect_scale1 >= 1.0
                     && contours[i].size() > 20 && contours[j].size() > 20 && rect_scale3 <= 1.3)
               {
                    // float area_scale = contourArea(contours[i]) / contourArea(contours[j]);
                    // if(area_scale <= 1)
                    // {
                    //     area_scale = 1.0 / area_scale;
                    // }

                //     if(area_scale <= 1000)
                //    {
                       ellipse0 = fitEllipse(contours[i]);//椭圆逼近获取每个轮廓中心和倾斜度//
                       ellipse1 = fitEllipse(contours[j]);
                       current_scale = ((ellipse0.angle >= 170 ? 180 - ellipse0.angle >= 170 : ellipse0.angle ) + 50) /
                       ((ellipse1.angle >= 170 ? 180 - ellipse1.angle >= 170 : ellipse1.angle) + 50);//两轮廓的倾斜度之比，加50来防止比例过大//
                       if(current_scale < 1)//比例小于1的话取倒数，方便设置阈值//
                       {
                           current_scale = 1.0 / current_scale;
                       }
                       if(current_scale <= min_scale)//循遍j个轮廓，找出倾斜度之比最小的//
                       {
                           min_scale = current_scale;
                           min_num = j;//记录下倾斜度比最小的轮廓的下角标//
                       }
                //    }
               }
           }
           cout << "min_scale = " << min_scale << endl << "---------------------------" << endl;
           if(min_num != -1)//min_num处值为-1，直到找到一个倾斜度比例小于阈值的轮廓才发生改变//
           {
                if(boundRect[i].x > boundRect[min_num].x)//挑选出两个轮廓最靠左上角的点//
                {
                    bigrect_x = boundRect[min_num].x;
                    bigrect_y = boundRect[min_num].y;
                }
                else
                {
                    bigrect_x = boundRect[i].x;
                    bigrect_y = boundRect[i].y;
                }
                width = abs(boundRect[i].x - boundRect[min_num].x) + boundRect[i].width;//计算装甲片的长和宽//
                height = boundRect[i].height;
                Rect bind_Rect(bigrect_x, bigrect_y, width, height);
                scale3 = (double)(width) / height;
                if(scale3 >= 1 && scale3 <= 20)
                {
                    Point2i center(bind_Rect.x + 0.5 * bind_Rect.width, bind_Rect.y + 0.5 * bind_Rect.height);
                    rect_scale1 = (float)boundRect[min_num].height / boundRect[min_num].width;
                    contours[min_num] = {{(5, 5)}};//把已经匹配的轮廓改变，以防重复匹配

                    find_corners(corners1, light_bar1, boundRect[min_num], ellipse1, src_img, contours, min_num);
                    find_corners(corners2, light_bar2, boundRect[i], ellipse0, src_img, contours, i);

                    circle(src_img, light_bar2[1], 4, Scalar(255, 255, 255));
                    circle(src_img, light_bar2[0], 4, Scalar(255, 255, 255));
                    circle(src_img, light_bar1[1], 4, Scalar(255, 255, 255));
                    circle(src_img, light_bar1[0], 4, Scalar(255, 255, 255));

                    line(src_img, light_bar1[0], light_bar2[1], Scalar(255, 255, 255));
                    line(src_img, light_bar1[1], light_bar2[0], Scalar(255, 255, 255));

                    Point2f center0;
                    center0 = center_point(light_bar1, light_bar2);
                    circle(src_img, center0, 4, Scalar(0, 255, 255));
                    rotated_matrix(light_bar1, light_bar2, rvec, tvec,
                    world_cor, camMatrix, distCoeff);

                    para.rx = theta_x;
                    para.ry = theta_y;
                    para.rz = theta_z;

                    para.x = tvec.at<double>(0,0);
                    para.y = tvec.at<double>(0,1);
                    para.z = tvec.at<double>(0,2);
                }
           }
        }
        // sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", src_img).toImageMsg();
        // pub.publish(msg);
        send.publish(para);
        imshow("src_video", src_img);
        waitKey(0);
    }
    return 0;
}


void find_corners(vector<Point2f> & corner, vector<Point2f> & light_bar, Rect &boundRect, RotatedRect &ellipse0, 
                    Mat &src_img, vector<vector<Point>> & contours, int con_num)
{
    int max, min;
    Rect rect_0, rect;
    Mat back_ground = Mat::zeros(src_img.size(), CV_8UC3);
    Mat mask = Mat::zeros(back_ground.size(), CV_8UC1);

    rect_0.x = boundRect.x - 20;
    rect_0.y = boundRect.y - 20;
    rect_0.width = boundRect.width + 40;
    rect_0.height = boundRect.height + 40;

    mask(rect_0).setTo(255);
    ellipse(back_ground, ellipse0, Scalar(255, 255, 255));
    //floodFill(back_ground, ellipse0.center, Scalar(255, 255, 255), &rect, Scalar(1, 1, 1), Scalar(1, 1, 1));
    drawContours(back_ground, contours, con_num, Scalar(255, 255, 255));
    cvtColor(back_ground, back_ground, COLOR_BGR2GRAY);
    goodFeaturesToTrack(back_ground, corner, 10, 0.01, 10, mask, 3, false, 0.04);
    max = max_y(corner);
    min = min_y(corner);

    light_bar[0].x =  corner[max].x;
    light_bar[0].y =  corner[max].y;
    light_bar[1].x =  corner[min].x;
    light_bar[1].y =  corner[min].y;
    imshow("a", back_ground);
}

int min_y(vector<Point2f> & corner)
{
    int min = 0;
    int min_0 = corner[0].y;
    for( size_t i = 0; i < corner.size(); i++ )
        {   
            if(corner[i].y < min_0)
                {
                    min = i ;
                    min_0 = corner[i].y;
                }  
        } 
    return min;
}

int max_y(vector<Point2f> & corner)
{
    int max = 0;
    int max_0 = corner[0].y;
    for( size_t i = 0; i < corner.size(); i++ )
        {   
            if(corner[i].y > max_0)
                {
                    max = i ;
                    max_0 = corner[i].y;
                }  
        } 
    return max;
}

Point center_point(vector<Point2f> &light_bar1, vector<Point2f> &light_bar2) 
{
	int x, y;
	int X1 = light_bar1[0].x - light_bar2[1].x, Y1 = light_bar1[0].y - light_bar2[1].y, X2 = 
                light_bar1[1].x - light_bar2[0].x, Y2 = light_bar1[1].y - light_bar2[0].y;

	if (X1*Y2 == X2*Y1)
        {
            return Point((light_bar1[1].x - light_bar2[0].x)/2,(light_bar1[1].y - light_bar2[0].y)/2);;
        } 

	int A = X1*light_bar1[0].y - Y1*light_bar1[0].x,B= X2*light_bar1[1].y - Y2*light_bar1[1].x;
	y = (A*Y2 - B*Y1) / (X1*Y2 - X2*Y1);
	x = (B*X1-A*X2) / (Y1*X2 - Y2*X1);

	return Point(x, y);
}

void rotated_matrix(vector<Point2f> &light_bar1, vector<Point2f> &light_bar2, Mat &rvec, Mat &tvec,
                    vector<Point3f> &world_cor, Mat &camMatrix, Mat &distCeoff)
{
    vector<Point2f> light_lef, light_rig;
    if(light_bar1[0].x < light_bar2[0].x)
    {
        light_lef = light_bar1;
        light_rig = light_bar2;
    }
    else
    {
        light_lef = light_bar2;
        light_rig = light_bar2;
    }
    
    vector<Point2f> point;
    point.clear();
    point.push_back(light_bar1[0]);
    point.push_back(light_bar1[1]);
    point.push_back(light_bar2[1]);
    point.push_back(light_bar2[0]);
    solvePnP(world_cor, point, camMatrix, distCeoff, rvec, tvec);
    Rodrigues(rvec, rvec);
    theta_y = atan2(rvec.at<double>(2, 1), rvec.at<double>(2, 2));
    theta_x = atan2(-rvec.at<double>(2, 0),
    sqrt(rvec.at<double>(2, 1)*rvec.at<double>(2, 1) + rvec.at<double>(2, 2)*rvec.at<double>(2, 2)));
    theta_z = atan2(rvec.at<double>(1, 0), rvec.at<double>(0, 0));
}
/*************************************************
Copyright:     Copyright 2018 Ares
Author:        Lcy
Date:          2018-11-19
Description:   装甲识别
Version:       1.0.6
History:       1.Date:2019-01-07
                 Author:daxia
                 Modification:加入了装甲版测距
               2.Date:2019-01-10
                 Author:daxia Lcy
                 Modification:解决了白色灯光的影响,已经将45度的问题已经解决
               3.Date:2019-01-12
                 Author:Lcy
                 Modification:加入了初步的运动估计
               4.Date:2019-01-13
                 Author:daxia
                 Modification:加入了弹道补偿
               5.Date:2019-01-14
                 Author:Lcy
                 Modification:加入了PID控制器
               6.Date:2019-01-18
                 Author:Lcy
                 Modification:增加了PID显示,修改运动预测；
**************************************************/
#include <iostream>
#include <pthread.h>
#include <math.h>
#include "armor_plate.h"
#include "img_process.h"
#include "opencv2/video/tracking.hpp"
#include "pid_cplus.h"
#include "Common_Header.h"
using namespace std;
using namespace cv;

const double PI = 3.1415926;

ImgProcess process;

// 用于solve pnp

double K[3][3] =
{
    172.377083, 0, 34.170591,
    0, 1720.52355, 223.71411480,
    0, 0, 1
};
double D[1][5] =
{
    -0.53867, 1.90497, 0.037784, 0.02694, -40.05843
};
cv::Mat K_(3, 3, CV_64FC1, K);
cv::Mat D_(1, 5, CV_64FC1, D);


// 小装甲世界坐标
float corners[4][2] =
{
    0.0, 0.0,
    0.0, 0.055,
    0.13, 0.055,
    0.13, 0.0,
};
// 大装甲世界坐标
float corners_big[4][2] =
{
    0.0, 0.0,
    0.0, 0.0275,
    0.0, 0.055,
    0.225, 0.0,
};
/*************************************************
Function:       ArmorPlate
Description:    构造函数
Input:
Output:
Return:
Others:
*************************************************/
ArmorPlate::ArmorPlate()
{
	our_team_ = OURTEAM;
	p_kp_ = 0.4;
	p_ki_ = 0.0;
	p_kd_ = 0;
    y_kp_ = 0.4;//哨兵y轴p值为1.6,把运动预测去掉；
    y_ki_ = 0;
    y_kd_ = 0;
    kp=0,4;
    error=10;
    Pitch_pid.PID_init(&Pitch_pid,true);
    Yaw_pid.PID_init(&Yaw_pid,false);

}

/*************************************************
Function:       CameraInit
Description:    摄像头初始化
Input:          device
Output:
Return:         false or true
Others:         bool
*************************************************/
bool ArmorPlate::CameraInit(int device)
{
	capture_armor_.open(device);
	if (!capture_armor_.isOpened())
	{
		printf("视觉辅助像头打开失败！\n");
		return false;

	}
	else
	{
	    // 设置摄像头参数
        capture_armor_.set(CAP_PROP_AUTO_EXPOSURE, 0.25); // where 0.25 means "manual exposure, manual iris"
        //capture_armor_.set(CV_CAP_PROP_EXPOSURE, 0.023  );
	    capture_armor_.set(CV_CAP_PROP_EXPOSURE, 0.0018); // 曝光//0.0007
        capture_armor_.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));
        capture_armor_.set(CV_CAP_PROP_FRAME_WIDTH,CMR_WIDTH);
        capture_armor_.set(CV_CAP_PROP_FRAME_HEIGHT,CMR_HEIGHT);
	    //cout << CV_CAP_PROP_FRAME_WIDTH << endl;
        //cout <<CV_CAP_PROP_FRAME_HEIGHT<< endl;
		return true;
	}
}
/*************************************************
Function:       Init_cap_V4()
Description:    摄像头初始化
Input:          device
Output:
Return:         false or true
Others:         bool
*************************************************/

bool ArmorPlate::Init_cap_V4()
{

    cap.Init("/dev/video0",1);
    cap.setVideoFormat(640,480,1);
    cap.info();
    //cap.setVideoFPS(30);
    cap.setExposureTime(0,5);
    cap.startStream();

}
/*************************************************
Function:       ImgPreProcess
Description:    图像预处理
Input:          src
Output:         dst
Return:
Others:         HSV
*************************************************/

void ArmorPlate::ImgPreProcess(const cv::Mat& src, cv::Mat& dst)
{
	Mat imgHSV, HSV_RIO;
	Mat image, hist_out;
	vector<Mat> channels_HSV;
	channels_HSV.clear();
	dst = 0;
	Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
	//hsv
	cvtColor(src, imgHSV, COLOR_BGR2HSV); // 转换 BGR to HSV
	split(imgHSV, channels_HSV); // 通道分离
	if (our_team_) // 我方红方 对方蓝方
	{
		threshold(channels_HSV[2], channels_HSV[2], 200, 255, THRESH_BINARY); // 设置阈值 最好设为动态阈值
		morphologyEx(channels_HSV[2], channels_HSV[2], MORPH_DILATE, element);  // 形态学操作 膨胀
		imgHSV.copyTo(image, channels_HSV[2]); // 拷贝
		inRange(image, Scalar(BLowH, BLowS, BLowV), Scalar(BHighH, BHighS, BHighV), dst); // Threshold the image返回的是二值化图像
	}
	else // 我方蓝方 对方红方
	{
		threshold(channels_HSV[2], channels_HSV[2], 150, 255, THRESH_BINARY); // 设置阈值 最好设为动态阈值
		morphologyEx(channels_HSV[2], channels_HSV[2], MORPH_DILATE, element); // 形态学操作 膨胀
		imgHSV.copyTo(image, channels_HSV[2]); // 拷贝
		inRange(image, Scalar(RLowH, RLowS, RLowV), Scalar(RHighH, RHighS, RHighV), hist_out); // Threshold the image返回的是二值化图像
		inRange(image, Scalar(RLowH_2, RLowS, RLowV), Scalar(RHighH_2, RHighS, RHighV), HSV_RIO);
		dst = hist_out | HSV_RIO;
	}
	//imshow("HSV", dst);
	morphologyEx(dst, dst, MORPH_DILATE, element, Point(-1, -1), 2); // 形态学操作 膨胀 2次
	imshow("预处理HSV", dst);
	waitKey(1);
}

/*************************************************
Function:       ImgPreProcess
Description:    图像预处理
Input:          src
Output:         dst
Return:
Others:         RGB 已解决：考虑设置一个阈值，完全去除白色影响
                遗留问题存在取舍问题就是说在装甲偏离的45度时候，我们
                去除白色的算法的选值现在为3.5，现在到了斜角度的时候存在
                虚影。
*************************************************/

//void ArmorPlate::ImgPreProcess(const cv::Mat& src, cv::Mat& dst)
//{
//    std::vector<Mat> img_channels; // 通道
//    split(src, img_channels); // 通道分离
//     imshow("yuantu",src);
//    if (our_team_) // 我方红方 对方蓝方
//    {
//        Mat img_blue_channels;
//        img_blue_channels = img_channels.at(0); // 蓝色分量
//        //imshow("blue_channels", img_blue_channels);
//        img_blue_channels = img_blue_channels - img_channels.at(1)*0.4 - img_channels.at(2)*0.4; // 去除白色影响
//        blur(img_blue_channels, img_blue_channels, Size(3, 3));
//        img_blue_channels = img_blue_channels*2; // 增强对比度
//        double maxValue_gray;
//        minMaxLoc(img_blue_channels, 0, &maxValue_gray, 0, 0); // 最大最小
//        if(maxValue_gray<150)
//            maxValue_gray=180;
//
//        Mat imgBin;
//        threshold(img_blue_channels, imgBin, maxValue_gray*0.6, 255, THRESH_BINARY); // 入口
//        cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)); // 构造核函数 getStructuringElement函数会返回指定形状和尺寸的结构元素
//        cv::dilate(imgBin, dst, element, Point(-1, -1), 3); // 膨胀(dilate) 局部最大值(浅色)扩大3圈
//        cv::erode(dst, dst, element, Point(-1, -1), 5); // 腐蚀(erode) 局部最小值(深色)扩大5圈
//        cv::dilate(dst, dst, element, Point(-1, -1), 3); // 膨胀(dilate) 局部最大值(浅色)扩大5圈
//        imshow("预处理RGB", dst);
//        //waitKey(1);
//    }
//    else // 我方蓝方 对方红方
//    {
//        Mat img_red_channels;
//        img_red_channels = img_channels.at(2); // 红色分量
//        imshow("red_channels", img_red_channels);
//        Mat clear_shadow = img_red_channels.clone();
//        Mat blue = img_channels.at(0).clone();
//        Mat green = img_channels.at(1).clone();
//        blur(blue, blue, Size(3, 3));
//        //blur(green, green, Size(3, 3));
//        img_red_channels = img_red_channels - (img_channels.at(0)*0.35 + img_channels.at(1)*0.35); // 去除白色影响
//        inRange(clear_shadow, 120, 255, clear_shadow);
//        inRange(blue, 0, 150, blue);
//        inRange(green, 0, 160, green);
//        blur(img_red_channels, img_red_channels, Size(3, 3));
//        //img_red_channels = img_red_channels*2; // 增强对比度
//        /*double maxValue_gray;
//        minMaxLoc(img_red_channels, 0, &maxValue_gray, 0, 0); // 最大最小
//        cout<<maxValue_gray<<endl;
//        if(maxValue_gray<180)
//            maxValue_gray=260;
//        */
//        Mat imgBin;
//        threshold(img_red_channels, imgBin, 150, 255, THRESH_BINARY); // 入口
//        bitwise_and(green, clear_shadow, imgBin);
//        bitwise_and(imgBin, blue, imgBin);
//        //imshow("二值化",imgBin);
//        cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(3, 3)); // 构造核函数 getStructuringElement函数会返回指定形状和尺寸的结构元素
//        cv::dilate(imgBin, dst, element, Point(-1, -1), 2); // 膨胀(dilate) 局部最大值(浅色)扩大3圈
//        cv::erode(dst, dst, element, Point(-1, -1), 2); // 腐蚀(erode) 局部最小值(深色)扩大5圈
//        cv::dilate(dst, dst, element, Point(-1, -1), 2); // 膨胀(dilate) 局部最大值(浅色)扩大5圈
//        imshow("预处理RGB", dst);
//        //waitKey(1);
//    }
//}

//void ArmorPlate::ImgPreProcess(const cv::Mat& src, cv::Mat& dst)
//{
//    std::vector<Mat> img_channels; // 通道
//    split(src, img_channels); // 通道分离
//    imshow("yuantu",src);
//    if (our_team_) // 我方红方 对方蓝方
//    {
//        Mat img_blue_channels;
//        img_blue_channels = img_channels.at(0); // 蓝色分量
//        //imshow("blue_channels", img_blue_channels);
//        img_blue_channels = img_blue_channels - img_channels.at(1)*0.4 - img_channels.at(2)*0.4; // 去除白色影响
//        blur(img_blue_channels, img_blue_channels, Size(3, 3));
//        img_blue_channels = img_blue_channels*2; // 增强对比度
//        double maxValue_gray;
//        minMaxLoc(img_blue_channels, 0, &maxValue_gray, 0, 0); // 最大最小
//        if(maxValue_gray<150)
//            maxValue_gray=180;
//
//        Mat imgBin;
//        threshold(img_blue_channels, imgBin, maxValue_gray*0.6, 255, THRESH_BINARY); // 入口
//        cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)); // 构造核函数 getStructuringElement函数会返回指定形状和尺寸的结构元素
//        cv::dilate(imgBin, dst, element, Point(-1, -1), 3); // 膨胀(dilate) 局部最大值(浅色)扩大3圈
//        cv::erode(dst, dst, element, Point(-1, -1), 5); // 腐蚀(erode) 局部最小值(深色)扩大5圈
//        cv::dilate(dst, dst, element, Point(-1, -1), 3); // 膨胀(dilate) 局部最大值(浅色)扩大5圈
//        imshow("预处理RGB", dst);
//        //waitKey(1);
//    }
//    else // 我方蓝方 对方红方
//    {
//        Mat img_red_channels;
//        img_red_channels = img_channels.at(2); // 红色分量
//        imshow("red_channels", img_red_channels);
//        Mat clear_shadow = img_red_channels.clone();
//        Mat blue = img_channels.at(0).clone();
//        Mat green = img_channels.at(1).clone();
//        blur(blue, blue, Size(3, 3));
//        //blur(green, green, Size(3, 3));
//        //img_red_channels = img_red_channels - (img_channels.at(0)*0.35 + img_channels.at(1)*0.35); // 去除白色影响
//        inRange(clear_shadow, 120, 255, clear_shadow);
//        inRange(blue, 0, 150, blue);
//        inRange(green, 0, 160, green);
//        //blur(img_red_channels, img_red_channels, Size(3, 3));
//        //img_red_channels = img_red_channels*2; // 增强对比度
//        /*double maxValue_gray;
//        minMaxLoc(img_red_channels, 0, &maxValue_gray, 0, 0); // 最大最小
//        cout<<maxValue_gray<<endl;
//        if(maxValue_gray<180)
//            maxValue_gray=260;
//        */
//        Mat imgBin;
//        //threshold(img_red_channels, imgBin, 150, 255, THRESH_BINARY); // 入口
//        bitwise_and(green, clear_shadow, imgBin);
//        bitwise_and(imgBin, blue, imgBin);
//        //imshow("二值化",imgBin);
//        cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(3, 3)); // 构造核函数 getStructuringElement函数会返回指定形状和尺寸的结构元素
//        cv::dilate(imgBin, dst, element, Point(-1, -1), 2); // 膨胀(dilate) 局部最大值(浅色)扩大3圈
//        cv::erode(dst, dst, element, Point(-1, -1), 2); // 腐蚀(erode) 局部最小值(深色)扩大5圈
//        cv::dilate(dst, dst, element, Point(-1, -1), 3); // 膨胀(dilate) 局部最大值(浅色)扩大5圈
//        imshow("预处理RGB", dst);
//        //waitKey(1);
//    }
//}

/*************************************************
Function:       FindArmor
Description:    定位装甲版
Input:          src dst
Output:         视野目标 击打目标
Return:
Others:
*************************************************/
void ArmorPlate::FindArmor(cv::Mat& src, cv::Mat& dst, std::vector<cv::RotatedRect>& all, cv::RotatedRect& target)
{
    all.clear();
    target.center.x = 0;
    target.center.y = 0;
    target.size.width = 0;
    target.size.height = 0;
    target.angle = 0;

    cv::RotatedRect s, s_fitEllipse, s_minAreaRect; // 用于筛选轮廓
    std::vector<cv::RotatedRect> ss; // 筛选完存放在这里
    ss.clear();

    vector<vector<Point>> contours; // 轮廓
    vector<Vec4i> hierarchy; // 层次

    findContours(dst, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE); // 寻找轮廓
    Mat drawing = Mat::zeros(dst.size(), CV_8UC3);//初始化就是将根据原来的方式进行
    RNG g_rng(12345);
    // 筛选轮廓，保存到集合里
    for (int i = 0; i < contours.size(); i++)
    {
        Scalar color = Scalar(g_rng.uniform(0, 255), g_rng.uniform(0, 255), g_rng.uniform(0, 255)); // 任意值
        drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, Point()); // 画出所有轮廓

        if (contours[i].size() >= 10) //
        {
            s_fitEllipse = fitEllipse(contours[i]); // 二维点集的椭圆拟合，用椭圆将二维点包含起来  点要大于6个
            s_minAreaRect = minAreaRect(contours[i]); // 最小旋转矩

            s.angle = s_fitEllipse.angle;
            s.center = s_fitEllipse.center;
            if (s_minAreaRect.size.width > s_minAreaRect.size.height)
            {
                s.size.height = s_minAreaRect.size.width;
                s.size.width = s_minAreaRect.size.height;
            }
            else
            {
                s.size.height = s_minAreaRect.size.height;
                s.size.width = s_minAreaRect.size.width;//由于minAreaRect这个函数返回的角度不区分长和宽的。
            }
            //限幅
            if ((s.size.width / s.size.height) > L_WH_RAT_MIN && (s.size.width / s.size.height) < L_WH_RAT_MAX)
                continue;
            int x = s.center.x - s.size.width;
            if (x < 0)
                continue;
            int y = s.center.y - s.size.height;
            if (y < 0)
                continue;
            int w = s.size.width + s.size.width;
            if (w > dst.cols - x)
                continue;
            int h = s.size.height + s.size.width;
            if (h > dst.rows - y)
                continue;
            if ((s.angle < 45 || s.angle > 135) )
            //if ((s.angle < 45 || s.angle>135) && (s.size.height > 10) && (s.size.height < 150)) // 通过角度和高度来判断
            {
                //cout << s.angle << endl;
                /*Point2f ppt[4];
                s.points(ppt); // 计算二维盒子顶点
                line(src, ppt[0], ppt[1], CV_RGB(0, 255, 0), 1, 8, 0);*/
                ss.push_back(s);
            }
        }

    }
    //imshow("all", drawing); // 显示所有找到的轮廓

    //判别装甲
    std::vector<RotatedRect> armors;
    vector<Armor_builded> armor_SECOND;//
    Armor_builded armor_FIRST; //定义装甲区域的旋转矩形
    float armor_center_x;
    float armor_center_y;

    armors.clear();
    armor_SECOND.clear();
    int nL, nW;

    if (ss.size() < 2) //如果检测到的旋转矩形个数小于2，则直接返回
    {
        target.center.x = 0;
        target.center.y = 0;
        target.size.width = 0;
        target.size.height = 0;
        target.angle = 0;
        all.push_back(target);
        //存储上一次装甲中心点
        //armor_center_x = 0;
        //armor_center_y = 0;
        //cout << "1" << endl;
        //return;
    }
    else
    {
        for (int i = 0; i < ss.size() - 1; i++) // 求任意两个旋转矩形的夹角
        {
            for (int j = i + 1; j < ss.size(); j++)
            {
                double ssiH = max(ss[i].size.height, ss[i].size.width);
                double ssiW = min(ss[i].size.height, ss[i].size.width);
                double ssjH = max(ss[j].size.height, ss[i].size.width);
                double ssjW = min(ss[j].size.height, ss[i].size.width);
                /*double height_diff = abs(ss[i].size.height - ss[j].size.height); // 高度差
                double height_sum = ss[i].size.height + ss[j].size.height; // 高度和
                double width_diff = abs(ss[i].size.width - ss[j].size.width); // 宽度差
                double width_sum = ss[i].size.width + ss[j].size.width; // 宽度和
                double angle_diff = fabs(ss[i].angle - ss[j].angle); // 角度差
                double Y_diff = abs(ss[i].center.y - ss[j].center.y); // 纵坐标差值
                double X_diff = abs(ss[i].center.x - ss[j].center.x); // 横坐标差值
                double MH_diff = (min(ss[i].size.height, ss[j].size.height)) * 2 / 3; // 高度差限幅
                double height_min=(min(ss[i].size.width,ss[j].size.width));
                double height_max = (max(ss[i].size.height, ss[j].size.height)); // 最大高度
                */
                double height_diff = fabs(ssiH - ssjH); // 高度差
                double height_sum = ssiH + ssjH; // 高度和
                double width_diff = fabs(ssiW - ssjW); // 宽度差
                double width_sum = ssiW + ssjW; // 宽度和
                double angle_diff = fabs(ss[i].angle - ss[j].angle); // 角度差
                double Y_diff = fabs(ss[i].center.y - ss[j].center.y); // 纵坐标差值
                double X_diff = fabs(ss[i].center.x - ss[j].center.x); // 横坐标差值
                double MH_diff = (min(ssiH, ssjH)) * 2 / 3; // 高度差限幅
                double height_min = (min(ssiH, ssjH));
                double height_max = (max(ssiH, ssjH)); // 最大高度
                //cout <<height_max << "  " << height_min <<endl;
                height_max = height_max * 0.3 + height_min * 0.7;
                double height_low = height_max * 0.6 + height_min * 0.4;
                //cout <<(Y_diff < MH_diff)<< "  " << (X_diff < height_max * 4.5) << "  "<< ((angle_diff < T_ANGLE_THRE) || (180 - angle_diff < T_ANGLE_THRE180)) << "  " << (height_diff / height_sum < T_HIGH_RAT) << "  " << (width_diff / width_sum < T_WHIDTH_RAT) << endl;
                //cout << (angle_diff ) << "  " << (180 - angle_diff ) << endl;
                //cout << ss[i].angle << " t " << ss[j].angle << endl;
                if (Y_diff < MH_diff && X_diff < height_max * 4.5 &&  X_diff > height_low * 1.5 &&
                    ((angle_diff < T_ANGLE_THRE) || (180 - angle_diff < T_ANGLE_THRE180)) &&
                        (height_diff / height_sum < T_HIGH_RAT) &&
                        (width_diff / width_sum < T_WHIDTH_RAT) &&
                        (min(ssiH, ssjH)/max(ssiH, ssjH) > 0.55)) // 还可以加入高度差限制
                {
                    //cout << "1OK" << endl;
                    armor_FIRST.armorS.center.x = ((ss[i].center.x + ss[j].center.x) / 2); //装甲中心的x坐标
                    armor_FIRST.armorS.center.y = ((ss[i].center.y + ss[j].center.y) / 2); //装甲中心的y坐标
                    armor_FIRST.armorS.angle = (ss[i].angle + ss[j].angle) / 2;   //装甲所在旋转矩形的旋转角度
                    if (180 - angle_diff < T_ANGLE_THRE180)
                        armor_FIRST.armorS.angle += 90;
                    nL = (ss[i].size.height + ss[j].size.height) / 2; //装甲的高度
                    nW = sqrt((ss[i].center.x - ss[j].center.x) * (ss[i].center.x - ss[j].center.x) + (ss[i].center.y - ss[j].center.y) * (ss[i].center.y - ss[j].center.y)); // 装甲的宽度等于两侧LED所在旋转矩形中心坐标的距离
                    if (nL < nW)
                    {
                        armor_FIRST.armorS.size.height = nL;
                        armor_FIRST.armorS.size.width = nW;
                    }
                    else
                    {
                        armor_FIRST.armorS.size.height = nW;
                        armor_FIRST.armorS.size.width = nL;
                    }
                    if (Y_diff < nW / 3)
                    {
                        armor_FIRST.build1_No = i;
                        armor_FIRST.build2_No = j;
                        armor_FIRST.build_features[0] = angle_diff;
                        armor_FIRST.build_features[1] = Y_diff;
                        armor_FIRST.build_features[2] = height_diff;
                        armor_FIRST.build_features[3] = width_diff;
                        armor_SECOND.push_back(armor_FIRST); //将找出的装甲的旋转矩形保存到vector
                    }
                }
                else if ((angle_diff < T_ANGLE_THREMIN || 180 - angle_diff < T_ANGLE_THRE180MIN) &&
                         Y_diff < MH_diff * 3 / 2 && X_diff < height_max * 4.5 && X_diff > height_low * 1.5 &&
                         height_diff / height_sum < T_HIGH_RAT_ANGLE &&
                         width_diff / width_sum < T_WHIDTH_RAT_ANGLE &&
                         (min(ssiH, ssjH)/max(ssiH, ssjH) > 0.55)
                        )
                {
                    //cout << "2OK" << endl;
                    armor_FIRST.armorS.center.x = ((ss[i].center.x + ss[j].center.x) / 2); //装甲中心的x坐标
                    armor_FIRST.armorS.center.y = ((ss[i].center.y + ss[j].center.y) / 2); //装甲中心的y坐标
                    armor_FIRST.armorS.angle = (ss[i].angle + ss[j].angle) / 2;   //装甲所在旋转矩形的旋转角度
                    if (180 - angle_diff < T_ANGLE_THRE180)
                        armor_FIRST.armorS.angle += 90;
                    nL = (ss[i].size.height + ss[j].size.height) / 2; //装甲的高度
                    nW = sqrt((ss[i].center.x - ss[j].center.x) * (ss[i].center.x - ss[j].center.x) + (ss[i].center.y - ss[j].center.y) * (ss[i].center.y - ss[j].center.y)); // 装甲的宽度等于两侧LED所在旋转矩形中心坐标的距离
                    if (nL < nW)
                    {
                        armor_FIRST.armorS.size.height = nL;
                        armor_FIRST.armorS.size.width = nW;
                    }
                    else
                    {
                        armor_FIRST.armorS.size.height = nW;
                        armor_FIRST.armorS.size.width = nL;
                    }
                    if (Y_diff < nW / 2)
                    {
                        armor_FIRST.build1_No = i;
                        armor_FIRST.build2_No = j;
                        armor_FIRST.build_features[0] = angle_diff;
                        armor_FIRST.build_features[1] = Y_diff;
                        armor_FIRST.build_features[2] = height_diff;
                        armor_FIRST.build_features[3] = width_diff;
                        armor_SECOND.push_back(armor_FIRST); //将找出的装甲的旋转矩形保存到vector
                    }
                }
                else if ((angle_diff < 3 || 180 - angle_diff < 2) &&
                         Y_diff < MH_diff * 2 && X_diff < height_max * 4
                         && X_diff > height_low * 1.5 && (min(ssiH, ssjH)/max(ssiH, ssjH) > 0.55)
                    //height_diff / height_sum < T_HIGH_RAT_ANGLE
                        )
                {
                    //cout << "3OK" << endl;
                    armor_FIRST.armorS.center.x = ((ss[i].center.x + ss[j].center.x) / 2); //装甲中心的x坐标
                    armor_FIRST.armorS.center.y = ((ss[i].center.y + ss[j].center.y) / 2); //装甲中心的y坐标
                    armor_FIRST.armorS.angle = (ss[i].angle + ss[j].angle) / 2;   //装甲所在旋转矩形的旋转角度
                    if (180 - angle_diff < T_ANGLE_THRE180)
                        armor_FIRST.armorS.angle += 90;
                    nL = (ss[i].size.height + ss[j].size.height) / 2; //装甲的高度
                    nW = sqrt((ss[i].center.x - ss[j].center.x) * (ss[i].center.x - ss[j].center.x) + (ss[i].center.y - ss[j].center.y) * (ss[i].center.y - ss[j].center.y)); // 装甲的宽度等于两侧LED所在旋转矩形中心坐标的距离
                    if (nL < nW)
                    {
                        armor_FIRST.armorS.size.height = nL;
                        armor_FIRST.armorS.size.width = nW;
                    }
                    else
                    {
                        armor_FIRST.armorS.size.height = nW;
                        armor_FIRST.armorS.size.width = nL;
                    }
                    if ((abs(ss[i].center.y - ss[j].center.y) < nW / 2))
                    {
                        armor_FIRST.build1_No = i;
                        armor_FIRST.build2_No = j;
                        armor_FIRST.build_features[0] = angle_diff;
                        armor_FIRST.build_features[1] = Y_diff;
                        armor_FIRST.build_features[2] = height_diff;
                        armor_FIRST.build_features[3] = width_diff;
                        armor_SECOND.push_back(armor_FIRST); //将找出的装甲的旋转矩形保存到vector
                    }
                }
                else if ((angle_diff < 3 || 180 - angle_diff < 2) &&
                         Y_diff < MH_diff * 3 && X_diff < height_max * 5
                         && X_diff > height_low * 1.5 && (min(ssiH, ssjH)/max(ssiH, ssjH) > 0.55)
                    //height_diff / height_sum < T_HIGH_RAT_ANGLE
                        )
                {
                    //cout << "4OK" << endl;
                    armor_FIRST.armorS.center.x = ((ss[i].center.x + ss[j].center.x) / 2); //装甲中心的x坐标
                    armor_FIRST.armorS.center.y = ((ss[i].center.y + ss[j].center.y) / 2); //装甲中心的y坐标
                    armor_FIRST.armorS.angle = (ss[i].angle + ss[j].angle) / 2;   //装甲所在旋转矩形的旋转角度
                    if (180 - angle_diff < T_ANGLE_THRE180)
                        armor_FIRST.armorS.angle += 90;
                    nL = (ss[i].size.height + ss[j].size.height) / 2; //装甲的高度
                    nW = sqrt((ss[i].center.x - ss[j].center.x) * (ss[i].center.x - ss[j].center.x) + (ss[i].center.y - ss[j].center.y) * (ss[i].center.y - ss[j].center.y)); //装甲的宽度等于两侧LED所在旋转矩形中心坐标的距离
                    if (nL < nW)
                    {
                        armor_FIRST.armorS.size.height = nL;
                        armor_FIRST.armorS.size.width = nW;
                    }
                    else
                    {
                        armor_FIRST.armorS.size.height = nW;
                        armor_FIRST.armorS.size.width = nL;
                    }
                    if (Y_diff < nW / 2)
                    {
                        armor_FIRST.build1_No = i;
                        armor_FIRST.build2_No = j;
                        armor_FIRST.build_features[0] = angle_diff;
                        armor_FIRST.build_features[1] = Y_diff;
                        armor_FIRST.build_features[2] = height_diff;
                        armor_FIRST.build_features[3] = width_diff;
                        armor_SECOND.push_back(armor_FIRST); //将找出的装甲的旋转矩形保存到vector
                    }
                }
            }
        }

        if (armor_SECOND.size() < 1)
        {
            int ss_width = 0;
            int ss_ID = 0;
            for (unsigned int SSS = 0; SSS < ss.size(); SSS++) //求任意两个旋转矩形的夹角
            {
                if (ss[SSS].size.width > ss_width && (ss[SSS].size.width / ss[SSS].size.height) < 0.4 && (ss[SSS].size.width / ss[SSS].size.height) > 0.15)
                {
                    ss_width = ss[SSS].size.width;
                    ss_ID = SSS;
                }

            }
            int WIDTH = 3 * ss[ss_ID].size.height;
            int HEIGHT = 3 * ss[ss_ID].size.height;
            int XX_RIGHT = ss[ss_ID].center.x;
            int XX_LEFT = ss[ss_ID].center.x - WIDTH;
            int YY = ss[ss_ID].center.y - ss[ss_ID].size.height * 3 / 2;

            if (XX_RIGHT + WIDTH > CMR_WIDTH)
            {
                WIDTH = CMR_WIDTH - XX_RIGHT;
            }
            if (XX_RIGHT < 0)
            {
                XX_RIGHT = 0;
            }
            if (XX_LEFT < 0)
            {
                XX_LEFT = 0;
            }
            if (XX_LEFT + WIDTH > CMR_WIDTH)
            {
                WIDTH =CMR_WIDTH - XX_LEFT;
            }
            if (YY + HEIGHT > CMR_HEIGHT)
            {
                HEIGHT = CMR_HEIGHT - YY;
            }
            if (YY < 0)
            {
                YY = 0;
            }

            if (ss[ss_ID].angle > 45)
            {

                Mat LEFT_rio = src(Rect(XX_RIGHT, YY, WIDTH, HEIGHT));
                Mat  Rio_out, Rio_out1;

                cvtColor(LEFT_rio, Rio_out, COLOR_BGR2GRAY);//转化为灰度图像

                equalizeHist(Rio_out, Rio_out);

                imshow("rio", Rio_out);
                GaussianBlur(Rio_out, Rio_out, Size(3, 3), 0, 0);//高斯滤波
                threshold(Rio_out, Rio_out1, 0, 255, THRESH_OTSU);//二值化
                imshow("RIO_二值化", Rio_out1);

                vector<vector<Point>> last_contours;
                vector<Vec4i>last_hierarchy;
                cv::RotatedRect s_center;
                std::vector<cv::RotatedRect> ss_center;
                ss_center.clear();
                findContours(Rio_out1, last_contours, last_hierarchy, RETR_LIST, CHAIN_APPROX_NONE);
                //drawContours(drawing_out, last_contours, -1, color, 1);//可以省去
                int	centerX_num = 0;

                for (size_t ii = 0; ii < last_contours.size(); ii++)
                {
                    if (last_contours[ii].size() > 10)
                    {
                        s_center = minAreaRect(last_contours[ii]);
                        ss_center.push_back(s_center);
                    }
                }
                for (size_t iii = 0; iii < ss_center.size() - 1; iii++)
                {
                    if (ss_center[iii].size.height > ss[ss_ID].size.height / 2 && ss_center[iii].size.width / ss_center[iii].size.width > 0.5)
                    {
                        for (size_t jjj = iii + 1; jjj < ss_center.size(); jjj++)
                        {
                            int centerX_sum = abs(ss_center[iii].center.x - ss_center[jjj].center.x);
                            int centerY_sum = abs(ss_center[iii].center.y - ss_center[jjj].center.y);
                            if (centerX_sum < 5 && centerY_sum < 5)
                                centerX_num++;
                        }
                    }
                }
                if (centerX_num > 3)
                {
                    ss[ss_ID].center.x = ss[ss_ID].center.x + cos((double)(180.0 - ss[ss_ID].angle) / 180.0*PI)*ss[ss_ID].size.height;
                    ss[ss_ID].center.y = ss[ss_ID].center.y - sin((double)(180.0 - ss[ss_ID].angle) / 180.0*PI)*ss[ss_ID].size.height;

                    target.center = ss[ss_ID].center;
                    target.size.width = ss[ss_ID].size.height * 2;
                    target.size.height = ss[ss_ID].size.height;

                    target.angle = ss[ss_ID].angle;

                    all.push_back(ss[ss_ID]);
                }
            }
            else
            {
                Mat LEFT_rio = src(Rect(XX_LEFT, YY, WIDTH, HEIGHT));
                Mat  Rio_out, Rio_out1;

                cvtColor(LEFT_rio, Rio_out, COLOR_BGR2GRAY);//转化为灰度图像

                equalizeHist(Rio_out, Rio_out);
                imshow("rio", Rio_out);
                GaussianBlur(Rio_out, Rio_out, Size(3, 3), 0, 0);//高斯滤波

                threshold(Rio_out, Rio_out1, 0, 255, THRESH_OTSU);//二值化
                imshow("RIO_二值化", Rio_out1);

                vector<vector<Point>> last_contours;
                vector<Vec4i>last_hierarchy;
                cv::RotatedRect s_center;
                std::vector<cv::RotatedRect> ss_center;
                ss_center.clear();
                findContours(Rio_out1, last_contours, last_hierarchy, RETR_LIST, CHAIN_APPROX_NONE);
                //drawContours(drawing_out, last_contours, -1, color, 1);//可以省去
                int	centerX_num = 0;

                for (size_t ii = 0; ii < last_contours.size(); ii++)
                {
                    if (last_contours[ii].size() > 10)
                    {
                        s_center = minAreaRect(last_contours[ii]);
                        ss_center.push_back(s_center);
                    }
                }
                //int SIze = ss_center.size();
                for (int iii = 0; iii < ss_center.size(); iii++)
                {
                    if (ss_center[iii].size.height > ss[ss_ID].size.height / 2 && ss_center[iii].size.width / ss_center[iii].size.width > 0.5)
                    {
                        for (size_t jjj = iii + 1; jjj < ss_center.size(); jjj++)
                        {
                            int centerX_sum = abs(ss_center[iii].center.x - ss_center[jjj].center.x);
                            int centerY_sum = abs(ss_center[iii].center.y - ss_center[jjj].center.y);
                            if (centerX_sum < 5 && centerY_sum < 5)
                            {
                                centerX_num++;
                            }
                        }
                    }
                }
                if (centerX_num > 3)
                {
                    ss[ss_ID].center.x = ss[ss_ID].center.x - cos((double)(ss[ss_ID].angle) / 180.0*PI)*ss[ss_ID].size.height;
                    ss[ss_ID].center.y = ss[ss_ID].center.y - sin((double)(ss[ss_ID].angle) / 180.0*PI)*ss[ss_ID].size.height;

                    target.center = ss[ss_ID].center;
                    target.size.width = ss[ss_ID].size.height * 2;
                    target.size.height = ss[ss_ID].size.height;

                    target.angle = ss[ss_ID].angle;

                    all.push_back(ss[ss_ID]);
                }
            }
        }
        else if (armor_SECOND.size() == 1)
        {
            target = armor_SECOND[0].armorS;
            all.push_back(armor_SECOND[0].armorS);
            //存储上一次装甲中心点
            armor_center_x = target.center.x;
            armor_center_y = target.center.y;
        }
        else
        {
            double min_feature = 9999999;
            for (int armor_i = 0; armor_i < armor_SECOND.size(); armor_i++)//对各个灯带进行遍历
            {
                armors.push_back(armor_SECOND[armor_i].armorS);
                //计算加权特征值
                double feature = armor_SECOND[armor_i].build_features[0] * 100 +
                                 armor_SECOND[armor_i].build_features[1] * 10 +
                                 armor_SECOND[armor_i].build_features[2] * 100 +
                                 //armor_SECOND[armor_i].build_features[3] * 0 +
                                 abs(armor_SECOND[armor_i].armorS.center.x - armor_center_x) * 50 +
                                 abs(armor_SECOND[armor_i].armorS.center.y - armor_center_y) * 50 -
                                 armor_SECOND[armor_i].armorS.size.height * 100 -
                                 armor_SECOND[armor_i].armorS.size.width * 100 -
                                 armor_SECOND[armor_i].armorS.size.height * armor_SECOND[armor_i].armorS.size.width * 2.5;
                cout << feature << endl;
                if (feature < min_feature) // 找到最小特征值
                {
                    min_feature = feature;
                    target = armor_SECOND[armor_i].armorS;
                }

            }
            //存储装甲中心点
            armor_center_x = target.center.x;
            armor_center_y = target.center.y;
            all = armors;
        }
    }
}
/*************************************************
Function:       solvePNP
Description:    计算装甲板距离
Input:
Output:
Return:
Others:
*************************************************/
double ArmorPlate::solvePNP(cv::RotatedRect &target)
{
    cv::Mat revc,K_;
    cv::Mat tvec,D_;
    cv::FileStorage fs2(PNP_MATRIX,cv::FileStorage::READ);
    fs2["camera_matrix"]>>K_;
    fs2["distortion_coefficients"]>>D_;

    std::vector<cv::Point3f>corners1;
    std::vector<cv::Point2f>observation_points;
    Point2f imagePoints[4];
    target.points(imagePoints);
    if(target.size.width/target.size.height>4) //大装甲
    {
        for (int i = 0; i < 4; i++) {
            Point3f tmp;
            tmp.x = corners_big[i][0];
            tmp.y = corners_big[i][1];
            tmp.z = 0;
            corners1.push_back(tmp);
        }
    }
    else
    {
        for (int i = 0; i < 4; i++) {
            Point3f tmp;
            tmp.x = corners[i][0];
            tmp.y = corners[i][1];
            tmp.z = 0;
            corners1.push_back(tmp);
        }

    }
    for(int i=0;i<4;i++)
    {
        observation_points.push_back(imagePoints[i]);
    }

    cv:: solvePnP(corners1,observation_points,K_,D_,revc,tvec,false);
    double solvepnp_z=tvec.at<double>(2,0);
    //cout << "depth" << solvepnp_z <<endl;
    return solvepnp_z;
}
/*************************************************
Function:       MotionPrediction
Description:    运动预测
Input:          target
Output:         预测之后的 target
Return:
Others:         利用多帧图像之的坐标进行预测
*************************************************/

bool ArmorPlate:: MotionPrediction(cv::RotatedRect& target, cv::RotatedRect& last_target, cv::RotatedRect& last_last_target)
{
    target.center.x = target.center.x + ((20 + (15 * target.center.x - 20 * last_target.center.x + 5 * last_last_target.center.x)) * 0.40);
    target.center.y = target.center.y + ((20 + (15 * target.center.y - 20 * last_target.center.y + 5 * last_last_target.center.y)) * 0.56);
    //target.center.x=target.center.x+((target.center.x/t2)-(last_target.center.x/t2)+((target.center.x/(t2*(t1+t2)))-(last_target.center.x/(t1*t2))+(last_last_target.center.x/(t1*(t1+t2))))*1/60)*(1/60)*6.4;
    //target.center.y=target.center.y+((target.center.y/t2)-(last_target.center.y/t2)+((target.center.y/(t2*(t1+t2)))-(last_target.center.y/(t1*t2))+(last_last_target.center.y/(t1*(t1+t2))))*1/60)*(1/60)*9.6;
}

/*************************************************
Function:       MotionPrediction
Description:    运动预测
Input:          target
Output:         预测之后的 target
Return:
Others:         利用卡尔曼滤波器进行预测,不知道什么问题。
*************************************************/

/*
bool ArmorPlate:: MotionPrediction(cv::RotatedRect& target, cv::RotatedRect& last_target)
{
    Kalman_example::KalmanFilter kf(50,50);
    int x,y;
    Point2f kalmanPoint(x,y);
    kalmanPoint = kf.run(last_target.center.x,last_target.center.y);
    target.center.x=kalmanPoint.x;
    target.center.y=kalmanPoint.y;

}
*/
/*************************************************
Function:       CalculateAngP
Description:    计算 P 轴角度
Input:
Output:
Return:
Others:
*************************************************/
float ArmorPlate::CalculateAngP(cv::RotatedRect& target)
{
    //return atan2((img_center_y-target.center.y),KV)*180/PI;
    return img_center_y-target.center.y;
}
/*************************************************
Function:       CalculateAngY
Description:    计算 Y 轴角度
Input:
Output:
Return:
Others:
*************************************************/
float ArmorPlate::CalculateAngY(cv::RotatedRect& target)
{
    //return atan2((target.center.x - img_center_x),KU)*180/PI;
    return target.center.x - img_center_x;
}

/*************************************************
Function:       BallistcCalculation
Description:     弹道计算
Input:
Output:
Return:
Others:
*************************************************/
double ArmorPlate::BallistcCalculation(double distance)
{
    double angle;
    double g=9.80;
    angle=asin(g*distance/BallisticCal_BallisticSpd/BallisticCal_BallisticSpd)*180.0/PI/2;
    return angle;
}

/*************************************************
Function:       AngYControl
Description:    云台 Y 轴控制
Input:
Output:
Return:
Others:
*************************************************/
/*
bool ArmorPlate::AngYControl()
{
    error_y_[2] = error_y_[1];//上上一次角度
    error_y_[1] = error_y_[0];//上一次角度
    error_y_[0] = ang_y_;//当前角度
    p_y=kp+error/100;
    ang_y_ = y_kp_ * error_y_[0] + y_ki_ * (error_y_[0] - 2 * error_y_[0] + error_y_[2]) + y_kd_ * (error_y_[0] - error_y_[1]);
            //+ p_y*error_y_[0]+y_ki_*(error_y_[0]-2*error_y_[0]+error_y_[2])+y_kd_ * (error_y_[0] - error_y_[1]);
}*/
bool  ArmorPlate::AngYControl()
{





}
/*************************************************
Function:       AngPControl
Description:    云台 P 轴控制
Input:
Output:
Return:
Others:
*************************************************/
bool ArmorPlate::AngPControl()
{
    error_p_[2] = error_p_[1];
    error_p_[1] = error_p_[0];
    error_p_[0] = ang_p_;

    ang_p_ = p_kp_ * error_p_[0] + p_ki_ * (error_p_[0] - 2 * error_p_[0] + error_p_[2]) + p_kd_ * (error_p_[0] - error_p_[1]);
}

/*************************************************
Function:       DrawTarget
Description:    显示参数
Input:
Output:
Return:
Others:
*************************************************/
void ArmorPlate::DisplayParameters()
{
    string text[6];
    stringstream s[6];
    s[0] << "p.kp:" << p_kp_;
    s[1] << "p.ki:" << p_ki_;
    s[2] << "p.kd:" << p_kd_;
    s[3] << "y.kp:" << y_kp_;
    s[4] << "y.ki:" << y_ki_;
    s[5] << "y.kd:" << y_kd_;
    for (int i = 0; i < 6; i++)
        s[i] >> text[i]; // 或者 temp = s.str();

    //创建空白图用于绘制文字
    Mat image = cv::Mat::zeros(cv::Size(250, 300), CV_8UC3);
    //设置背景
    image.setTo(cv::Scalar(0, 0, 0));
    //设置绘制文本的相关参数
    int font_face = cv::FONT_HERSHEY_COMPLEX;
    double font_scale = 0.8;
    int thickness = 1;
    //int baseline;
    //获取文本框的长宽
    //Size text_size = getTextSize(text, font_face, font_scale, thickness, &baseline);
    //将文本框居中绘制
    //Point origin;
    //origin.x = image.cols / 2 - text_size.width / 2;
    //origin.y = image.rows / 2 + text_size.height / 2;

    putText(image, text[0], Point(20, 40), font_face, font_scale, cv::Scalar(255, 255, 255), thickness, 16, 0);
    putText(image, text[1], Point(20, 80), font_face, font_scale, cv::Scalar(255, 255, 255), thickness, 16, 0);
    putText(image, text[2], Point(20, 120), font_face, font_scale, cv::Scalar(255, 255, 255), thickness, 16, 0);
    putText(image, text[3], Point(20, 160), font_face, font_scale, cv::Scalar(255, 255, 255), thickness, 16, 0);
    putText(image, text[4], Point(20, 200), font_face, font_scale, cv::Scalar(255, 255, 255), thickness, 16, 0);
    putText(image, text[5], Point(20, 240), font_face, font_scale, cv::Scalar(255, 255, 255), thickness, 16, 0);

    //显示绘制解果
    //imshow("PID", image);
    waitKey(1);
}
/*************************************************
Function:       DrawAll
Description:    画出视野目标
Input:
Output:
Return:
Others:
*************************************************/
void DrawAll(vector<RotatedRect> rect, Mat img)
{
    for (int i = 0; i < rect.size(); i++)
    {
        Point2f ppt[4];
        rect[i].points(ppt); // 计算二维盒子顶点
        line(img, ppt[0], ppt[1], CV_RGB(255, 255, 255), 1, 8, 0);
        line(img, ppt[1], ppt[2], CV_RGB(255, 255, 255), 1, 8, 0);
        line(img, ppt[2], ppt[3], CV_RGB(255, 255, 255), 1, 8, 0);
        line(img, ppt[3], ppt[0], CV_RGB(255, 255, 255), 1, 8, 0);
    }

}
/*************************************************
Function:       DrawTarget
Description:    画出击打目标
Input:
Output:
Return:
Others:
*************************************************/
void DrawTarget(RotatedRect box, Mat img)
{
    Point2f pts[8];
    pts[0].x = box.center.x;
    pts[0].y = box.center.y - 5;
    pts[1].x = box.center.x;
    pts[1].y = box.center.y + 5;
    pts[2].x = box.center.x - 5;
    pts[2].y = box.center.y;
    pts[3].x = box.center.x + 5;
    pts[3].y = box.center.y;

    pts[4].x = img_center_x;
    pts[4].y = img_center_y - 5;
    pts[5].x = img_center_x;
    pts[5].y = img_center_y +  5;
    pts[6].x = img_center_x - 5;
    pts[6].y = img_center_y ;
    pts[7].x = img_center_x + 5;
    pts[7].y = img_center_y ;
    line(img, pts[0], pts[1], CV_RGB(0, 255, 0), 2, 8, 0);
    line(img, pts[2], pts[3], CV_RGB(0, 255, 0), 2, 8, 0);
    line(img, pts[4], pts[5], CV_RGB(255, 255, 255), 2, 8, 0);
    line(img, pts[6], pts[7], CV_RGB(255, 255, 255), 2, 8, 0);
}
/***************************************************
 * function    用于运动预测的时候，采用了三帧的看他的中心
 * @param box
 * @param img
 * *************************************************
 */
void DrawTarget1(RotatedRect box, Mat img)
{
    Point2f pts[4];
    pts[0].x = box.center.x;
    pts[0].y = box.center.y - 5;
    pts[1].x = box.center.x;
    pts[1].y = box.center.y + 5;
    pts[2].x = box.center.x - 5;
    pts[2].y = box.center.y;
    pts[3].x = box.center.x + 5;
    pts[3].y = box.center.y;

    line(img, pts[0], pts[1], CV_RGB(0, 0, 255), 2, 8, 0);
    line(img, pts[2], pts[3], CV_RGB(0, 0, 255), 2, 8, 0);
}
void DrawTarget2(RotatedRect box, Mat img)
{
    Point2f pts[4];
    pts[0].x = box.center.x;
    pts[0].y = box.center.y - 5;
    pts[1].x = box.center.x;
    pts[1].y = box.center.y + 5;
    pts[2].x = box.center.x - 5;
    pts[2].y = box.center.y;
    pts[3].x = box.center.x + 5;
    pts[3].y = box.center.y;

    line(img, pts[0], pts[1], CV_RGB(255, 0, 0), 2, 8, 0);
    line(img, pts[2], pts[3], CV_RGB(255, 0, 0), 2, 8, 0);
}
/*************************************************
Function:       AutoShoot
Description:    辅助射击
Input:
Output:
Return:
Others:         主函数调用
*************************************************/
void ArmorPlate::AutoShoot()
{
    Mat img_target = armor_image_; // 视觉辅助原图
    //cout<<armor_image_.rows<<' '<<armor_image_.cols<<endl;
    //process.ThreadProcess(armor_image_, pre_image_, 0, 4); // 使用多线程进行图像预处理
	ImgPreProcess(armor_image_, pre_image_); // 进行图像预处理
    last_last_target_ = last_target_;
    last_target_ = target_;
    FindArmor(armor_image_, pre_image_, all_target_, target_); // 寻找装甲板
    new_target_ = target_;
    DrawAll(all_target_, img_target); // 全部目标
    DrawTarget(target_, img_target); // 击打目标
    depth_ = solvePNP(target_);
    //cout << "depth:" << depth_ << endl;

    if (target_.size.height > 5)  // 目标有效
    {
        Pitch_pid.PID_Change_Pitch(&Pitch_pid,img_center_y,target_.center.y);
        ang_p_=Pitch_pid.PID_Control_P(&Pitch_pid,img_center_y,target_.center.y);
        cout << "ang_p:" << ang_p_<< endl;

        Yaw_pid.PID_Change_Yaw(&Yaw_pid,img_center_x,target_.center.x);
        ang_y_=Yaw_pid.PID_Control_P(&Yaw_pid,img_center_x,target_.center.x);
        cout << "ang_y" << ang_y_ << endl << endl;

        if (last_last_target_.size.height > 5)  // 上次有目标
            //MotionPrediction(new_target_, last_target_, last_last_target_); // 运动预测

        //DrawTarget2(last_target_, img_target); // 击打目标
        DrawTarget1(new_target_, img_target); // 击打目标
        

        if (abs(ang_p_) < 2 && abs(ang_y_) < 4)
        {
            shoot_flag_ = 1;

        }
        else
        {
            shoot_flag_ = 0;
        }
    }
    else // 失去目标
    {
        last_last_target_.center.x = 0;
        last_last_target_.center.y = 0;
        last_target_.center.x = 0;
        last_target_.center.y = 0;
        shoot_flag_ = 0;
        ang_p_ = 0;
        ang_y_ = 0;
    }

    DisplayParameters();
    imshow("shot", img_target);
    //waitKey(1);
}







/*************************************************
以下内容为线程创建部分内容，已取消使用，改为一个单独的类
*************************************************/
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"
/*************************************************
Function:       ReadThread
Description:    读取线程函数
Input:
Output:
Return:
Others:         在线程里进行预处理
*************************************************/
void* ArmorPlate::ImgPreProcessThread(void *pData)
{
    ArmorPlate *armor = reinterpret_cast<ArmorPlate*>(pData);
    cout << "线程已进入" << endl;
    while (1)
    {
        cout << "线程已进入" << endl;
        if(1)
        {
        }
        else
            break;
    }
    cout << "线程已退出" << endl;
    pthread_exit(0);
}
#pragma clang diagnostic pop
/*************************************************
Function:       StartThreadFunc
Description:    开始线程
Input:          func pthread par
Output:
Return:
Others:         func        线程函数
                pthread     线程函数所在pthread变量
                par         线程函数参数
*************************************************/
int ArmorPlate::StartThread(void *(*func) (void *), pthread_t* pthread, void* par)
{
    memset(pthread, 0, sizeof(pthread_t));
    int temp;
    /*创建线程*/
    if((temp = pthread_create(pthread, NULL, func, par)) != 0)
        cout << "线程创建失败!" << endl;
    else
    {
        cout << pthread_self() << endl;
        cout << "线程" << *pthread << "创建成功！" << endl;
    }
    return temp;
}
/*************************************************
Function:       StopThread
Description:    结束线程
Input:          pthread 线程函数所在pthread变量
Output:
Return:
Others:
*************************************************/
int ArmorPlate::StopThread(pthread_t* pthread)
{
    cout << "正在退出线程" << *pthread << "！" << endl;
    if(*pthread !=0)
        pthread_join(*pthread, NULL);

    cout << "线程" << *pthread << "已退出！" << endl;
}
/*************************************************
Function:       ThreadInit
Description:    初始化一个线程 读取线程
Input:
Output:
Return:         true or false
Others:         bool
*************************************************/
bool ArmorPlate::ThreadInit()
{
    //pthread_t thread;
    cout << pthread_self() << endl;
    int COM_READ_STATU = 0;
    if(StartThread(ImgPreProcessThread, &img_preprocess_thread_,  &COM_READ_STATU) != 0)
    {
        cout << "线程创建失败" << endl;
        return false;
    }
    return  true;
}


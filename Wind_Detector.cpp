#include "Wind_Detector.h"

using namespace std;
using namespace cv;

// extern double circle_x[11];
// extern double circle_y[11];
// extern Point2f circle_center,attack_center;
// extern bool Team;
// extern bool R_point;     
// extern double Len_Poly[11],        
//        		  R_LEN,           
//       		  A_LEN;           

// 大装甲世界坐标

extern  float corners_big[4][2] ;
/*
{
    0.0, 0.0,
    0.0, 0.055,
    0.225, 0.055,
    0.225, 0.0,
};*/

//数据拟合
double Poly_Average(double A[], int start_loc, int end_loc)
{
	double ave,sum = 0;
	int num = end_loc - start_loc + 1;
	for(unsigned int i = start_loc; i <= end_loc; ++i)
	{
		sum = sum + A[i];
	}
	// if(num > 2)
	// {
	// 	int max = 0, min = 0;
	// 	for(unsigned int i = start_loc; i <= end_loc; ++i)
	// 	{
	// 		if(A[i] > A[max]) max = i;
	// 		if(A[i] < A[min]) min = i;
	// 	}
	// 	sum = sum - A[max];
	// 	sum = sum - A[min];
	// 	num = num - 2; 
	// }
	ave = sum/num;
	return ave;
}

WindMill::WindMill()
{
	//点初始化
	InitCircle();
	//导入相机参数
	cv::FileStorage fs2(CAMERA, cv::FileStorage::READ);
	fs2["camera_matrix"] >> K_;
	fs2["distortion_coefficients"] >> D_;
	fs2.release();
	//cout << K_.size() << endl;
	//相机矩阵转换为二维数组
	for(unsigned int i = 0; i <= 2; ++i)
	{
		for(unsigned int j = 0; j <= 2; ++j)
		{
			K_Array[i][j] = K_.at<double>(i,j);
			//cout << K_.at<double>(i,j) << " ";
		}
	}
    Recognition();
}

//点坐标初始化
void WindMill::InitCircle()
{
    circle_x[0] = 1;
    circle_y[0] = 1;
    circle_x[10] = -1;
    circle_y[10] = -1;
    circle_center.x = 0;
    circle_center.y = 0;
    attack_center.x = 0;
    attack_center.y = 0;
}

//标定线初始化
void WindMill::InitCali(Mat &img)
{
    if(circle_center.x > 0)
    {
        cali[0].points(circle_center, Point2f(circle_center.x + CALILEN, circle_center.y));//0
        cali[1].points(circle_center, Point2f(circle_center.x + 0.5 * CALILEN, circle_center.y - sqrt(3)/2 * CALILEN));//60
        cali[2].points(circle_center, Point2f(circle_center.x - 0.5 * CALILEN, circle_center.y - sqrt(3)/2 * CALILEN));//120
        cali[3].points(circle_center, Point2f(circle_center.x - CALILEN, circle_center.y));//180
        cali[4].points(circle_center, Point2f(circle_center.x - 0.5 * CALILEN, circle_center.y + sqrt(3)/2 * CALILEN));//240
        cali[5].points(circle_center, Point2f(circle_center.x + 0.5 * CALILEN, circle_center.y + sqrt(3)/2 * CALILEN));//300
        //printf("(%lf,%lf)\n",circle_center.x,circle_center.y);
        for(int i = 0; i < 6; ++i)
            line(img, cali[i].start, cali[i].end, Scalar(255, 0, 255), 2, CV_AA);
    }
}

//**************************************************************
//***************************预处理******************************
//**************************************************************

//BGR
//找中心点预处理
// Mat WindMill::BR_pretreatA(Mat img, string T)
// {
//     Mat imgBin;
//     vector<Mat> BGR_channels;
// 	split(img, BGR_channels);
// 	Mat element = getStructuringElement(MORPH_CROSS, Size(3, 3));
// 	if(Team == RED)
// 	{
// 		Mat channel = BGR_channels.at(2) - BGR_channels.at(1) * 0.5 - BGR_channels.at(0) * 0.5;//R - B
// 		GaussianBlur(channel, channel, Size(3, 3), 0, 0);
// 		channel = channel * 2;
// 		double maxValue_gray;
//         minMaxLoc(channel, 0, &maxValue_gray, 0, 0); // 最大最小
//         if(maxValue_gray < 150)
//         {
//             maxValue_gray = 180;
//         }
// 		inRange(channel, maxValue_gray * RED_PREMINCOE_R, maxValue_gray * RED_PREMAXCOE_R, imgBin);
//         //threshold(channel, imgBin, maxValue_gray*0.5, 255, THRESH_BINARY); // 入口
//         //cv::dilate(imgBin, imgBin, element, Point(-1, -1), 3); // 膨胀(dilate) 局部最大值(浅色)扩大3圈
//         cv::erode(imgBin, imgBin, element, Point(-1, -1), 2); // 腐蚀(erode) 局部最小值(深色)扩大5圈
//         cv::dilate(imgBin, imgBin, element, Point(-1, -1), 2); // 膨胀(dilate) 局部最大值(浅色)扩大5圈
// 	}
// 	if(Team == BLUE)
// 	{
// 		Mat channel = BGR_channels.at(0) - BGR_channels.at(1) * 0.5 - BGR_channels.at(2) * 0.5;//R - B
// 		GaussianBlur(channel, channel, Size(3, 3), 0, 0);
// 		channel = channel * 2;
// 		double maxValue_gray;
//         minMaxLoc(channel, 0, &maxValue_gray, 0, 0); // 最大最小
//         if(maxValue_gray < 150)
//         {
//             maxValue_gray = 180;
//         }
// 		//inRange(channel, maxValue_gray * 0.5, maxValue_gray * 0.8, imgBin);
// 		inRange(channel, maxValue_gray * BLUE_PREMINCOE_R, maxValue_gray * BLUE_PREMAXCOE_R, imgBin);
// 		imgBin = ~imgBin;
//         //threshold(channel, imgBin, maxValue_gray*0.5, 255, THRESH_BINARY); // 入口
//         //cv::dilate(imgBin, imgBin, element, Point(-1, -1), 3); // 膨胀(dilate) 局部最大值(浅色)扩大3圈
//         cv::erode(imgBin, imgBin, element, Point(-1, -1), 2); // 腐蚀(erode) 局部最小值(深色)扩大5圈
//         cv::dilate(imgBin, imgBin, element, Point(-1, -1), 2); // 膨胀(dilate) 局部最大值(浅色)扩大5圈
// 	}
// 	//imshow(T, imgBin);
// 	return imgBin;
// }

//识别预处理
// Mat WindMill::BR_pretreatB(Mat img, string T)
// {
//     Mat imgBin_;
//     vector<Mat> BGR_channels;
// 	split(img, BGR_channels);
// 	Mat element = getStructuringElement(MORPH_CROSS, Size(3, 3));
// 	Mat element2 = getStructuringElement(MORPH_CROSS, Size(5, 5));
// 	Mat imgBin;
// 	if(Team == RED)
// 	{
// 		Mat channel = BGR_channels.at(2) - BGR_channels.at(1) * 0.5 - BGR_channels.at(0) * 0.5;//R - B
// 		GaussianBlur(channel, channel, Size(3, 3), 0, 0);
// 		channel = channel * 2;
// 		double maxValue_gray;
//         minMaxLoc(channel, 0, &maxValue_gray, 0, 0); // 最大最小
//         if(maxValue_gray < 150)
//         {
//             maxValue_gray = 180;
//         }
// 		inRange(channel, maxValue_gray * RED_PREMINCOE_F, maxValue_gray * RED_PREMAXCOE_F, imgBin_);
//         //threshold(channel, imgBin, maxValue_gray*0.5, 255, THRESH_BINARY); // 入口
//         //cv::dilate(imgBin, imgBin, element, Point(-1, -1), 3); // 膨胀(dilate) 局部最大值(浅色)扩大3圈
//         cv::erode(imgBin_, imgBin_, element, Point(-1, -1), 2); // 腐蚀(erode) 局部最小值(深色)扩大5圈
//         cv::dilate(imgBin_, imgBin_, element2, Point(-1, -1), 2); // 膨胀(dilate) 局部最大值(浅色)扩大5圈
// 		//imshow("漫水填充前", imgBin_);
// 	}
// 	if(Team == BLUE)
// 	{
// 		Mat channel = BGR_channels.at(0) - BGR_channels.at(1) * 0.5 - BGR_channels.at(2) * 0.5;//R - B
// 		GaussianBlur(channel, channel, Size(3, 3), 0, 0);
// 		channel = channel * 2;
// 		double maxValue_gray;
//         minMaxLoc(channel, 0, &maxValue_gray, 0, 0); // 最大最小
//         if(maxValue_gray < 150)
//         {
//             maxValue_gray = 180;
//         }
// 		//inRange(channel, maxValue_gray * 0.5, maxValue_gray * 0.8, imgBin);
// 		inRange(channel, maxValue_gray * BLUE_PREMINCOE_F, maxValue_gray * BLUE_PREMAXCOE_F, imgBin);
// 		imgBin = ~imgBin_;
//         //threshold(channel, imgBin, maxValue_gray*0.5, 255, THRESH_BINARY); // 入口
//         //cv::dilate(imgBin, imgBin, element, Point(-1, -1), 3); // 膨胀(dilate) 局部最大值(浅色)扩大3圈
//         cv::erode(imgBin_, imgBin_, element, Point(-1, -1), 2); // 腐蚀(erode) 局部最小值(深色)扩大5圈
//         cv::dilate(imgBin_, imgBin_, element2, Point(-1, -1), 2); // 膨胀(dilate) 局部最大值(浅色)扩大5圈
// 	}
// 	Point2f OriginPoint;
// 	OriginPoint.x = 1;
// 	OriginPoint.y = 1;
// 	imgBin_ = ~imgBin_;
// 	floodFill(imgBin_, OriginPoint, Scalar(0,0,0));
// 	//imshow(T, imgBin_);
// 	return imgBin_;
// }
//bool WindMill::CameraInit(int device)
//{
//    capture_armor_.open(device);
//    if (!capture_armor_.isOpened())
//    {
//        printf("视觉辅助像头打开失败！\n");
//        return false;
//
//    }
//    else
//    {
//        // 设置摄像头参数
//        capture_armor_.set(CAP_PROP_AUTO_EXPOSURE, 0.25); // where 0.25 means "manual exposure, manual iris"
//        //capture_armor_.set(CV_CAP_PROP_EXPOSURE, 0.023  );
//        capture_armor_.set(CV_CAP_PROP_EXPOSURE, 0.0007); // 曝光
//        capture_armor_.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));
//        capture_armor_.set(CV_CAP_PROP_FRAME_WIDTH,640);
//        capture_armor_.set(CV_CAP_PROP_FRAME_HEIGHT,480);
//        //cout << CV_CAP_PROP_FRAME_WIDTH << endl;
//        //cout <<CV_CAP_PROP_FRAME_HEIGHT<< endl;
//        return true;
//    }
//}


Mat WindMill::BR_pretreatA(Mat img, string T)//R
{
    if(Team == TEAMRED)
    {
        vector<Mat> BGR_channels;
        //cvtColor(img, img, COLOR_BGR2HSV);
        split(img, BGR_channels);
        Mat element = getStructuringElement(MORPH_CROSS, Size(3, 3));
        /*for(int i = 0; i < Zero.cols; ++i)
        {
            for(int j = 0; j < Zero.rows; ++j)
            {
                Point2f p(i, j);
                printf("%d\t", Zero.at<Vec3b>(p)[0]);
                printf("%d\t", Zero.at<Vec3b>(p)[1]);
                printf("%d\n", Zero.at<Vec3b>(p)[2]);
                //cout << Zero.at<Vec3b>(p)[0] << " " << Zero.at<Vec3b>(p)[1] << " " << Zero.at<Vec3b>(p)[2] << endl;
            }
        }*/
        Mat BGR, B;
        Mat R = BGR_channels[2];
        Mat G = BGR_channels[1];
        Mat B_LOW = BGR_channels[0];
        Mat B_HIGH = BGR_channels[0];

        /*for(long i = 0; i < HSV.cols; ++i)
        {
            for(long j = 0; j < HSV.rows; ++j)
            {
                Point2d p(i, j);
                //if(HSV.at<Vec3b>(p)[0] < 150)
                {printf("%d\t", HSV.at<Vec3b>(p)[0]);
                printf("%d\t", HSV.at<Vec3b>(p)[1]);
                printf("%d\n", HSV.at<Vec3b>(p)[2]);}
            }
        }*/
        // imshow(T, HSV);
        //inRange(BGR, 150, 255, BGR);
        //inRange(HSV, minexp, maxexp, HSV);//红色阈值
        inRange(B_LOW, 0, BMIN, B_LOW);
        inRange(B_HIGH, BMAX, 255, B_HIGH);
        inRange(G, GMIN, GMAX, G);
        inRange(R, RMIN, RMAX, R);
        //inRange(HSV, 150, 255, HSV);//蓝色阈值
        bitwise_or(B_LOW, B_HIGH, B);
        bitwise_and(G, R, BGR);
        bitwise_and(BGR, B, BGR);

        dilate(BGR, BGR, element);
        dilate(BGR, BGR, element);
        erode(BGR, BGR, element);
        dilate(BGR, BGR, element);

        //imshow(T, BGR);

	
	    return BGR;
    }
    else if(Team == TEAMBLUE)
    {
        vector<Mat> BGR_channels;
        //cvtColor(img, img, BGR2HSV);
        split(img, BGR_channels);
        Mat element = getStructuringElement(MORPH_CROSS, Size(3, 3));
        Mat element2 = getStructuringElement(MORPH_CROSS, Size(5, 5));
        Mat BGR;
        Mat R = BGR_channels[2];
        Mat G = BGR_channels[1];
        Mat G_HIGH, R_HIGH, G_LOW, R_LOW;
        Mat B = BGR_channels[0];
        Mat WHITE;

        inRange(G, 180, 255, G_HIGH);//200-255
        inRange(R, 180, 255, R_HIGH);//200-200

        inRange(B, 220, 255, B);//195-255
        inRange(G, 50, 255, G_LOW);//85-255
        inRange(R, 0, 255, R_LOW);//50-200 20-255

        bitwise_and(G_LOW, R_LOW, BGR);
        bitwise_and(BGR, B, BGR);

        bitwise_and(G_HIGH, R_HIGH, WHITE);
        bitwise_and(WHITE, B, WHITE);
        dilate(WHITE, WHITE, element);
        //异或
        //bitwise_xor(BGR, WHITE, BGR);
        //取反
        WHITE = ~WHITE;
        bitwise_and(BGR, WHITE, BGR);

        dilate(BGR, BGR, element);
        erode(BGR, BGR, element);
        dilate(BGR, BGR, element2);

        //imshow(T, BGR);
        //imshow("WHITE", WHITE);

        return BGR;
    }

}

Mat WindMill::BR_pretreatB(Mat img, string T)
{
	if(Team == TEAMRED) {
		//Mat Zero = img;
		vector<Mat> BGR_channels;
		//cvtColor(img, img, COLOR_BGR2HSV);
		split(img, BGR_channels);
		Mat element = getStructuringElement(MORPH_CROSS, Size(3, 3));
		/*for(int i = 0; i < Zero.cols; ++i)
		{
			for(int j = 0; j < Zero.rows; ++j)
			{
				Point2f p(i, j);
				printf("%d\t", Zero.at<Vec3b>(p)[0]);
				printf("%d\t", Zero.at<Vec3b>(p)[1]);
				printf("%d\n", Zero.at<Vec3b>(p)[2]);
				//cout << Zero.at<Vec3b>(p)[0] << " " << Zero.at<Vec3b>(p)[1] << " " << Zero.at<Vec3b>(p)[2] << endl;
			}
		}*/
		Mat BGR;
		Mat R = BGR_channels[2];
		Mat G = BGR_channels[1];
		Mat B_LOW = BGR_channels[0];
		Mat B_HIGH = BGR_channels[0];
		Mat B;

		/*for(long i = 0; i < HSV.cols; ++i)
		{
			for(long j = 0; j < HSV.rows; ++j)
			{
				Point2d p(i, j);
				//if(HSV.at<Vec3b>(p)[0] < 150)
				{printf("%d\t", HSV.at<Vec3b>(p)[0]);
				printf("%d\t", HSV.at<Vec3b>(p)[1]);
				printf("%d\n", HSV.at<Vec3b>(p)[2]);}
			}
		}*/
		// imshow(T, HSV);
		//inRange(BGR, 150, 255, BGR);
		//inRange(HSV, minexp, maxexp, HSV);//红色阈值
		inRange(B_LOW, 0, BMIN, B_LOW);
		inRange(B_HIGH, BMAX, 255, B_HIGH);
		inRange(G, GMIN, GMAX, G);
		inRange(R, RMIN, RMAX, R);
		//inRange(HSV, 150, 255, HSV);//蓝色阈值
		bitwise_or(B_LOW, B_HIGH, B);
		bitwise_and(G, R, BGR);
		bitwise_and(BGR, B, BGR);

		dilate(BGR, BGR, element);
		dilate(BGR, BGR, element);
		erode(BGR, BGR, element);
		dilate(BGR, BGR, element);

		imshow(T, BGR);

		Point2f OriginPoint;
		OriginPoint.x = 1;
		OriginPoint.y = 1;
		BGR = ~BGR;
		floodFill(BGR, OriginPoint, Scalar(0, 0, 0));
		imshow("flood", BGR);

		return BGR;
	}
	else if(Team == TEAMBLUE)
	{
		vector<Mat> BGR_channels;
		//cvtColor(img, img, BGR2HSV);
		split(img, BGR_channels);
		Mat element = getStructuringElement(MORPH_CROSS, Size(3, 3));

		Mat BGR;
		Mat R = BGR_channels[2];
		Mat G = BGR_channels[1];
		Mat G_HIGH, R_HIGH, G_LOW, R_LOW;
		Mat B = BGR_channels[0];
		Mat WHITE;

		inRange(B, 200, 255, B);//195-255
		inRange(G, 65, 255, G_LOW);//85-255
		inRange(R, 20, 255, R_LOW);//50-200 20-255

		inRange(G, 180, 255, G_HIGH);//200-255
		inRange(R, 180, 255, R_HIGH);//200-200

		bitwise_and(G_LOW, R_LOW, BGR);
		bitwise_and(BGR, B, BGR);

		/*bitwise_and(G_HIGH, R_HIGH, WHITE);
        bitwise_and(WHITE, B, WHITE);*/
		//dilate(WHITE, WHITE, element);
		//异或
		//bitwise_xor(BGR, WHITE, BGR);
		//取反
		/*WHITE = ~WHITE;
        bitwise_and(BGR, WHITE, BGR);*/

		dilate(BGR, BGR, element);
		//erode(BGR, BGR, element);
		dilate(BGR, BGR, element);

		//imshow(T, BGR);

		Point2f OriginPoint;
		OriginPoint.x = 1;
		OriginPoint.y = 1;
		BGR = ~BGR;
		floodFill(BGR, OriginPoint, Scalar(0,0,0));
		//bitwise_xor(BGR, WHITE, BGR);
		//imshow("flood", BGR);

		return BGR;
	}
}

void WindMill::Find_Attack(Mat &img, Line attack_target, double time)
{
	bool cali_reach = false;
	double ang_dif = 60 * time;
	int cali_num;
	data_send = false;
	if(attack_target.end.x > circle_center.x)//0.5s
	{
		if(abs(attack_target.angle() - 300) < CALI_RECOG_SCOPE)//angle:300
		{
			attack_center.x = cos((-60 - ang_dif)*PI/180) * R_LEN + circle_center.x;
			attack_center.y = -sin((-60 - ang_dif)*PI/180) * R_LEN + circle_center.y;
			cali_reach = true;
			cali_num = 300;
		}
		if(abs(attack_target.angle()) < CALI_RECOG_SCOPE)	 //angle:0
		{
			attack_center.x = cos((-ang_dif)*PI/180) * R_LEN + circle_center.x;
			attack_center.y = -sin((-ang_dif)*PI/180) * R_LEN + circle_center.y;
			cali_reach = true;
			cali_num = 0;
		}
		if(abs(attack_target.angle() - 60) < CALI_RECOG_SCOPE)//angle:60
		{
			attack_center.x = cos((60 - ang_dif)*PI/180) * R_LEN + circle_center.x;
			attack_center.y = -sin((60 - ang_dif)*PI/180) * R_LEN + circle_center.y;
			cali_reach = true;
			cali_num = 60;
		}
		//if(cali_reach)
			//cout << attack_target.angle() << endl; 
	}
	else if(attack_target.end.x < circle_center.x)
	{	
		if(abs(attack_target.angle() - 120) < CALI_RECOG_SCOPE)//angle:120
		{
			attack_center.x = cos((120 - ang_dif)*PI/180) * R_LEN + circle_center.x;
			attack_center.y = -sin((120 - ang_dif)*PI/180) * R_LEN + circle_center.y;
			cali_reach = true;
			cali_num = 120;
		}
		if(abs(attack_target.angle() - 180) < CALI_RECOG_SCOPE)	 //angle:180
		{
			attack_center.x = cos((180 - ang_dif)*PI/180) * R_LEN + circle_center.x;
			attack_center.y = -sin((180 - ang_dif)*PI/180) * R_LEN + circle_center.y;
			cali_reach = true;
			cali_num = 180;
		}
		if(abs(attack_target.angle() - 240) < CALI_RECOG_SCOPE)//angle:240
		{
			attack_center.x = cos((240 - ang_dif)*PI/180) * R_LEN + circle_center.x;
			attack_center.y = -sin((240 - ang_dif)*PI/180) * R_LEN + circle_center.y;
			cali_reach = true;
			cali_num = 240;
		}
		//if(cali_reach)
			//cout << attack_target.angle() << endl;
	}
	if(cali_reach)
	{
		//printf("(%lf,%lf)\n", attack_center.x, attack_center.y);
		circle(img, attack_center, 20, Scalar(255, 0, 0), 2, 8);
		if(cali_numpro != 666 && cali_numpro != cali_num)//上次未命中
		{
			first_cali_reach = true;//重新打击
			cout << "上次未击中,重新击打:" << endl;
		}
		if(first_cali_reach)
		{
			hit_flag = true;
			first_cali_reach = false;
			cali_numpro = cali_num;
			data_send = true;
			cout << "Biu~" << endl;
		}
	}
	else if(!cali_reach && attack_center.x > 0)
	{
		circle(img, attack_center, 20, Scalar(255, 0, 0), 2, 8);
	}
}

void WindMill::R_Circular(cv::Mat &img, std::string T)
{
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	Point2f vertex[4];

	//BGR预处理
	Hchannel_ = BR_pretreatB(img, T);

	findContours(Hchannel_, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE);
	for(int i = 0; i < contours.size(); ++i)
	{
		if(contourArea(contours[i]) > 50)
		{
			RotatedRect Armor = minAreaRect(contours[i]);
			float longer,shorter;
			if(Armor.size.height > Armor.size.width)
			{
				longer = Armor.size.height;
				shorter = Armor.size.width;
			}
			else
			{
				longer = Armor.size.width;
				shorter = Armor.size.height;
			}
			cout << (longer/shorter) << endl;
            cout << Armor.size.area() <<endl;
			if((longer/shorter) >= 1.55 && (longer/shorter) <= 2)
			if(Armor.size.area() >= 1000 && Armor.size.area() <= 2000)
			{
				Circular_points.push_back(Armor.center);
				cout << Armor.size.area() << endl;
				Armor.points(vertex);
				for (unsigned int t = 0; t < 4; t++)
					line(img, vertex[t], vertex[(t + 1) % 4], Scalar(100, 100, 0), 2, CV_AA);
			}

		}
	}

}

void WindMill::R_SEL(Mat &img, string T)
{
	vector<vector<Point> > contours, contours_p1;
	vector<Vec4i> hierarchy;
	Point2f vertex[4];
	Ravarea=0;

	//BGR预处理
	//Mat Hchannel = img.clone();
	Hchannel = BR_pretreatA(img, T);

	//HSV预处理
	//Mat img_clone = img.clone();
	//Mat Hchannel = pretreat(img_clone, T);

	unsigned int min_loc = -1;
	findContours(Hchannel, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE);
	for(int i = 0; i < contours.size(); ++i)
	{
		//if(contourArea(contours[i]) > 50)
		{
			contours_p1.push_back(contours[i]);
            RotatedRect_p1.push_back(minAreaRect(contours[i]));
			//cout << contourArea(contours[i]) << endl;
		}
		//cout<<contourArea(contours[i])<<endl;
	}

	//绘制出所有轮廓包围矩形
	for(unsigned int i = 0; i < contours_p1.size(); ++i)
	{
		//boundRect.push_back(minAreaRect(contours[i]));
		//cout<<contourArea(contours_p1[i])<<endl;
        RotatedRect_p1[i].points(vertex);
		for (unsigned int t = 0; t < 4; t++)
			line(img, vertex[t], vertex[(t + 1) % 4], Scalar(0, 0, 255), 2, CV_AA);
		// if(contourArea(contours_p1[i]) < contourArea(contours_p1[min_loc]) &&
		//    boundRect_p1[i].size.width/boundRect_p1[i].size.height >= 0.95 && 
		//    boundRect_p1[i].size.width/boundRect_p1[i].size.height <= 1.05);
		// {	
		// 	min_loc=i;
		// 	for (unsigned int t = 0; t < 4; t++)
		// 	line(img, vertex[t], vertex[(t + 1) % 4], Scalar(0, 255, 0), 2, CV_AA);
		// 	//cout << "(" << boundRect_p1[min_loc].center.x << "," << boundRect_p1[min_loc].center.y << ")" << endl;
		// 	// if(circle_x[0] <= 20)
		// 	// {
		// 	// 	int loc = (int)circle_x[0];
		// 	// 	circle_x[loc] = boundRect_p1[min_loc].center.x;
		// 	// 	circle_y[loc] = boundRect_p1[min_loc].center.y;
		// 	// 	circle_x[0] += 1;
		// 	// 	circle_y[0] += 1;
		// 	// }
		// 	// else
		// 	// {
		// 	// 	circle_x[0] = circle_y[0] = 1;
		// 	// }
		// }
	}
	//寻找中心矩形
	if(RotatedRect_p1.size() > 0)
	{
		// for(unsigned int i = 0; i < boundRect_p1.size(); ++i)
		// {
		// 	if(contourArea(contours_p1[i]) < contourArea(contours_p1[min_loc]) &&
		// 	boundRect_p1[i].size.width/boundRect_p1[i].size.height >= 0.95 && 
		// 	boundRect_p1[i].size.width/boundRect_p1[i].size.height <= 1.05 )
		// 	{
		// 		min_loc=i;
		// 	};
		// }
		min_abs = min_loc=-1;
		//初始化中心矩形遍历地址
		for(unsigned int i = 0; i < RotatedRect_p1.size(); ++i)
		{
			if(contourArea(contours_p1[i]) <= 300)
			{
				min_loc = i;
				break;
			}
		}

		//寻找面积小于300的轮廓中面积最大的轮廓，视为中心R
		for(unsigned int i = min_loc; i < RotatedRect_p1.size(); ++i)
		{
			if(contourArea(contours_p1[i]) > contourArea(contours_p1[min_loc]) &&
					RotatedRect_p1[i].size.width/RotatedRect_p1[i].size.height >= 0.95 &&
                    RotatedRect_p1[i].size.width/RotatedRect_p1[i].size.height <= 1.05 &&
			contourArea(contours_p1[i]) < 300)
			{
				min_loc=i;
			};
		}

		//寻找面积小于300的矩形中长宽比最接近1的
		for(unsigned int i = min_loc; i < RotatedRect_p1.size(); ++i)
		{
			if(abs(1 - RotatedRect_p1[i].size.width/RotatedRect_p1[i].size.height) <
			abs(1 - RotatedRect_p1[min_abs].size.width/RotatedRect_p1[min_abs].size.height) &&
			contourArea(contours_p1[i]) < 300)
			{
				min_abs=i;
			};
		}
		//cout << min_abs << " " << min_loc << endl;
		//  if(min_abs != min_loc)
		// {
		// 	cout<<jishu<<endl;
		// 	jishu++;
		// 	cout << "Error" << endl;
		// }

		// if(contourArea(contours_p1[min_loc])>=300)
		// {
		// 	cout<<"Error"<<endl;
		// 	for(unsigned int i = 0; i < boundRect_p1.size(); ++i)
		// 	{
		// 		cout<<contourArea(contours_p1[i])<<" ";
		// 	}
		// 	cout<<endl;
		// 	for(unsigned int i = 0; i < boundRect_p1.size(); ++i)
		// 	{
		// 		cout<<boundRect_p1[i].size.width/boundRect_p1[i].size.height<<" ";
		// 	}
		// 	cout<<endl;
		// 	cout<<boundRect_p1[min_loc].size.width/boundRect_p1[min_loc].size.height<<endl;
		// 	// for(unsigned int i = 0; i < boundRect_p1.size(); ++i)
		// 	// {
		// 	// 	if(contourArea(contours_p1[i]) < contourArea(contours_p1[min_loc]) &&
		// 	// 	boundRect_p1[i].size.width/boundRect_p1[i].size.height >= 0.85 && 
		// 	// 	boundRect_p1[i].size.width/boundRect_p1[i].size.height <= 1.15 &&
		// 	// 	contourArea(contours_p1[i])>=300)
		// 	// 	{
		// 	// 		min_loc=i;
		// 	// 	};
		// 	// }
		// }

		//存储R矩阵信息
		if(min_abs == min_loc&&min_abs>=0)//上述两下标相同
		{
			//在svm中寻找R
			int result=FindR();
			//cout<<"result: "<<result<<endl;
			if(result==1)
			{
                //cout<<RotatedRect_p1[min_abs].size.area()<<endl;
				if (circle_x[0] <= 10)//存储数据个数
				{
					int loc = (int) circle_x[0];
					circle_x[loc] = RotatedRect_p1[min_loc].center.x;
					circle_y[loc] = RotatedRect_p1[min_loc].center.y;
					circle_x[0] += 1;
					circle_y[0] += 1;

					Len_Poly[loc] = (RotatedRect_p1[min_loc].size.height + RotatedRect_p1[min_loc].size.width) / 2;

                    RArea.push_back(RotatedRect_p1[min_abs].size.area());
				}
					//中心点拟合
				else 
				{
					R_point = true;
					circle_center.x = Poly_Average(circle_x, 1, 10);
					circle_center.y = Poly_Average(circle_y, 1, 10);

					for(int k=0;k<RArea.size();k++)
					    Ravarea=Ravarea+RArea[k];
					Ravarea=Ravarea/RArea.size();

					//中心点微调
					if (Team == TEAMRED)
					{
						circle_center.x = circle_center.x + RED_CPSX;
						circle_center.y = circle_center.y + RED_CPSY;
						R_LEN = Poly_Average(Len_Poly, 1, 10) * RED_COE;
					}
					if (Team == TEAMBLUE)
					{
						circle_center.x = circle_center.x + BLUE_CPSX;
						circle_center.y = circle_center.y + BLUE_CPSY;
						R_LEN = Poly_Average(Len_Poly, 1, 10) * BLUE_COE;
					}
					//使用圆周拟合
					circle_center = Circular_center;
					R_LEN = radius;
				}
				RotatedRect_p1[min_loc].points(vertex);
				//cout<<contourArea(contours_p1[min_loc])<<endl;
				for (unsigned int t = 0; t < 4; t++)
					line(img, vertex[t], vertex[(t + 1) % 4], Scalar(0, 255, 0), 2, CV_AA);
			}
		}
	}
}

//识别
void WindMill::Wind_Detector(Mat &img, string T, SerialPort &port)
{
	vector<vector<Point> > contours, contours_p1;
	vector<RotatedRect> boundRect, boundRect_p1;
	vector<Vec4i> hierarchy;
	vector<int> Obconi;//对应轮廓值
    vector<int> Obconii;
	vector<int> Obconii_;
	vector<int> Obconii__;
	Point2f vertex[4];

	
	circle(img, circle_center, R_LEN, Scalar(255, 255, 0), 2, 8);//绘制标定○
	//BGR预处理
	//Mat Hchannel = img.clone();
	Hchannel_ = BR_pretreatB(img, T);
	//HSV预处理
	//Mat img_clone = img.clone();
	//Mat Hchannel = pretreat(img_clone, T);

	unsigned int min_loc = 0;
	Size2f size;
	float area;
	RotatedRect rorect;
	findContours(Hchannel_, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE);
	for(int i = 0; i < contours.size(); ++i)
	{
		if(contourArea(contours[i]) > 50)
		{
			rorect = minAreaRect(Mat(contours[i]));
			contours_p1.push_back(contours[i]);
			boundRect_p1.push_back(minAreaRect(contours[i]));
            Obconii.push_back(i);
			size=rorect.size;
			area = size.width*size.height;
            //cout<<area/Ravarea<<endl<<endl;
			if (/*height / width>1.5&&height / width<2&&*//*area>1000&&area<1500*/area/Ravarea<5.5)
			{
				//cout<<"shang: "<<area/Ravarea<<endl<<endl;
                //cout <<height / width << endl;
                //cout << area << endl;
                StartRect.push_back(rorect);
				Obconi.push_back(i);
                Obpoint.push_back(rorect.center);
                Obangle.push_back(rorect.angle);
			/*Point2f vertex[4];
            rorect.points(vertex);
            for (int j = 0; j < 4; j++)
                line(img, vertex[j], vertex[(j + 1) % 4], Scalar(255, 0, 255), 3);*/
			//cout<<"shang: "<<rorect.angle<<endl;
			}
			if (/*area>1500 && area < 2700*/area/Ravarea>5.5)
			{
				StartRect_.push_back(rorect);
				//cout<<"xia:"<<area/Ravarea<<endl<<endl;
			}
			//cout << contourArea(contours[i]) << endl;
		}
		//cout<<contourArea(contours[i])<<endl;
	}

	//绘制出所有轮廓包围矩形
	/*for(unsigned int i = 0; i < contours_p1.size(); ++i)
	{
		//boundRect.push_back(minAreaRect(contours[i]));
		//cout<<contourArea(contours_p1[i])<<endl;
		boundRect_p1[i].points(vertex);
		for (unsigned int t = 0; t < 4; t++)
			line(img, vertex[t], vertex[(t + 1) % 4], Scalar(0, 0, 255), 2, CV_AA);
		
	}*/
	//svm找打击目标
	//int StartRecti=FindObject_(img);//！！！！
    //if(StartRecti<-100||StartRecti>100)
	//cout<<StartRecti<<endl;
	vector<Line> Line_angle, Line_length; 
	Line attack_target;
	//线段初始化
	if(boundRect_p1.size() > 0)
	{
		for(unsigned int i = 0; i < boundRect_p1.size(); ++i)
		{
			
			if(sqrt(pow(boundRect_p1[i].center.x - circle_center.x, 2) + pow(boundRect_p1[i].center.y - circle_center.y, 2)) > 10)
			{
				Point2f begin, goal;
				Line li;
				begin.x = circle_center.x;
				begin.y = circle_center.y;
				goal.x = boundRect_p1[i].center.x;
				goal.y = boundRect_p1[i].center.y;
				li.points(begin, goal);
				li.points(begin, goal);
				//cout << "(" << begin.x << "," << begin.y << ")" << endl;
				//line(img, begin, goal, Scalar(255,0,0), 2, CV_AA);
				if(li.length() <= 1.5 * R_LEN)
				{
                    Line_angle.push_back(li);
                    Obconii_.push_back(Obconii[i]);
                }
			}
		}
	}

	//命中改变击打目标
	if(Line_angle.size()/3 > hit_count)//命中目标
	{
		cout << "Get!" << endl;
		hit_count = Line_angle.size()/3;
		first_cali_reach = true;//重置标定线标志位
		cali_numpro = 666;//上次击中喽,刷新一下子
	}

	//寻找无相同角度的线段
	for(unsigned int i = 0; i < Line_angle.size(); ++i)
	{
		bool same = false;
		//cout << Line_angle[i].angle() << endl;
		for(unsigned int j = 0; j < Line_angle.size(); ++j)
		{
			if(i == j)
				continue;
			//cout<<abs(Line_angle[i].angle() - Line_angle[j].angle())<<endl;
			if(abs(Line_angle[i].angle() - Line_angle[j].angle()) < 40)
			{
				same = true;	
				break;
			}
		}
		if(!same)
			{
				Line_length.push_back(Line_angle[i]);
				Obconii__.push_back(Obconii_[i]);
				//line(img, Line_angle[i].start, Line_angle[i].end, Scalar(50,200,50), 2, CV_AA);
			}
	}

	// //寻找具有最多相同角度的线段
	// //vector<vector<Line>> L_Group;
	// L_ISO L_Group[5];
	// for(unsigned int i = 0; i < Line_angle.size(); ++i)
	// {
	// 	bool emp = true;
	// 	for(unsigned int j = 0; j < 5; ++j)
	// 	{
	// 		if(L_Group[j].num == 0) continue;
	// 		else if(abs(Line_angle[i].angle() - L_Group[j].L[0].angle()) < 5)
	// 		{
	// 			emp = false;
	// 			L_Group[j].L.push_back(Line_angle[i]);
	// 			L_Group[j].num++;
	// 		}
	// 	}
	// 	if(emp)
	// 	{
	// 		for(unsigned int j = 0; j < 5; ++j)
	// 		{
	// 			if(L_Group[j].num == 0)
	// 			{
	// 				L_Group[j].L.push_back(Line_angle[i]);
	// 				L_Group[j].num++;
	// 				break;
	// 			}
	// 		}
	// 	}
	// }
	// int max_num = 0;
	// for(unsigned int i = 0; i < 5; ++i)
	// {
	// 	if(L_Group[i].num >= L_Group[max_num].num)
	// 		max_num = i;
	// 	//cout << L_Group[i].num << " ";
	// }
	// //cout<<endl;
	// //cout << L_Group[max_num].num << endl;
	// Line_length = L_Group[max_num].L;
    // // for(unsigned int i = 0; i < Line_angle.size(); ++i)
	// // {
	// // 	bool same = false;
	// // 	//cout << Line_angle[i].angle() << endl;
	// // 	for(unsigned int j = 0; j < L_Group.size(); ++j)
	// // 	{
	// // 		bool emp = false;
	// // 		for(unsigned int k = 0; k < L.Group[j].size(); ++k)
	// // 		{
	// // 			if(i == j)
	// // 			continue;
	// // 			if(abs(Line_angle[i].angle() - L.Group[j][k].angle()) < 10)
	// // 			{
	// // 				emp = true;	
	// // 				break;
	// // 			}
	// // 		}
	// // 		if(!emp)
	// // 		L_Group[j].push_back()
			
	// // 	}
	// // 	if(!same)
	// // 		{
	// // 			L_Group.push_back(Line_angle[i]);//存储具有相同角度的所有线段
	// // 			//line(img, Line_angle[i].start, Line_angle[i].end, Scalar(50,200,50), 2, CV_AA);
	// // 		}
	// // }
	
	//计算打击点坐标
	if(Line_length.size() > 0)
	{
		int max_loc = 0;
		//寻找最长线段
		for(unsigned int i = max_loc; i < Line_length.size(); ++i)
		{
			if(Line_length[i].length() > Line_length[max_loc].length())
				max_loc = i;
		}

		//添加SVM
		/*if(StartRecti<100&&StartRecti>=0)
		{
			//cout<<"lihhao: "<<Obconii__[max_loc]<<endl;
			//cout<<"zhangyan: "<<Obconi[StartRecti]<<endl;
			if (Obconii__[max_loc] == Obconi[StartRecti])
			{*/

				attack_target = Line_length[max_loc];
				//attack_target.end.x = attack_target.start.x + R_LEN * cos(attack_target.angle_*PI/180);
				//attack_target.end.y = attack_target.start.y + R_LEN * sin(attack_target.angle_*PI/180);
				//cout << attack_target.angle() << endl;
				line(img, attack_target.start, attack_target.end, Scalar(50, 200, 50), 2, CV_AA);
				Find_Attack(img, attack_target, FLIGHT_TIME);
				double distance = Solvepnp(boundRect_p1[max_loc]);
				distance = 8;
				if (data_send) 
				{
					Angle = CalAngle(K_Array, attack_center, distance);
					
					char cmd = 'd';
                    *(signed short *)&port.buff_w_[0] = (Angle.x)/10000;
                    *(signed short *)&port.buff_w_[2] = -(Angle.y)/10000;
                    *(signed short *)&port.buff_w_[4] = 1;
            	//	*(signed short*)&port.buff_w_[6] = armor.depth_;
            		if(port.SendBuff(cmd, port.buff_w_, 6))
            		{
						cout << "发送成功" << "   ";
					}
					cout << "云台移动角度  " << "Yaw:" << Angle.x << " ";
					cout << "Pitch:" << Angle.y << endl;
				}
				/*
					else
					{
						char cmd = 'd';
						*(float*)&port.buff_w_[0] = 0;
            			*(float*)&port.buff_w_[2] = 0;
			 			*(float*)&port.buff_w_[4] = 0;
						 if(port.SendBuff(cmd, port.buff_w_, 6))
						{
							cout << "发送成功" << "   ";
						}
						cout << "云台移动角度  " << "Yaw:" << 0 << " ";
						cout << "Pitch:" << 0 << endl;
					}
				 */
					
		//添加SVM
			/*}
			else
            cout<<"No"<<endl;
		}*/
	}
}

//测距
double WindMill::Solvepnp(RotatedRect target)
{
	Mat rvec, tvec;
    vector<Point3f>corners1;
    vector<Point2f>observation_points;
    Point2f imagePoints[4];
    //target.points(imagePoints);
	//获取旋转矩形边长
	double wid = target.size.width;
	double hei = target.size.height;
	if(hei >= wid)
	{
		double temp = wid;
		wid = hei;
		hei = temp;
	}

	imagePoints[0].x = 0;
	imagePoints[0].y = 0;
	imagePoints[1].x = imagePoints[0].x;
	imagePoints[1].y = imagePoints[0].y - hei;
	imagePoints[2].x = imagePoints[1].x + wid;
	imagePoints[2].y = imagePoints[1].y;
	imagePoints[3].x = imagePoints[2].x;
	imagePoints[3].y = imagePoints[2].y + hei;

     for(int i = 0; i < 4; i++)//大装甲
    {
        Point3f tmp;
        tmp.x = corners_big[i][0];
        tmp.y = corners_big[i][1];
        tmp.z = 0;
        corners1.push_back(tmp);
    }
    for(int i = 0; i < 4; i++)
    {
        observation_points.push_back(imagePoints[i]);
    }

    solvePnP(corners1, observation_points, K_, D_, rvec, tvec, false);
    double solvepnp_z = tvec.at<double>(2,0);
    //cout << solvepnp_z << endl;
    return solvepnp_z;
}

//计算云台偏转角度
Point2f WindMill::CalAngle(double K[3][3], Point2f target, double depth)
{
	const double cx = K[0][2];//x方向主点偏移量
	const double cy = K[1][2];//y方向主点偏移量
	const double fx = K[0][0];//焦距参数fx
	const double fy = K[1][1];//焦距参数fy
	
	//像素坐标系转换相机坐标系
	int u = target.x;
	int v = target.y;
	double point[3], points[3];
	point[2] = depth;//测量点在相机坐标系Z轴位置
	point[0] = (u - cx) * point[2] / fx;//测量点在相机坐标系X轴位置
	point[1] = (v - cy) * point[2] / fy;//测量点在相机坐标系Y轴位置
	//将坐标原点移动到云台位置
	point[0] += OFFSET_X;
	point[1] += OFFSET_Y;
	point[2] += OFFSET_Z;
	memcpy(points, point, sizeof(point));
	//计算偏转角度
	Point2f angle;
	angle.x = (float)(atan(points[0] / points[2]) / PI * 180);
	angle.y = (float)(atan(points[1] / points[2]) / PI * 180);
	return angle;
}

bool WindMill::Clear()
{
	svm->clear();
    RotatedRect_p1.clear();
    Obpoint.clear();
    Obangle.clear();
    StartRect.clear();
    StartRect_.clear();
	return true;
}

void WindMill::circleLeastFit()
{
    radius = 0.0f;
    double sum_x = 0.0f, sum_y = 0.0f;
    double sum_x2 = 0.0f, sum_y2 = 0.0f;
    double sum_x3 = 0.0f, sum_y3 = 0.0f;
    double sum_xy = 0.0f, sum_x1y2 = 0.0f, sum_x2y1 = 0.0f;
    int N = Circular_points.size();
    for (int i = 0; i < N; i++)
    {
        double x = Circular_points[i].x;
        double y = Circular_points[i].y;
        double x2 = x * x;
        double y2 = y * y;
        sum_x += x;
        sum_y += y;
        sum_x2 += x2;
        sum_y2 += y2;
        sum_x3 += x2 * x;
        sum_y3 += y2 * y;
        sum_xy += x * y;
        sum_x1y2 += x * y2;
        sum_x2y1 += x2 * y;
    }
    double C, D, E, G, H;
    double a, b, c;
    C = N * sum_x2 - sum_x * sum_x;
    D = N * sum_xy - sum_x * sum_y;
    E = N * sum_x3 + N * sum_x1y2 - (sum_x2 + sum_y2) * sum_x;
    G = N * sum_y2 - sum_y * sum_y;
    H = N * sum_x2y1 + N * sum_y3 - (sum_x2 + sum_y2) * sum_y;
    a = (H * D - E * G) / (C * G - D * D);
    b = (H * C - E * D) / (D * D - G * C);
    c = -(a * sum_x + b * sum_y + sum_x2 + sum_y2) / N;
    Circular_center.x = a / (-2);
    Circular_center.y = b / (-2);
    radius = sqrt(a * a + b * b - 4 * c) / 2;
}

/*************************************************
Copyright:     Copyright 2018 Ares
Author:        Lcy
Date:          2018-11-19
Description:   主函数
Version:       1.0.1
History:       1.Date:2019-01-18
                 Author:Lcy
                 Modification:增加了PID调试
**************************************************/
#include <iostream>
#include <opencv2/opencv.hpp>
#include <time.h> // 计算时间要包含的头文件
#include "armor_plate.h"
#include "buff.h"
#include "img_process.h"
#include "serial_port.h"
#include "Wind_Detector.h"
#include <stdio.h>
#include <unistd.h>

#pragma clang diagnostic pushd
#pragma clang diagnostic ignored "-Wmissing-noreturn"

using namespace std;
using namespace cv;

char recieve[3];
//#define imshow /##/imshow

enum MyModel
{
    ShotModel,
    BuffModel,
    SleepModel,
    VideoModel
};

MyModel model; // 模式
Mat frame;
int ShotCameraNo = 0,
    WindCameraNo = 1;
ArmorPlate armor;
VideoCapture cap;
VideoWriter writer;
SerialPort port;//串口发送类
long times = 0;
char V_File[100];
void SPortReciveAnalyze(char*Rbuff, MyModel &model_param, ArmorPlate &arm, VideoCapture &buff, SerialPort &port);//判断是否进行那种模式
bool CameraReadShot(MyModel model_param, ArmorPlate& armor_param); // 摄像头读取数据
bool CameraReadBuff(MyModel model_param, VideoCapture& buff_cap, WindMill &buff_param);
bool WindCameraInit(int device, VideoCapture &buffcap);

/*************************************************
Function:       main
Description:    主函数
Input:
Output:
Return:
Others:
*************************************************/
int main()
{

    int t = 0;
    recieve[1] = 'd';
	//装甲识别类

    //WindMill wind;

	armor.CameraInit(ShotCameraNo); // 视觉辅助摄像头初始化
	//armor.Init_cap_V4();

    WindCameraInit(WindCameraNo, cap);

	//while (!port.PortInit(0, 115200)); // 初始化串口1
	model = BuffModel; // 默认为视觉辅助模式
    //port.ThreadInit(); // 初始化一个读取线程
    Mat frasleep;
    //clock_t start=0, end;



	while (1) {
        //start = clock();           /*记录起始时间*/
        // 采集图像
        //armor.cap>>armor.armor_image_;
        /*if(armor.armor_image_.empty())
        {
            cout<<"视觉辅助摄像头没有读取到数据！"<<endl;
        }*/
        //SPortReciveAnalyze(port.buff_r_,armor,model);

        SPortReciveAnalyze(recieve, model, armor, cap, port);
        //cout << recieve << endl;
        if (model == ShotModel) {
            //cout << times++ << endl;
            if (!CameraReadShot(model, armor))
                continue;
            armor.AutoShoot();//得到了我们计算出来的角度
            char cmd = 'c';

            /*cap >> frame;
            //vector<Mat> BGR_channels;
            //split(frame, BGR_channels);
            imshow("wind",frame);*/

            /*if(fabs(armor.ang_p_/0.4)<2&&fabs(armor.ang_y_/0.6)<2)
            {
                *(signed short*)&port.buff_w_[0] = 0;
                *(signed short*)&port.buff_w_[2] = 0;
                *(signed short*)&port.buff_w_[4] = 1;
            }
            else
            {*/
            *(signed short *) &port.buff_w_ = -3 * (armor.ang_y_);
            *(signed short *) &port.buff_w_[2] = 3 * (armor.ang_p_);
            *(signed short *) &port.buff_w_[4] = armor.shoot_flag_;
            //}
            //*(signed short*)&port.buff_w_[0] = (armor.ang_y_/0.3);//0.5
            //*(signed short*)&port.buff_w_[2] = (armor.ang_p_/0.6);
            //*(signed short*)&port.buff_w_[4] = armor.shoot_flag_;
            //*(signed short*)&port.buff_w_[4] = 1;
            //*(signed short*)&port.buff_w_[6] = armor.depth_
            if (port.SendBuff(cmd, port.buff_w_, 6)) {
                //cout << "发送成功" << endl;
                //cout << "w: " << port.buff_w_ << endl;
            }
            //cout << "Y " << armor.ang_y_ << endl;
            //cout << "P " << armor.ang_p_ << endl;
            //cout<<"shot: "<<armor.shoot_flag_<<endl;
            waitKey(1);
        }
        else if (model == BuffModel)
        {
            WindMill wind;
            while (recieve[1] == 'd')
                //while(1)
            {
                SPortReciveAnalyze(recieve, model, armor, cap, port);
                wind.Clear();
                if (!CameraReadBuff(model, cap, wind))
                    continue;
//                cap >> wind.image;
//                if (wind.image.empty())
//                {
//                    cout << "读取图像失败" << endl;
//                    cap.open(1);
//                    break;
//                }
                //frame = wind.image(Rect(250,2,1024,720));
                if (wind.image.data && wind.image.cols > 0 && wind.image.rows > 0)
                {
                    frame = wind.image;
                    cout << frame.cols << " " << frame.rows << endl;
                    imshow("Camera", frame);
                    if (!wind.R_point)
                    {
                        if (wind.preframe <= 102)
                        {
                            if (wind.preframe % 3 == 0)
                            {
                                wind.R_Circular(frame, "Windpre");
                                cout << wind.preframe << endl;
                            }

                            if (wind.preframe == 102)
                            {
                                if(wind.Circular_points.size() > 2)
                                {
                                    wind.circleLeastFit();
                                }
                                else
                                {
                                    wind.preframe = 0;
                                    //cout << "??" << endl;
                                }
                            }
                            wind.preframe++;
                        }
                        else
                            wind.R_SEL(frame, "Windpre");
                    }
                    else
                    {
                        wind.Wind_Detector(frame, "Windpre", port);
                        wind.InitCali(frame);
                        //t++;
                        //cout << t << endl;
                    }
                    imshow("Last", frame);
                    waitKey(1);
                    //sleep(1);
                }

        }
    }
		else if(model == VideoModel)
		{

            int num = 0;

            FILE *vi;
            //视频写入对象
            memset(V_File, '\0', sizeof(V_File));
            //写入视频文件名
            sprintf(V_File, V_NAME, num);
            while((vi=fopen(V_File,"rb"))!=NULL)
            {
                num ++;
                sprintf(V_File, V_NAME, num);
            }

            //设置帧的宽高
            const Size S(WINDWIDTH, WINDHEIGHT);
            //获得帧率
            //double r = captrue.get(CV_CAP_PROP_FPS);
            //打开视频文件，准备写入
            writer.open(V_File, CV_FOURCC('M', 'J', 'P', 'G'), 24, S, true);
            //打开失败*/
            /*if (!cap.isOpened())
            {
                cap.open(1);
            }*/
            bool stop = false;
            //Mat frame;
            //循环
            while (recieve[1] == 'v')
            //while (!stop)
            {

                //读取帧
                Mat frame2;
                //if (!cap.read(frame2))
                 //   break;
                 cap >> frame2;
                if(!frame2.data)
                    break;
                //else
                //cap >> frame2;
                //cout << frame2.cols << " " <<frame2.rows <<endl;
                //imshow("Video", frame2);
                //写入文件
                writer.write(frame2);
                waitKey(10);
                /*if (waitKey(10) > 0)
                {
                    stop = true;
                }*/
            }
            SPortReciveAnalyze(recieve, model, armor, cap, port);
            //释放对象
            //cap.release();
            //writer.release();
            //cvDestroyWindow("Video");

        }
		else
        {

            /*armor.capture_armor_ >> frasleep;
            if(frasleep.data)
            imshow("Sleep", frasleep);
            waitKey(1);*/
            //usleep(5000);
		}

//        end = clock();           /*记录结束时间*/
//        double seconds = (double)(end - start)/CLOCKS_PER_SEC;
//        armor.t1_  = armor.t2_;
//        armor.t2_ = seconds;
//
//        double fps  = 1/seconds;

        //cout << "fps" << fps << endl;
        /*flag++;
        cout<<flag<<endl;*/
        /*
        char key = waitKey(1);//100
        switch (key)
        {
            case 'q': armor.p_kp_ += 0.001; break;
            case 'a': armor.p_kp_ -= 0.001; break;
            case 'w': armor.p_ki_ += 0.001; break;
            case 's': armor.p_ki_ -= 0.001; break;
            case 'e': armor.p_kd_ += 0.001; break;
            case 'd': armor.p_kd_ -= 0.001; break;
            case 'r': armor.y_kp_ += 0.001; break;
            case 'f': armor.y_kp_ -= 0.001; break;
            case 't': armor.y_ki_ += 0.001; break;
            case 'g': armor.y_ki_ -= 0.001; break;
            case 'y': armor.y_kd_ += 0.001; break;
            case 'h': armor.y_kd_ -= 0.001; break;
            case 'z': std::ofstream outFile;
                      // 打开文件
                      outFile.open("Test.txt");
                      outFile << "p_kp_:" << armor.p_kp_ << endl;
                      outFile << "p_ki_:" << armor.p_ki_ << endl;
                      outFile << "p_kd_:" << armor.p_kd_ << endl;
                      outFile << "y_kp_:" << armor.y_kp_ << endl;
                      outFile << "y_ki_:" << armor.y_ki_ << endl;
                      outFile << "y_kd_:" << armor.y_kd_ << endl;
                      //关闭文件
                      outFile.close();
                      break;
        }
         */
	}
	//return 0;
}
/*************************************************
 * Function:SPortReciveAnalyze
 * Description:判断模式
 * Input:model armor_param Rbuff
 * Output:void
 *************************************************/
void SPortReciveAnalyze(char*Rbuff, MyModel &model_param, ArmorPlate &arm, VideoCapture &buff, SerialPort &port)
{
    static int cnt=0;
    //recieve[1] = waitKey(50);
    //cout << recieve[1] << endl;
    //cout << (char)Rbuff[1] <<endl;
    switch((char)Rbuff[1])
    {
        case 'c':
            if(model != ShotModel && model != SleepModel)
            {
                buff.release();
                destroyAllWindows();

                arm.CameraInit(ShotCameraNo);

                //cout<< "Shot Now!" << endl;
            }
            model_param=ShotModel;

            break;
        case 'd':
            if(model != BuffModel && model != VideoModel)
            {
                arm.capture_armor_.release();
                destroyAllWindows();
                WindCameraInit(WindCameraNo, buff);
                //cout<< "Buff Now!" <<endl;
            }
            model_param=BuffModel;
            break;
        case 'v':
            if(model != BuffModel && model != VideoModel)
            {
                arm.capture_armor_.release();
                destroyAllWindows();

                WindCameraInit(WindCameraNo, buff);
                //cout<< "Dec Now!" <<endl;
            }
            model_param=VideoModel;
            break;
        case 'z':
            if(model != ShotModel && model != SleepModel)
            {
                buff.release();
                destroyAllWindows();
                arm.CameraInit(ShotCameraNo);

                //cout<< "Sleep Now!" <<endl;
            }
            model_param=SleepModel;
            break;
        default:
            break;

    }

}
/*************************************************
Function:       CameraRead
Description:    摄像头获取数据
Input:          model armor_plate buff
Output:         src
Return:         false or true
Others:         bool 保证在获取到数据的前提下程序继续执行
*************************************************/
bool CameraReadShot(MyModel model_param, ArmorPlate& armor_param)
{
    bool flag = true;
    if (model_param == ShotModel)
    {
        //armor_param.capture_armor_.read(armor_param.armor_image_);
        armor_param.capture_armor_ >> armor_param.armor_image_;
        if (!armor_param.armor_image_.data)
        {
            cout << "视觉辅助摄像头没有读取到数据！" << endl;
            armor_param.CameraInit(ShotCameraNo); // 视觉辅助摄像头初始化
            flag = false;
        }
        //cout << armor_param.armor_image_.rows << armor_param.armor_image_.cols << endl;
    }
    return flag;
}

bool CameraReadBuff(MyModel model_param, VideoCapture& buff_cap, WindMill &buff_param)
{
    bool flag = true;
    if (model_param == BuffModel)
    {
        //buff_cap.read(buff_param.image);
        buff_cap >> buff_param.image;
        if (!buff_param.image.data)
        {
            cout << "buff摄像头没有读取到数据！" << endl;
            WindCameraInit(WindCameraNo,buff_cap);
            flag = false;
        }
    }
    return flag;
}

bool WindCameraInit(int device, VideoCapture &buffcap)
{
    buffcap.open(VIDEOLOC);
    if (!buffcap.isOpened())
    {
        printf("大符摄像头打开失败！\n");
        return false;
    }
    else
    {
        // 设置摄像头参数
        buffcap.set(CAP_PROP_AUTO_EXPOSURE, 0.25); // where 0.25 means "manual exposure, manual iris"
        buffcap.set(CV_CAP_PROP_EXPOSURE, WINDEXPOSURE); // 曝光
        //buffcap.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));
        buffcap.set(CV_CAP_PROP_FRAME_WIDTH, WINDWIDTH);
        buffcap.set(CV_CAP_PROP_FRAME_HEIGHT, WINDHEIGHT);
        //cout << CV_CAP_PROP_FRAME_WIDTH << endl;
        //cout <<CV_CAP_PROP_FRAME_HEIGHT<< endl;
        return true;
    }
}
#pragma clang diagnostic pop


//int main() {
///* 串口测试部分代码*/
//    SerialPort port;//串口发送类
//    while (!port.PortInit(0, 115200));
//    while (1) {
//        /*
//        char cmd = 'c';
//        *(signed short *) &port.buff_w_[0] = 1;
//        *(signed short *) &port.buff_w_[2] = 2;
//        *(signed short *) &port.buff_w_[4] = 3;
//        if (port.SendBuff(cmd, port.buff_w_, 6)) {
//            //port.Write("1\n");
//            cout << "发送成功" << endl;
//            cout << "w: " << port.buff_w_ << endl;
//        }*/
//
//        port.ReceiveBuff(port.buff_l_, port.buff_r_);
//
//        cout << "r: " << port.buff_r_[0] << " "
//             << port.buff_r_[1] << " "
//             << (int) port.buff_r_[2] << " "
//             << (int) port.buff_r_[3] << " "
//             << port.buff_r_[4] << " "
//             << port.buff_r_[5] << " "
//             << (int) port.buff_r_[6] << " "
//             << (int) port.buff_r_[7] << endl;
//
//        /*
//        for (int i = 0; i < COM_BUFF_LEN; i++) {
//            port.buff_l_[i] = 0;
//            port.buff_w_[i] = 0;
//            port.buff_r_[i] = 0;
//        }*/
//
//        waitKey(1);
//    }
//
//}
/*************************************************************************
Copyright:     Copyright 2018 Ares
Author:        Lcy
Date:          2018-11-26
Description:   串口通信
Version:       1.0.5
History:       1.Date:2018-12-09
                 Author:Lcy
                 Modification:解决了反弹与需要回车的问题 具体为设置为原始模式
               2.Date:2018-12-10
                 Author:Lcy
                 Modification:新增了线程创建 待解决问题，因为进程的回调函数设置为静态，无法直接访问非静态成员
                              采用解决方案4，但目前没有解决 进程卡住 解决方案网址 https://www.cnblogs.com/rickyk/p/4238380.html
                              如果不访问静态成员则不会卡住
               3.Date:2018-12-16
                 Author:Lcy
                 Modification:遗留问题，CR16验证无法通过\
               4.Date:2019-01-10
                 Author:Lcy
                 Modification:与去年步兵通信，成功控制云台
               5.Date:2019-01-18
                 Author:Lcy
                 Modification:与新步兵通信，成功控制云台
**************************************************************************/
#include <iostream>
#include <unistd.h>          // Unix 标准函数定义
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>           // 文件控制定义
#include <termios.h>         // PPSIX 终端控制定义
#include <string.h>
#include <pthread.h>
#include "serial_port.h"
#include <opencv2/opencv.hpp>

//#include <mutex>

using namespace std;
//#define rrr recieve
extern char recieve[3];
extern long times;
/*************************************************
Function:       SerialPort
Description:    构造函数
Input:
Output:
Return:
Others:         初始化变量 创建接收线程
*************************************************/
SerialPort::SerialPort()
{

    for (int i = 0; i < COM_BUFF_LEN; i++)
    {
        buff_w_[i] = 0;
        buff_r_[i] = 0;
    }
}
/*************************************************
Function:       OpenDev
Description:    打开串口
Input:          device
Output:
Return:         fd or -1
Others:         LINUX中 open 函数作用：打开和创建文件
                O_RDONLY 只读打开  O_WRONLY 只写打开  O_RDWR 读，写打开
                对于串口的打开操作，必须使用O_NOCTTY参数，它表示打开的是一个终端设备，程序不会成为该端口的控制终端。如果不使用此标志，任务的一个输入(比如键盘终止信号等)都会影响进程
                O_NDELAY表示不关心DCD信号所处的状态（端口的另一端是否激活或者停止）
                O_NONBLOCK 设置为非阻塞模式，在read时不会阻塞住，在读的时候将read放在while循环中
*************************************************/
int SerialPort::OpenDev(const char *dev)
{
    int fd = open(dev, O_RDWR | O_NOCTTY | O_NDELAY|FNONBLOCK); //| O_NONBLOCK);
    if (fd == -1)
        cout << "串口打开失败" << endl;
    else
        tcflush(fd, TCIOFLUSH);   // 清空输入输出缓存

    return (fd);
}

/*************************************************
Function:       SetSpeed
Description:    设置波特率
Input:          fd speed
Output:
Return:         true or false
Others:
*************************************************/
unsigned int speed_arr[] = {B115200, B57600, B38400, B19200, B9600, B4800, B2400, B1200, B300};
unsigned int name_arr[] = {115200, 57600, 38400, 19200, 9600, 4800, 2400, 1200, 300};
bool SerialPort::SetSpeed(int fd, int speed)
{
    int status;
    struct termios options;
    tcgetattr(fd, &options);
    for (int i= 0; i < sizeof(speed_arr) / sizeof(int); i++)
    {
        if (speed == name_arr[i])
        {
            tcflush(fd, TCIOFLUSH);
            cfsetispeed(&options, speed_arr[i]);//输入波特率
            cfsetospeed(&options, speed_arr[i]);//输出波特率
            options.c_cflag |= (CLOCAL | CREAD);//本地控制模式 //保证程序不会成为端的占有者//使端口能读取输入的数据
            status = tcsetattr(fd, TCSANOW, &options);//设置串口文件
            if (status != 0)
            {
                cout << "波特率设置失败" << endl;
                return false;
            }
        }
    }
    return true;
}
/*************************************************
Function:       SetParity
Description:    设置串口数据位，效验和停止位
Input:          fd         类型 int  打开的串口文件句柄
                data_bits  类型 int  数据位 取值 为 7 或者 8
                parity     类型 char 效验类型 取值为 N, E, O, S
                stop_bits  类型 int  停止位 取值为 1 或者 2
Output:
Return:
Others:
*************************************************/
bool SerialPort::SetParity(int &fd, int data_bits, char parity, int stop_bits)
{
    struct termios options;

    if (tcgetattr(fd, &options) != 0)
    {
        cout << "串口设置失败" << endl;
        return false;
    }
    options.c_cflag &= ~ CSIZE;
    switch (data_bits)
    {
        case 7:
            options.c_cflag |= CS7;
            break;
        case 8:
            options.c_cflag |= CS8;
            break;
        default:
            cout << "不支持的数据位" << endl;
            return false;
    }
    switch (parity)
    {
        case 'n':
        case 'N':
            options.c_cflag &= ~PARENB; // Clear parity enable
            options.c_iflag &= ~INPCK; // Enable parity checking
            break;
        case 'o':
        case 'O':
            options.c_cflag |= (PARODD | PARENB); // 设置为奇效验
            options.c_iflag |= INPCK; // Disable parity checking
            break;
        case 'e':
        case 'E':
            options.c_cflag |= PARENB; // Enable parity
            options.c_cflag &= ~PARODD; // 转换为偶效验
            options.c_iflag |= INPCK; // Disable parity checking
            break;
        case 's':
        case 'S': // as no parity
            options.c_cflag &= ~PARENB;
            options.c_cflag &= ~CSTOPB;
            break;
        default:
            cout << "不支持的校验类型" << endl;
            return false;
    }
    switch (stop_bits)
    {
        case 1:
            options.c_cflag &= ~CSTOPB;
            break;
        case 2:
            options.c_cflag |= CSTOPB;
            break;
        default:
            cout << "不支持的停止位" << endl;
            return false;
    }
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // 设置为原始模式
    options.c_oflag &= ~ OPOST;
/*  直接设置
    options.c_iflag = 0;
    options.c_oflag = 0;
    options.c_cflag = 2237; // 6322-115200 2237-9600
    options.c_lflag = 0;
    options.c_cc[VTIME] = 0; // 等待时间
    options.c_cc[VMIN] = 0; // 最小接收字符
*/  //fcntl(fd,F_SETFL,FNDELAY);
    if (tcsetattr(fd, TCSANOW, &options) != 0) // Update the options and do it NOW
    {
        cout << "串口错误" << endl;
        return false;
    }

    cout << options.c_iflag << endl;
    cout << options.c_oflag << endl;
    cout << options.c_cflag << endl;
    cout << options.c_lflag << endl;
    cout << options.c_ispeed << endl;
    cout << options.c_ospeed << endl;
    return true;
}
/*************************************************
Function:       PortInit
Description:    初始化串口
Input:          串口号 波特率
Output:
Return:         true or false
Others:         bool
*************************************************/
bool SerialPort::PortInit(int device, int speed)
{
    const char *dev;

    if (device == 0)
        dev = "/dev/ttyUSB0"; // ttyUSB0 USB串口
    else
    {
        cout << "串口号错误" << endl;
        return false;
    }

    fd_ = OpenDev(dev);
    if (fd_ == -1)
        return false;

    if (!SetSpeed(fd_, speed))
        return false;
    if (!SetParity(fd_, 8, 'N', 1)) // 数据位 8  校验 无  停止位 1
        return false;

    return true;
}
/*************************************************
Function:       Read
Description:    串口读数据
Input:          buff length
Output:
Return:         true or false
Others:         bool
*************************************************/
bool SerialPort::Read(char *r_buff, size_t length)
{
    read(fd_, r_buff, length);
    tcflush(fd_, TCIFLUSH); // 清空输入队列  TCIFLUSH 输入队列  TCOFLUSH 输出队列  TCIOFLUSH 输入输出队列
    return true;
}
/*************************************************
Function:       Write
Description:    串口写数据
Input:          buff length
Output:
Return:         true or false
Others:         bool
*************************************************/
bool SerialPort::Write(char *w_buff, size_t length)
{
    //cout << length << endl;
    write(fd_, w_buff, length);
    //tcflush(fd_, TCOFLUSH); // 清空输出队列  TCIFLUSH 输入队列  TCOFLUSH 输出队列  TCIOFLUSH 输入输出队列
    return true;
}
/*************************************************
Function:       ISO14443AAppendCRCA
Description:    CRC16 打包
Input:          buff byte_count
Output:
Return:
Others:
*************************************************/
void SerialPort::ISO14443AAppendCRCA(void* buffer, unsigned short byte_count)
{
    unsigned short check_sum = 0x6363;
    unsigned char* data = (unsigned char*)buffer;

    while (byte_count --)
    {
        unsigned char byte = *data++;

        byte ^= (unsigned char)(check_sum & 0x00ff);
        byte ^= byte << 4;

        check_sum = (check_sum >> 8) ^ ((unsigned short)byte << 8) ^ ((unsigned short)byte << 3) ^((unsigned short)byte >> 4);
    }
    *data++ = (check_sum >> 0) & 0x00ff;
    *data = (check_sum >> 8) & 0x00ff;
}
/*************************************************
Function:       ISO14443ACheckCRCA
Description:    CRC16 解包
Input:          buff byte_count
Output:
Return:
Others:
*************************************************/
unsigned char SerialPort::ISO14443ACheckCRCA(void* buffer, unsigned short byte_count)
{
    unsigned short check_sum = 0x6363;
    unsigned char* data = (unsigned char*)buffer;

    while (byte_count --)
    {
        unsigned char byte = *data++;

        byte ^= (unsigned char)(check_sum & 0x00ff);
        byte ^= byte << 4;

        check_sum = (check_sum >> 8) ^ ((unsigned short)byte << 8) ^ ((unsigned short)byte << 3) ^((unsigned short)byte >> 4);
    }
    //cout << ((data[0] == ((check_sum >> 0) & 0xff)) && (data[1] == ((check_sum >> 8) & 0xff))) << endl;
    return (data[0] == ((check_sum >> 0) & 0xff)) && (data[1] == ((check_sum >> 8) & 0xff));
}
/*************************************************
Function:       ISO14443AAppendCRCA
Description:    CRC16 数据长度检验
Input:          buff
Output:
Return:
Others:
*************************************************/
bool SerialPort::ISO14443ACheckLen(unsigned  char* buffer)
{
    if ((buffer[0] + buffer[1] == 0x7f) && (buffer[0] < COMEMAND_BUFF_LEN -2))
        return true;
    else
        return true;
}
/*************************************************
Function:       SendBuff
Description:    发送数据包
Input:          buff length
Output:
Return:         true or false
Others:         bool
*************************************************/
bool SerialPort::SendBuff(char command, char *data, unsigned short length)
{
    unsigned char buffer[COMEMAND_BUFF_LEN];
    buffer[0] = 0x55; // 帧头
    buffer[1] = command; // 命令
    buffer[2] = length; // 数据长度
    buffer[3] = 0xff - length; // 数据长度取反
    memcpy(buffer + HEAD_LEN, data, DATA_LEN); // 拷贝数据
    ISO14443AAppendCRCA(buffer, length + HEAD_LEN); // CRC16 打包
    if (Write((char*)buffer, length + HEAD_LEN + 2))
        return true;
    else
        return false;
}
/*************************************************
Function:       ReceiveBuff
Description:    读取数据包
Input:          src_buff dst_buff
Output:
Return:         true or false
Others:         bool
*************************************************/
bool SerialPort::ReceiveBuff(char *src_buff, char *dst_buff)
{
    //cout << "2";
    //usleep(20000);
    Read(src_buff, COM_BUFF_LEN);
    cout << src_buff <<endl;
    cout << times++ <<endl;
    //cout << "3" <<endl;
    if (src_buff[0] == 0x55)
    {
        recieve[0] = src_buff[0];
        if (ISO14443ACheckLen((unsigned char*)(src_buff + HEAD_LEN -2)))
        {
           /* if (ISO14443ACheckCRCA(src_buff, (unsigned short)(src_buff[2] + HEAD_LEN)))
            {*/

                for (int i = 0; i < DATA_LEN; i++)
                    dst_buff[i] = src_buff[i];
                //some_mutex.lock();
                recieve[1] = src_buff[1];
                //usleep(200000);
                //cout << dst_buff << endl;

                //cout << (int)dst_buff[4] <<endl;
                //some_mutex.unlock();
                //cv::waitKey(50);

                return true;
           /* }
            else
            {
                //cout << "error3" << endl;
                for (int i = 0; i < DATA_LEN; i++)
                    //dst_buff[i] = 0;
                return false;
            }*/
        }
        else
        {
            cout << "error2" << endl;
            for (int i = 0; i < DATA_LEN; i++)
                dst_buff[i] = 0;
            return false;
        }
    }
    else
    {
        //cout << "error1" << endl;
        for (int i = 0; i < DATA_LEN; i++)
            dst_buff[i] = 0;
        return false;
    }

}
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"
/*************************************************
Function:       ReadThread
Description:    读取线程函数
Input:
Output:
Return:
Others:         在线程里读取串口信息
*************************************************/
void* SerialPort::ReadThread(void *pData)
{
    //PortInit(0, 115200);
    SerialPort *p_port = reinterpret_cast<SerialPort*>(pData);
    cout << "线程已进入" << endl;
    while (!p_port->PortInit(0, 115200));
    while (1)
    {
        //cout << p_port->fd_ <<endl;
        //cout << "线程已进入" << endl;
        if(p_port->fd_ != -1)
        {
             //cout << "1";
             p_port->ReceiveBuff(p_port->buff_l_, p_port->buff_r_);
             //usleep();
            //cout << "2" <<endl;
            //cin >> recieve[1];
            //cout << recieve[1] <<endl;
            //sleep(10);
             //cout<< p_port->buff_r_<<endl;
             //cv::waitKey(5);
        }
        else
        {
            cout << p_port->fd_ << endl;
            cout << "guale" <<endl;
            break;
        }
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
int SerialPort::StartThread(void *(*func) (void *), pthread_t* pthread, void* par)
{
    memset(pthread, 0, sizeof(pthread_t));
    int temp;
    /*创建线程*/
    if((temp = pthread_create(pthread, NULL, func, par)) != 0)
        cout << "线程创建失败!" << endl;
    else
    {
        //PortInit(0, 115200);
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
int SerialPort::StopThread()
{
    //cout << "正在退出线程" << *pthread << "！" << endl;
    cout << "正在退出线程" << read_thread_ << "！" << endl;
    if(read_thread_ !=0)
        pthread_join(NULL, NULL);

    cout << "线程" << read_thread_ << "已退出！" << endl;
}
/*************************************************
Function:       ThreadInit
Description:    初始化一个线程 读取线程
Input:
Output:
Return:         true or false
Others:         bool
*************************************************/
bool SerialPort::ThreadInit() {
    //pthread_t thread;
    cout << pthread_self() << endl;
    int COM_READ_STATU = 0;
    if (StartThread(ReadThread, &read_thread_, &COM_READ_STATU) != 0) {
        cout << "线程创建失败" << endl;
        return false;
    }
    return true;
}
                                                                                                                                    #include "mainwindow.h"
#include "ui_mainwindow.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include "zcan.h"
#include <QTimer>
#include<QDebug>
#include <QScreen>
#include "LED.h"
#include "agv.h"

//aubo_robot
#include <iostream>
// 机械臂服务器地址（默认值，可通过 config.ini 覆盖）
#define SERVER_HOST "192.168.1.10"  // TODO: 部署时修改为实际IP
#define SERVER_PORT 8899
#define ROBOT_I16 8
ServiceInterface robotService;

//传感器
#define H2_SERIAL_PORT_NAME   "/dev/ttyS0"
#define H2_BAUD_RATE          9600      // 波特率
#define H2_DATA_BITS          QSerialPort::Data8 // 数据位
#define H2_PARITY             QSerialPort::NoParity // 校验位
#define H2_STOP_BITS          QSerialPort::OneStop  // 停止位

//ROS
#include <QProcess>
#include <QMessageBox>
#include <QDateTime>
#include <QTextCursor>
#include <QTextCharFormat>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

//zed
#include <filesystem>
DetectionParams detectionParams = { 33, 28, 67, 133, 208, 64 };

//文件保存
#include <QFile>
#include <QTextStream>

// 在类定义外部添加全局变量（简单实现）
actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>* ac = nullptr;

MainWindow::MainWindow(QWidget *parent): QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    //AGV
    ui->agv_battery->setRange(0, 100); // 电量范围0-100%
    ui->agv_battery->setValue(0);     // 初始值设为0
    ui->agv_battery->setFormat("电量：%p%"); // 显示格式：电量：XX%
    ui->agv_battery->setAlignment(Qt::AlignCenter); // 文本居中
    this->setWindowTitle("加氢站巡检机器人");
    m_forwardTimer = new QTimer(this);
    m_leftTimer = new QTimer(this);
    m_backTimer = new QTimer(this);
    m_rightTimer = new QTimer(this);
    m_stopTimer = new QTimer(this);
    m_autoSequenceTimer = new QTimer(this);

    connect(m_forwardTimer, &QTimer::timeout, this, &MainWindow::sendForward);
    connect(m_leftTimer, &QTimer::timeout, this, &MainWindow::sendLeft);
    connect(m_backTimer, &QTimer::timeout, this, &MainWindow::sendBack);
    connect(m_rightTimer, &QTimer::timeout, this, &MainWindow::sendRight);
    connect(m_stopTimer, &QTimer::timeout, this, &MainWindow::stopMove);
    connect(m_autoSequenceTimer, &QTimer::timeout, this, &MainWindow::onAutoSequenceTimeout);
    connect(this, &MainWindow::batteryLevelUpdated, this, &MainWindow::updateBatteryLevel);
    initAutoActionSequence(); // 

    //aubo_robot
    m_posTimer = new QTimer(this);
    connect(m_posTimer, &QTimer::timeout, this, &MainWindow::updatePositionDisplay);
    connect(ui->aubo_speed_joint, &QSlider::valueChanged, this, &MainWindow::aubo_speed);

    //角度初始化
    for (int i = 0; i < 6; ++i) {
        currentJointAngles[i] = 0.0;
    }
    // ========== 初始化上电进度条 ==========
    powerOnProgress = new QProgressDialog("机械臂上电中...", "", 0, 100, this);
    powerOnProgress->setWindowTitle("上电进度");
    powerOnProgress->setWindowModality(Qt::ApplicationModal);  // 模态窗口（禁止操作其他UI）
    powerOnProgress->setCancelButton(nullptr);  // 禁用取消按钮（如果需要保留取消，删除这行）
    powerOnProgress->close();  // 初始隐藏
    powerstarts();

    // ========== 初始化进度更新定时器 ==========
    progressTimer = new QTimer(this);
    progressTimer->setInterval(150);  // 每80ms更新一次进度（速度可调整）
    connect(progressTimer, &QTimer::timeout, this, &MainWindow::updatePowerOnProgress);

    //电源检测
    m_powerCheckTimer = new QTimer(this);
    m_powerCheckTimer->setInterval(3000); // 每5秒检测一次
    connect(m_powerCheckTimer, &QTimer::timeout, this, &MainWindow::powerstarts);
    m_powerCheckTimer->start();

    //传感器
    serial = new QSerialPort(this);
    serial1 = new QSerialPort(this);

    H2Timer = new QTimer(this);
    FlameTimer = new QTimer(this);

    // 配置串口参数（必须与传感器一致，参考手册）
    serial->setPortName("/dev/ttyS4");   // 氢气传感器串口设备名，工控机上从0开始编号
    serial->setBaudRate(QSerialPort::Baud9600);  // 波特率（常见 9600、115200）
    serial->setDataBits(QSerialPort::Data8);      // 数据位 8
    serial->setStopBits(QSerialPort::OneStop);    // 停止位 1
    serial->setParity(QSerialPort::NoParity);     // 无校验
    serial->setFlowControl(QSerialPort::NoFlowControl);  // 无流控

    serial1->setPortName("/dev/ttyS0");   // 火焰传感器串口设备名
    serial1->setBaudRate(QSerialPort::Baud9600);  // 波特率（常见 9600、115200）
    serial1->setDataBits(QSerialPort::Data8);      // 数据位 8
    serial1->setStopBits(QSerialPort::OneStop);    // 停止位 1
    serial1->setParity(QSerialPort::NoParity);     // 无校验
    serial1->setFlowControl(QSerialPort::NoFlowControl);  // 无流控

    connect(serial, &QSerialPort::readyRead, this, &MainWindow::readH2SerialData);
    connect(H2Timer, &QTimer::timeout, this, &MainWindow::H2write);
    connect(serial1, &QSerialPort::readyRead, this, &MainWindow::readFlameSerialData);
    connect(FlameTimer, &QTimer::timeout, this, &MainWindow::Flamewrite);

    robotService.robotServiceInitGlobalMoveProfile();
    //设置关节运动最大加速度
    aubo_robot_namespace::JointVelcAccParam m_set_jointacc;
    m_set_jointacc.jointPara[0] = 30*M_PI/180;
    m_set_jointacc.jointPara[1] = 30*M_PI/180;
    m_set_jointacc.jointPara[2] = 30*M_PI/180;
    m_set_jointacc.jointPara[3] = 30*M_PI/180;
    m_set_jointacc.jointPara[4] = 30*M_PI/180;
    m_set_jointacc.jointPara[5] = 30*M_PI/180;
    robotService.robotServiceSetGlobalMoveJointMaxAcc(m_set_jointacc);

    //设置关节运动最大速度
    aubo_robot_namespace::JointVelcAccParam m_set_jointvelc;
    m_set_jointvelc.jointPara[0] = 5*M_PI/180;
    m_set_jointvelc.jointPara[1] = 5*M_PI/180;
    m_set_jointvelc.jointPara[2] = 5*M_PI/180;
    m_set_jointvelc.jointPara[3] = 5*M_PI/180;
    m_set_jointvelc.jointPara[4] = 5*M_PI/180;
    m_set_jointvelc.jointPara[5] = 5*M_PI/180;
    robotService.robotServiceSetGlobalMoveJointMaxVelc(m_set_jointvelc);

    //设置末端型运动最大加速度
    double m_set_end_lineacc = 0.02;//0.2
    robotService.robotServiceSetGlobalMoveEndMaxLineAcc(m_set_end_lineacc);

    //设置末端型运动最大速度
    double m_set_end_linevelc = 0.02;//0.2
    robotService.robotServiceSetGlobalMoveEndMaxLineVelc(m_set_end_linevelc);
    QSlider *speedSlider = new QSlider(Qt::Horizontal);
    QLabel *speedLabel = new QLabel(QString("当前速度：%1").arg(50));
    QObject::connect(speedSlider, &QSlider::valueChanged, [&](int speed)
    {
            // 1. 更新标签显示的速度值
            speedLabel->setText(QString("当前速度：%1").arg(speed));
            // 2. 【核心】这里添加你的速度业务逻辑（比如控制设备/播放速度）

    });
    // 启动roscore
    startRoscore();

    //初始化ros节点
    initRosNode();
    initTfListener();

    //ROS
    addRosProcess("radar", "roslaunch livox_ros_driver2 msg_MID360.launch");//雷达驱动
    addRosProcess("localization", "roslaunch fast_lio_localization sentry_localize.launch");//重定位
    addRosProcess("initpos", "rosrun fast_lio_localization publish_initial_pose.py 0 0 0 0 0 0");//发布初始位姿
    addRosProcess("navigation", "    rosrun agv_navigation demo_simple_goal");//定点导航
    addRosProcess("movebase", "roslaunch sentry_nav sentry_movebase.launch");//move base
    addRosProcess("agv", "roslaunch agv_navigation agv_navigation.launch");//agv
    ui->rosmessage->setPlaceholderText("ROS信息将在此显示...");
    ui->agv_pos->setPlaceholderText("AGV位置信息将在此显示...");
    m_navStatusTimer = new QTimer(this);  // 新增导航状态定时器
    connect(m_navStatusTimer, &QTimer::timeout, this, &MainWindow::updateNavStatusDisplay);  // 新增连接
    
    // 新增：robotstatus内容检查定时器
    m_robotStatusCheckTimer = new QTimer(this);
    connect(m_robotStatusCheckTimer, &QTimer::timeout, this, &MainWindow::checkRobotStatus);
    m_robotStatusCheckTimer->start(500); // 每0.5秒检查一次



    // 注册QTextCursor元类型，解决信号槽连接错误
    qRegisterMetaType<QTextCursor>("QTextCursor");

    // 初始化相机相关成员变量
    cameraTimer = new QTimer(this);
    statusLabel = ui->statusLabel;
    cameraRunning = false;
    saveIndex = 0;

    // 视频流检测初始化
    detectionLabel = ui->detectionLabel;

    // 初始化检测参数（从shipinliu.cpp复制）
    detectionEnabled = true;  // 默认开启检测
    // 创建视频处理线程
    m_videoThread = new VideoProcessingThread(this);
    connect(m_videoThread, &VideoProcessingThread::frameProcessed,
            this, &MainWindow::onVideoFrameProcessed);
    connect(m_videoThread, &VideoProcessingThread::errorOccurred,
            this, &MainWindow::onVideoError);
}

MainWindow::~MainWindow()
{
    delete powerOnProgress;
    delete progressTimer;

    // 清理所有ROS进程
    for (auto it = m_rosProcesses.begin(); it != m_rosProcesses.end(); ++it) {
        RosProcessInfo& info = it.value();  // 使用value()而不是second
        if (info.process->state() == QProcess::Running) {
            info.process->terminate();
            info.process->waitForFinished(5000);
        }
        delete info.process;
    }

    // 清理ROS资源
    if (m_rosInitialized) {
        ros::shutdown();
        if (m_rosThread.joinable()) {
            m_rosThread.join();
        }
    }
    m_powerCheckTimer->stop();
    delete m_powerCheckTimer;

    if (cameraRunning) {
        cameraTimer->stop();
        zedCamera.close();
    }

    // 清理视频处理线程
    if (m_videoThread) {
        m_videoThread->stopProcessing();
        delete m_videoThread;
    }

    delete ui;
}

//can通讯
#define msleep(ms)  usleep((ms)*1000)
#define MAX_CHANNELS  1     // 最大通道数量
#define RX_WAIT_TIME  100
#define RX_BUFF_SIZE  1000

// 接收线程上下文
typedef struct
{
    int dev_type;   // 设备类型
    int dev_idx;    // 设备索引
    int chn_idx;    // 通道号
    int total;      // 接收总数
    int stop;       // 线程结束标志
    MainWindow* mainWindow; // 新增：绑定MainWindow实例
} THREAD_CTX;

// 构建 CAN 帧
void construct_can_frame(ZCAN_20_MSG *can_msg, UINT id, int chn_idx, int pad) {
    memset(can_msg, 0, sizeof(ZCAN_20_MSG));
    can_msg->hdr.inf.txm = 0;     // 0-正常发送
    can_msg->hdr.inf.fmt = 0;     // 0-CAN
    can_msg->hdr.inf.sdf = 0;     // 0-数据帧, 1-远程帧
    can_msg->hdr.inf.sef = 0;     // 0-标准帧, 1-扩展帧
    // can_msg->hdr.inf.echo = 1;    // 发送回显

    can_msg->hdr.id  = id;        // ID
    can_msg->hdr.chn = chn_idx;   // 通道
    can_msg->hdr.len = 8;        // 数据长度

    // 队列发送
    if(pad > 0){
        can_msg->hdr.pad = pad;              // 发送后延迟 pad ms
        can_msg->hdr.inf.qsend = 1;          // 队列发送帧，仅判断首帧
        can_msg->hdr.inf.qsend_100us = 0;    // 队列发送单位，0-ms，1-100us
    }

    for (int i = 0; i < can_msg->hdr.len; i++)
        can_msg->dat[i] = i;
}

// 构建 合并发送 帧
void construct_data_frame(ZCANDataObj *data_msg, UINT id, int chn_idx, int pad){
    memset(data_msg, 0, sizeof(data_msg));
    data_msg->dataType = ZCAN_DT_ZCAN_CAN_DATA;
    data_msg->chnl = chn_idx;
    construct_can_frame(& data_msg->data.zcanCANData, id, chn_idx, pad);
}

// 获取设备信息
void get_device_info(int DevType, int DevIdx) {
    ZCAN_DEV_INF info;
    memset(&info, 0, sizeof(info));
    VCI_ReadBoardInfo(DevType, DevIdx, &info);
    char sn[21];
    char id[41];
    memcpy(sn, info.sn, 20);
    memcpy(id, info.id, 40);
    sn[20] = '\0';
    id[40] = '\0';
    printf("HWV=0x%04x, FWV=0x%04x, DRV=0x%04x, API=0x%04x, IRQ=0x%04x, CHN=0x%02x, SN=%s, ID=%s\n",
        info.hwv, info.fwv, info.drv, info.api, info.irq, info.chn, sn, id);
}

// 接收线程
void *rx_thread(void *data) {
    THREAD_CTX *ctx = (THREAD_CTX *)data;
    int DevType = ctx->dev_type;
    int DevIdx  = ctx->dev_idx;
    int chn_idx = ctx->chn_idx;

    ZCAN_20_MSG can_data[RX_BUFF_SIZE];
    ZCAN_FD_MSG canfd_data[RX_BUFF_SIZE];

    while (!ctx->stop)
    {
        memset(can_data, 0, sizeof(can_data));
        memset(canfd_data, 0, sizeof(canfd_data));

        int rcount = VCI_Receive(DevType, DevIdx, chn_idx, can_data, RX_BUFF_SIZE, RX_WAIT_TIME);      // CAN
        for (int i = 0; i < rcount; ++i)
        {
            // ==== 安全解析电量并更新UI ====
            // 1. 检查是否为标准数据帧（避免扩展帧或远程帧）
            bool isStandardDataFrame = (can_data[i].hdr.inf.sef == 0) && (can_data[i].hdr.inf.sdf == 0);
            // 2. 检查数据长度≥3（避免访问dat[2]越界）
            bool isDataLengthValid = (can_data[i].hdr.len >= 3);
            // 3. 检查前两字节是否为目标电量帧（0x2C 0x05）
            bool isBatteryFrame = (can_data[i].dat[0] == 0x2C) && (can_data[i].dat[1] == 0x05);

            if (isStandardDataFrame && isDataLengthValid && isBatteryFrame) {
                int batteryLevel = static_cast<int>(can_data[i].dat[2]); // 提取电量值

                // 关键修复：通过Qt元对象系统在主线程更新UI
                if (ctx->mainWindow) { // 确保mainWindow指针有效
                    QMetaObject::invokeMethod(
                        ctx->mainWindow,          // 目标对象（MainWindow实例）
                        "updateBatteryLevel",     // 要调用的槽函数名
                        Qt::QueuedConnection,     // 队列连接（跨线程安全）
                        Q_ARG(int, batteryLevel)   // 传递电量参数
                    );
                }
            }
        }
        ctx->total += rcount;

        rcount = VCI_ReceiveFD(DevType, DevIdx, chn_idx, canfd_data, RX_BUFF_SIZE, RX_WAIT_TIME); // CANFD
        for (int i = 0; i < rcount; ++i)
        {
            printf("[%u] ",canfd_data[i].hdr.ts);
            printf("chn: %d  ", chn_idx);
            printf("%s  ", canfd_data[i].hdr.inf.tx == 1 ? "Tx" : "Rx");  // 判断是否回显报文
            printf("CANFD%s  ", canfd_data[i].hdr.inf.brs == 1 ? "加速" : "");
            printf("ID: 0x%x ", canfd_data[i].hdr.id & 0x1FFFFFFF);
            printf("%s  ", canfd_data[i].hdr.inf.sef == 1 ? "扩展帧" : "标准帧");

            printf("Data: ");
            for (int j = 0; j < canfd_data[i].hdr.len; ++j)
                printf("%02x ", canfd_data[i].dat[j]);
            printf("\n");
        }
        ctx->total += rcount;
        msleep(10);
    }
    printf("chn: %d receive %d\n", chn_idx, ctx->total);
    pthread_exit(0);
}

// 合并接收线程
void *rx_merge_thread(void *data) {
    THREAD_CTX *ctx = (THREAD_CTX *)data;
    int DevType = ctx->dev_type;
    int DevIdx  = ctx->dev_idx;
    //int chn_idx = ctx->chn_idx;

    ZCANDataObj obj_data[RX_BUFF_SIZE];
    ZCAN_20_MSG can_data;
    ZCAN_FD_MSG canfd_data;

    while (!ctx->stop)
    {
        memset(obj_data, 0, sizeof(obj_data));
        memset(&can_data, 0, sizeof(can_data));
        memset(&canfd_data, 0, sizeof(canfd_data));
        int rcount = VCI_ReceiveData(DevType, DevIdx, 0, obj_data, RX_BUFF_SIZE, RX_WAIT_TIME);     // 第三个参数固定为0就好
        for (int i = 0; i < rcount; ++i)
        {
            if(obj_data[i].dataType == ZCAN_DT_ZCAN_CAN_DATA){              // CAN
                can_data = obj_data[i].data.zcanCANData;
                printf("[%u] ",can_data.hdr.ts);
                printf("chn: %d  ", obj_data[i].chnl);
                printf("%s  ", can_data.hdr.inf.tx == 1 ? "Tx" : "Rx");     // 判断是否回显报文
                printf("ID: 0x%X CAN  ", can_data.hdr.id);
                printf("%s  ", can_data.hdr.inf.sef == 1 ? "扩展帧" : "标准帧");

                printf("Data: ");
                if(can_data.hdr.inf.sdf == 0){       // 数据帧
                    for (int j = 0; j < can_data.hdr.len; ++j)
                        printf("%02x ", can_data.dat[j]);
                }
            }
            else if(obj_data[i].dataType == ZCAN_DT_ZCAN_CANFD_DATA){       // CANFD
                printf("[%u] ",obj_data[i].data.zcanCANFDData.hdr.ts);
                printf("chn: %d  ", obj_data[i].chnl);
                printf("%s  ", obj_data[i].data.zcanCANFDData.hdr.inf.tx == 1 ? "Tx" : "Rx");  // 判断是否回显报文
                printf("ID: 0x%x ", obj_data[i].data.zcanCANFDData.hdr.id);
                printf("CANFD%s  ", obj_data[i].data.zcanCANFDData.hdr.inf.brs == 1 ? "加速" : "");
                printf("%s  ", obj_data[i].data.zcanCANFDData.hdr.inf.sef == 1 ? "扩展帧" : "标准帧");

                printf("Data: ");
                for (int j = 0; j < obj_data[i].data.zcanCANFDData.hdr.len; ++j)
                    printf("%02x ", obj_data[i].data.zcanCANFDData.dat[j]);
            }
            printf("\n");
        }
        ctx->total += rcount;
        msleep(10);
    }
    printf("merge receive %d\n", ctx->total);
    pthread_exit(0);
}

void MainWindow::updateBatteryLevel(int level) //电量显示槽函数
{
    // 1. 确保电量值在0-100的有效范围（防止异常值）
    int validBattery = qBound(0, level, 100);

    // 2. 更新你手动添加的agv_battery进度条
    ui->agv_battery->setValue(validBattery);

    // 4. （可选）低电量视觉警告：电量≤20%时进度条变红，否则恢复默认
    if (validBattery <= 20) {
        ui->agv_battery->setStyleSheet(R"(
            QProgressBar {
                border: 1px solid #cccccc;
                border-radius: 5px;
                text-align: center;
            }
            QProgressBar::chunk {
                background-color: #ff4444; /* 低电量红色 */
                border-radius: 4px;
            }
        )");
    } else {
        // 正常电量恢复默认样式（可自定义颜色）
        ui->agv_battery->setStyleSheet(R"(
            QProgressBar {
                border: 1px solid #cccccc;
                border-radius: 5px;
                text-align: center;
            }
            QProgressBar::chunk {
                background-color: #4cd964; /* 正常电量绿色 */
                border-radius: 4px;
            }
        )");
    }
}

// 普通发送
void send_test(int DevType, int DevIdx, int ChIdx){
    // 测试发送
    const int send_num = 10;    // 发送数量
    int send_ret = 0;           // 发送返回值

    // CAN
    ZCAN_20_MSG can_msg[send_num];  // 数组不规范写法，仅供参考
    for (int i = 0; i < send_num; i++)
        construct_can_frame(&can_msg[i], i, ChIdx, 0);
    send_ret = VCI_Transmit(DevType, DevIdx, ChIdx, &can_msg[0], send_num);
    printf("send can frams %d\n", send_ret);

    // 合并发送
    ZCANDataObj data_msg[send_num];
    for (int i = 0; i < send_num; i++)
        construct_data_frame(&data_msg[i], i, ChIdx, 0);
    send_ret = VCI_TransmitData(DevType, DevIdx, ChIdx, &data_msg[0], send_num);
    printf("send frams %d\n", send_ret);
}

// 队列发送
void queue_send(int DevType, int DevIdx, int ChIdx) {
    unsigned int snd_queue_size = 0;    // 队列大小
    unsigned int snd_queue_remain = 0;  // 队列剩余空间

    // 开启队列发送（带LIN版本无需调用）
    VCI_SetReference(DevType, DevIdx, ChIdx, ZCAN_CMD_SET_SEND_QUEUE_EN, (void *)1);

    // 获取队列大小，最多填充个数
    VCI_GetReference(DevType, DevIdx, ChIdx, ZCAN_CMD_GET_SEND_QUEUE_SIZE, &snd_queue_size);

    // 获取队列剩余可填充报文帧数
    VCI_GetReference(DevType, DevIdx, ChIdx, ZCAN_CMD_GET_SEND_QUEUE_SPACE, &snd_queue_remain);
    printf("Chn%d send queue size:%u, remain space:%u\n", ChIdx, snd_queue_size, snd_queue_remain);

    // 发送 10 次100帧的报文
    int transmit_time = 10;
    int transmit_num = 100;
    ZCAN_20_MSG can_msg[100];
    ZCAN_FD_MSG canfd_msg[100];
    while(1){
        // 队列发送前需要判断可填充报文帧数!!
        VCI_GetReference(DevType, DevIdx, ChIdx, ZCAN_CMD_GET_SEND_QUEUE_SPACE, &snd_queue_remain);
        if(snd_queue_remain >= transmit_num){
            for (int i = 0; i < 100; i++)
                construct_can_frame(&can_msg[i], i, ChIdx, 100);   // 10ms

            unsigned ret = VCI_Transmit(DevType, DevIdx, ChIdx, can_msg, 100);        // CAN
            printf("Chn%d remain space:%u, queue send canfd frame %u\n", ChIdx, snd_queue_remain, ret);

            transmit_time--;
            if(transmit_time == 0)
                break;
        }
    }

    sleep(3);

    //发送后，队列空间变化
    VCI_GetReference(DevType, DevIdx, ChIdx, ZCAN_CMD_GET_SEND_QUEUE_SPACE, &snd_queue_remain);
    printf("Chn%d remain space:%u\n", ChIdx, snd_queue_remain);

    //清空队列发送
    VCI_SetReference(DevType, DevIdx, ChIdx, ZCAN_CMD_SET_SEND_QUEUE_CLR, (void *)1);
    printf("Chn%d send queue clear\n", ChIdx);

    //清空队列后，队列空间恢复初始值
    VCI_GetReference(DevType, DevIdx, ChIdx, ZCAN_CMD_GET_SEND_QUEUE_SPACE, &snd_queue_remain);
    printf("Chn%d remain space:%u\n", ChIdx, snd_queue_remain);
}
int DevType = USBCANFD;    // 设备类型号 33-usbcanfd
int DevIdx = 0;                 // 设备索引号
int IsMerge = 0;                // 是否合并接收

THREAD_CTX rx_ctx[MAX_CHANNELS];        // 接收线程上下文
pthread_t rx_threads[MAX_CHANNELS]; // 接收线程

void MainWindow::on_connectButton_clicked()
{
    // 打开设备
        if (!VCI_OpenDevice(DevType, DevIdx, 0))
            QMessageBox::critical(this, "打开状态","打开失败");
        else
            QMessageBox::information(this,"打开状态","打开成功");
        return ;
}


void MainWindow::on_initializeButton_clicked()
{

    for (int i = 0; i < MAX_CHANNELS; i++){
            ZCAN_INIT init;         // 波特率结构体，数据根据zcanpro的波特率计算器得出
            init.clk = 60000000;    // clock: 60M(V1.01) 80M(V1.03即以上)
            init.mode = 0;          // 0-正常

            init.aset.tseg1 = 14;   // 仲裁域 500kbps
            init.aset.tseg2 = 3;
            init.aset.sjw = 2;
            init.aset.smp = 0;
            init.aset.brp = 5;

            init.dset.tseg1 = 10;   // 数据域 2000kbps
            init.dset.tseg2 = 2;
            init.dset.sjw = 2;
            init.dset.smp = 0;
            init.dset.brp = 1;

            if (!VCI_InitCAN(DevType, DevIdx, i, &init))    // 初始化通道
                    {
                        //printf("InitCAN(%d) fail\n", i);
                        QMessageBox::critical(this, "启动状态","CAN启动失败");
                        return ;
                    }
            else
                    //printf("InitCAN(%d) success\n", i);
                QMessageBox::information(this,"启动状态","CAN启动成功");

            U32 on = 1;
            if (!VCI_SetReference(DevType, DevIdx, i, CMD_CAN_TRES, &on)) // 终端电阻
            {
                printf("CMD_CAN_TRES fail\n");
            }

            if(i == 0){
                if (!VCI_SetReference(DevType, DevIdx, i, ZCAN_CMD_SET_CHNL_RECV_MERGE, &IsMerge)) // 合并接收
                {
                    printf("ZCAN_CMD_SET_CHNL_RECV_MERGE fail\n");
                }
            }

            if (!VCI_StartCAN(DevType, DevIdx, i))          // 启动通道
                   {

                       printf("StartCAN(%d) fail\n", i);
                       return ;
                   }
                   printf("StartCAN(%d) success\n", i);

            rx_ctx[i].dev_type = DevType;
            rx_ctx[i].dev_idx = DevIdx;
            rx_ctx[i].chn_idx = i;
            rx_ctx[i].total = 0;
            rx_ctx[i].stop = 0;
            rx_ctx[i].mainWindow = this; // 绑定当前MainWindow实例


            if(IsMerge && i == 0){
                pthread_create(&rx_threads[i], NULL, rx_merge_thread, &rx_ctx[i]);  // 合并接收线程
            }
            else{
                pthread_create(&rx_threads[i], NULL, rx_thread, &rx_ctx[i]);        // 创建接收线程
            }
        }
}


void MainWindow::on_closeButton_clicked()
{
    // 停止所有接收线程
    for (int i = 0; i < MAX_CHANNELS; i++) {
        rx_ctx[i].stop = 1; // 设置终止标志
        pthread_join(rx_threads[i], NULL); // 等待线程退出
    }

    // 关闭设备
        if (!VCI_CloseDevice(DevType, DevIdx))
            QMessageBox::critical(this, "关闭状态","CAN关闭失败");
        else
            QMessageBox::information(this, "关闭状态","CAN关闭成功");
        return ;
}

//发送运动数据
void MainWindow::sendMovementCommand(const U8 data[8])
{
    const UINT targetId = 0x611;// 目标CAN ID
    // 构造CAN帧
    ZCAN_20_MSG canMsg;
    memset(&canMsg, 0, sizeof(ZCAN_20_MSG));

    canMsg.hdr.inf.txm = ZCAN_TX_NORM;  // 正常发送模式
    canMsg.hdr.inf.fmt = 0;             // CAN 2.0帧
    canMsg.hdr.inf.sdf = 0;             // 数据帧
    canMsg.hdr.inf.sef = 0;             // 标准帧（11位ID）
    canMsg.hdr.id = targetId;           // 目标ID
    canMsg.hdr.chn = 0;                 // 通道0
    canMsg.hdr.len = 8;                 // 数据长度8字节
    memcpy(canMsg.dat, data, 8);        // 填充运动指令数据

    // 发送CAN帧
    UINT ret = VCI_Transmit(DevType, DevIdx, 0, &canMsg, 1);
    if (ret != 1) {
        QMessageBox::critical(this, "发送失败", "运动指令发送失败！请检查连接！" );
        stopMove();
    }
}

void MainWindow::sendForward()
{
    const U8 data[8] = {0x2A, 0x01, 0x00, 0x4C, 0x00, 0x00, 0x00, 0x00};//小车直行0.076m/s
    sendMovementCommand(data);
}

void MainWindow::sendBack()
{
    const U8 data[8] = {0x2A, 0x01, 0xFF, 0xF0, 0x00, 0x00, 0x00, 0x00};//小车后退
    sendMovementCommand(data);
}

void MainWindow::sendLeft()
{
    const U8 data[8] = {0x2A, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10};//小车左转0.016rad/s
    sendMovementCommand(data);
}

void MainWindow::sendRight()
{
    const U8 data[8] = {0x2A, 0x01, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xF0};//小车右转
    sendMovementCommand(data);
}

void MainWindow::on_forwardButton_clicked()
{
    if (!m_isforwardSending)
    {
        m_isforwardSending = true;
        ui->forwardButton->setText("停止直行");
        m_forwardTimer->start(100); // 100ms周期启动定时器
    }
    else
    {
        m_isforwardSending = false;
        ui->forwardButton->setText("直行");
        m_forwardTimer->stop();
    }
}

void MainWindow::on_backButton_clicked()
{
    if (!m_isbackSending)
    {
        m_isbackSending = true;
        ui->backButton->setText("停止后退");
        m_backTimer->start(100); // 100ms周期启动定时器
    }
    else
    {
        m_isbackSending = false;
        ui->backButton->setText("后退");
        m_backTimer->stop();
    }
}

void MainWindow::on_leftButton_clicked()
{
    if (!m_isleftSending)
    {
        m_isleftSending = true;
        ui->leftButton->setText("停止左转");
        m_leftTimer->start(100); // 100ms周期启动定时器
    }
    else
    {
        m_isleftSending = false;
        ui->leftButton->setText("左转");
        m_leftTimer->stop();
    }
}

void MainWindow::on_rightButton_clicked()
{
    if (!m_isrightSending)
    {
        m_isrightSending = true;
        ui->rightButton->setText("停止右转");
        m_rightTimer->start(100); // 100ms周期启动定时器
    }
    else
    {
        m_isrightSending = false;
        ui->rightButton->setText("右转");
        m_rightTimer->stop();
    }
}

void MainWindow::stopMove()
{
    m_forwardTimer->stop();
    m_leftTimer->stop();
    m_backTimer->stop();
    m_rightTimer->stop();
}

void MainWindow::onAutoSequenceTimeout()
{
    stopMove(); // 停止所有动作
    if (m_currentActionIndex >= m_autoActionSequence.size()) {
        m_isautoSending = false;
        return;
    }

    auto pair = m_autoActionSequence[m_currentActionIndex];
    // 启动对应动作的定时器
    if (pair.first == &MainWindow::sendForward) {
        m_forwardTimer->start(100);
    } else if (pair.first == &MainWindow::sendLeft) {
        m_leftTimer->start(100);
    } else if (pair.first == &MainWindow::sendBack) {
        m_backTimer->start(100);
    } else if (pair.first == &MainWindow::sendRight) {
        m_rightTimer->start(100);
    }

    // 设置单次定时器，持续时间结束后进入下一个动作
    m_autoSequenceTimer->singleShot(pair.second, this, [this]() {
        stopMove();
        m_currentActionIndex++;
        onAutoSequenceTimeout(); // 手动触发下一次超时
    });
}

void MainWindow::initAutoActionSequence() //小车自动巡检动作列表
{
    m_autoActionSequence =
    {
        //{&MainWindow::sendForward, 30000}, //直行5秒
        //{&MainWindow::sendRight, 5000},   //右转5秒
        //{&MainWindow::sendForward, 5000}, //直行5秒
    };
    // 可扩展：从文件/数据库读取序列
}


void MainWindow::on_auto_move_clicked()
{
    if (!m_isautoSending)
    {
        m_isautoSending = true;
        m_currentActionIndex = 0;
        onAutoSequenceTimeout(); // 直接启动第一个动作
    } else {
        m_isautoSending = false;
        stopMove();
        m_autoSequenceTimer->stop();
    }
}

//传感器
void MainWindow::readH2SerialData()
{

}

/*void MainWindow::H2write()
{

    QString hexString = "01 03 00 00 00 01 84 0A";
    // 将十六进制字符串转换为二进制字节数组
    QByteArray sendData = QByteArray::fromHex(hexString.toLatin1());

    // 发送二进制数据
    serial->write(sendData);
    serial->flush();
    //qDebug()<<"发："<<sendData;

    QByteArray receivedData = serial->readAll();
    //QString hexReceived = receivedData.toHex(' ');
    //qDebug()<<"收到原始数据："<<hexReceived;
    for(int i =0;i<receivedData.size() - 2;i++)
    {
        if(receivedData.at(i) == 0x01 &&
           receivedData.at(i+1) == 0x03 &&
           receivedData.at(i+2) == 0x02)
        {
            if(i+4<receivedData.size()){
                quint8 highByte = receivedData.at(i + 3);
                quint8 lowByte = receivedData.at(i+4);
                quint16 buf = (highByte << 8) | lowByte;
                 ui->H2->setText(QString("氢气的浓度：%1 ppm").arg(buf));
                 if(buf >8)
                 {
                     led.setLED(ui->led1, 1, 32);
                 }
                 else{
                     led.setLED(ui->led1, 2, 32);
                 }

            }
           else{
               qDebug()<<"数据长度不足";
            }
        break;
    }}
}*/

void MainWindow::H2write()
{
    QString hexString = "01 03 00 00 00 01 84 0A";
    QByteArray sendData = QByteArray::fromHex(hexString.toLatin1());
    serial->write(sendData);
    serial->flush();

    QByteArray receivedData = serial->readAll();
    for(int i = 0; i < receivedData.size() - 2; i++)
    {
        if(receivedData.at(i) == 0x01 &&
           receivedData.at(i+1) == 0x03 &&
           receivedData.at(i+2) == 0x02)
        {
            if(i+4 < receivedData.size()){
                quint8 highByte = receivedData.at(i + 3);
                quint8 lowByte = receivedData.at(i+4);
                quint16 buf = (highByte << 8) | lowByte;
                ui->H2->setText(QString("氢气的浓度：%1 ppm").arg(buf));

                if(buf > 10)
                {
                    led.setLED(ui->led1, 1, 24);
                }
                else{
                    led.setLED(ui->led1, 2, 24);
                }
            }
            break;
        }
    }
}


void MainWindow::on_H2display_clicked()
{
    if (!serial->isOpen())
    {
        if (!serial->open(QIODevice::ReadWrite))
        {
            QMessageBox::critical(this, "错误", "串口打开失败，请检查连接！");
            return;
        }
        ui->H2display->setText("氢气传感器已连接");
    }
    H2Timer->start(1000);
}

/*void MainWindow::Flamewrite()
{
    QString hexString = "01 03 02 68 00 02 44 6F";

    // 将十六进制字符串转换为二进制字节数组
    QByteArray sendData1 = QByteArray::fromHex(hexString.toLatin1());

    // 发送二进制数据
    serial1->write(sendData1);
    serial1->flush();
    //qDebug()<<"发："<<sendData1;

    QByteArray data = serial1->readAll();
    //qDebug()<<"收到原始数据："<<data.toHex(' ');

    // 检查数据格式是否正确（根据实际收到的9字节数据调整判断条件）
    if(data.size() >= 9 && data[0] == 0x01 && data[1] == 0x03 && data[2] == 0x04)
    {
        // 按小端顺序提取字节：00（data[3]）、00（data[4]）、80（data[6]）、40（data[5]）
        QByteArray floatBytes;
        floatBytes.append(data[3]);  // 第0字节（低位）
        floatBytes.append(data[4]);  // 第1字节
        floatBytes.append(data[6]);  // 第2字节
        floatBytes.append(data[5]);  // 第3字节（高位）

        float value;
        memcpy(&value, floatBytes.data(), 4);  // 直接转换
        int intValue = qRound(value);

        if(intValue > 14){
            intValue = 20;
        }
        if(intValue >14)//设置警报状态
        {
            led.setLED(ui->led2, 1, 32);
        }
        else{
            led.setLED(ui->led2, 2, 32);
        }
        ui->Flame->setText(QString("火焰大小：%1").arg(intValue));
    }
}*/

void MainWindow::Flamewrite()
{
    QString hexString = "01 03 02 68 00 02 44 6F";
    QByteArray sendData1 = QByteArray::fromHex(hexString.toLatin1());
    serial1->write(sendData1);
    serial1->flush();

    QByteArray data = serial1->readAll();
    if(data.size() >= 9 && data[0] == 0x01 && data[1] == 0x03 && data[2] == 0x04)
    {
        QByteArray floatBytes;
        floatBytes.append(data[3]);
        floatBytes.append(data[4]);
        floatBytes.append(data[6]);
        floatBytes.append(data[5]);

        float value;
        memcpy(&value, floatBytes.data(), 4);
        int intValue = qRound(value);

        if(intValue > 14) {
            intValue = 20;
        }

        if(intValue > 14) {
            led.setLED(ui->led2, 1, 24);
        } else {
            led.setLED(ui->led2, 2, 24);
        }
        ui->Flame->setText(QString("火焰大小：%1").arg(intValue));
    }
}




void MainWindow::readFlameSerialData()
{
    /*QByteArray receivedData = serial1->readAll();
    if (receivedData.isEmpty()) return;
    // 在界面日志显示接收的原始数据（十六进制形式）
    QString hexReceived = receivedData.toHex(' ');
    qDebug()<<"原始数据："<<hexReceived;

    quint8 firstByte = static_cast<quint8>(receivedData[0]);
    quint8 secondByte = static_cast<quint8>(receivedData[1]);
    if (receivedData.size() >= 5)
    {
        if (firstByte == 0x01 && secondByte == 0x03)
        {
        quint8 highByte = static_cast<quint8>(receivedData[3]);
        quint8 lowByte = static_cast<quint8>(receivedData[4]);
        quint16 concentration = (highByte << 8) | lowByte;
        FlameTimer->start(100);
        ui->Flame->setText(QString("火焰大小：").arg(concentration));
        }
    }
    else
    {
        return;
    }*/
}

void MainWindow::on_Flamedisplay_clicked()
{
    if (!serial1->isOpen())
    {
        if (!serial1->open(QIODevice::ReadWrite))
        {
            QMessageBox::critical(this, "错误", "串口打开失败，请检查连接！");
            return;
        }
        ui->Flamedisplay->setText("火焰传感器已连接");
    }
    FlameTimer->start(1000);
}

void MainWindow::on_login_clicked()//机械臂登录
{
    int ret = aubo_robot_namespace::InterfaceCallSuccCode;
    aubo_robot_namespace::RobotDhPara robotDhPara;
    aubo_robot_namespace::RobotType robotType = static_cast<aubo_robot_namespace::RobotType>(8);
    /** 接口调用 : 登录 ***/
    ret = robotService.robotServiceLogin(SERVER_HOST, SERVER_PORT, "aubo", "123456",robotType,robotDhPara);
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        m_login = true;
        QMessageBox::information(this, "登录状态","机械臂登录成功");
        ui->armstatus->setText("机械臂状态：已登录");
        ui->login->setText("退出登录");
    }
    else
    {
        ret = robotService.robotServiceLogout();
        m_login = false;
        ui->login->setText("登录");
        ui->armstatus->setText("机械臂状态：未登录");
        QMessageBox::critical(this, "登录状态","机械臂退出");
    }
}

// ========== 进度条更新函数 ==========
void MainWindow::updatePowerOnProgress()
{
    // 进度逐步增加（模拟上电过程，可根据实际回调调整）
    currentProgress += 5;
    if (currentProgress >= 99) {  // 留1%等上电完成后拉满
        currentProgress = 99;
        progressTimer->stop();  // 暂停增长，等上电结果
    }
    powerOnProgress->setValue(currentProgress);
}

void MainWindow::on_poweron_clicked()
{
    // 1. 重置进度状态
    currentProgress = 0;
    powerOnProgress->setValue(0);
    powerOnProgress->show();  // 显示进度条
    progressTimer->start();   // 启动进度更新

    // 2. 机械臂上电参数
    int ret;
    aubo_robot_namespace::ROBOT_SERVICE_STATE result;
    aubo_robot_namespace::ToolDynamicsParam toolDynamicsParam;
    toolDynamicsParam.payload = 0;
    toolDynamicsParam.positionX = 0;
    toolDynamicsParam.positionY = 0;
    toolDynamicsParam.positionZ = 0;
    toolDynamicsParam.toolInertia.xx = 0;
    toolDynamicsParam.toolInertia.xy = 0;
    toolDynamicsParam.toolInertia.xz = 0;
    toolDynamicsParam.toolInertia.yy = 0;
    toolDynamicsParam.toolInertia.yz= 0;
    toolDynamicsParam.toolInertia.zz = 0;

    // 3. 子线程执行上电操作（关键：避免UI卡顿）
    QThread *powerOnThread = QThread::create([&]()
    {
        ret = robotService.rootServiceRobotStartup(toolDynamicsParam,6,true,true,1000,result);
    });

    // 4. 上电完成后的回调处理
    connect(powerOnThread, &QThread::finished, this, [=]()
    {
        // 停止定时器，进度条拉满
        progressTimer->stop();
        currentProgress = 100;
        powerOnProgress->setValue(100);

        // 延迟关闭进度条（让用户看到100%）
        QTimer::singleShot(300, this, [=]() {
            powerOnProgress->hide();

            // 显示上电结果
            if (ret == aubo_robot_namespace::InterfaceCallSuccCode)
            {
                QMessageBox::information(this, "上电状态", "上电成功！机械臂已启动");
                ui->powerstatus->setText("机械臂电源：已上电");
            }
            else
            {
                QMessageBox::information(this, "上电状态", "上电成功！机械臂已启动");
                ui->powerstatus->setText("机械臂电源：未上电");
            }
        });

        // 释放线程资源
        powerOnThread->deleteLater();
    });

    // 启动上电线程
    powerOnThread->start();
}

void MainWindow::powerstarts()
{
    aubo_robot_namespace::RobotDiagnosis robotdiagnosis;
    int ret = robotService.robotServiceGetRobotDiagnosisInfo(robotdiagnosis);
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        // 更新电源状态变量
        m_power = robotdiagnosis.armPowerStatus;
        if(m_power == 1)
        {
            ui->powerstatus->setText("机械臂电源：已上电");
        }
        else
        {
           ui->powerstatus->setText("机械臂电源：未上电");
        }
    }
}

void MainWindow::on_powerout_clicked()
{
    QMessageBox::StandardButton reply;
    reply = QMessageBox::question(this, "断电确认", "确定要给机械臂断电吗？",
                                 QMessageBox::Yes | QMessageBox::No);

    if (reply == QMessageBox::Yes) {
        // 用户确认后才执行断电操作
        int ret = robotService.robotServiceRobotShutdown(true);
        if( ret == aubo_robot_namespace::InterfaceCallSuccCode )
        {
            QMessageBox::information(this, "断电状态", "断电成功");
            ui->powerstatus->setText("机械臂电源：未上电");
            m_power = false;
        }
        else
        {
            QMessageBox::critical(this, "断电状态", "断电失败");
        }
    }
}

//位姿信息
void RealTimeWaypointCallback(const aubo_robot_namespace::wayPoint_S *wayPointPtr, void *arg)
{
    (void)arg;
    aubo_robot_namespace::wayPoint_S wayPoint = *wayPointPtr;
    aubo_robot_namespace::Rpy tempRpy;
    robotService.quaternionToRPY(wayPoint.orientation,tempRpy);

    if (!arg) return;
        MainWindow *mainWindow = static_cast<MainWindow*>(arg);
    if (mainWindow)
    {
        mainWindow->setCurrentWayPoint(*wayPointPtr); // 调用 Setter
        mainWindow->setCurrentRpy(tempRpy);
        mainWindow->setJointAngles(wayPoint.jointpos);
        QMetaObject::invokeMethod(mainWindow, "updatePositionDisplay", Qt::QueuedConnection);
    }

}

void MainWindow::setCurrentWayPoint(const aubo_robot_namespace::wayPoint_S &wayPoint)
{
    currentWayPoint = wayPoint; // 间接修改私有成员
}

void MainWindow::setCurrentRpy(const aubo_robot_namespace::Rpy &rpy)
{
    currentRpy = rpy; // 显式赋值 currentRpy
}

void MainWindow::setJointAngles(const double angles[6])
{
    for (int i = 0; i < 6; i++) {
        currentJointAngles[i] = angles[i];
    }
}

void MainWindow::updatePositionDisplay()
{
    ui->posx->setText(QString::number(currentWayPoint.cartPos.position.x, 'f', 3));
    ui->posy->setText(QString::number(currentWayPoint.cartPos.position.y, 'f', 3));
    ui->posz->setText(QString::number(currentWayPoint.cartPos.position.z, 'f', 3));
    ui->posrx->setText(QString::number(currentRpy.rx*180/M_PI, 'f', 3));
    ui->posry->setText(QString::number(currentRpy.ry*180/M_PI, 'f', 3));
    ui->posrz->setText(QString::number(currentRpy.rz*180/M_PI, 'f', 3));
    ui->j1->setText(QString::number(currentJointAngles[0]* 180.0 / M_PI, 'f', 3));
    ui->j2->setText(QString::number(currentJointAngles[1]* 180.0 / M_PI, 'f', 3));
    ui->j3->setText(QString::number(currentJointAngles[2]* 180.0 / M_PI, 'f', 3));
    ui->j4->setText(QString::number(currentJointAngles[3]* 180.0 / M_PI, 'f', 3));
    ui->j5->setText(QString::number(currentJointAngles[4]* 180.0 / M_PI, 'f', 3));
    ui->j6->setText(QString::number(currentJointAngles[5]* 180.0 / M_PI, 'f', 3));
}

void MainWindow::on_getpos_clicked()
{
    robotService.robotServiceRegisterRealTimeRoadPointCallback(RealTimeWaypointCallback, this);
    m_posTimer->start(100);
}

void MainWindow::setjointacc()//速度设置
{
    // 设置关节运动最大加速度
    aubo_robot_namespace::JointVelcAccParam m_set_jointacc;
    m_set_jointacc.jointPara[0] = 30*M_PI/180;
    m_set_jointacc.jointPara[1] = 30*M_PI/180;
    m_set_jointacc.jointPara[2] = 30*M_PI/180;
    m_set_jointacc.jointPara[3] = 30*M_PI/180;
    m_set_jointacc.jointPara[4] = 30*M_PI/180;
    m_set_jointacc.jointPara[5] = 30*M_PI/180;
    robotService.robotServiceSetGlobalMoveJointMaxAcc(m_set_jointacc);
    // 设置关节运动最大速度
    aubo_robot_namespace::JointVelcAccParam m_set_jointvelc;
    m_set_jointvelc.jointPara[0] = 10*M_PI/180;
    m_set_jointvelc.jointPara[1] = 10*M_PI/180;
    m_set_jointvelc.jointPara[2] = 10*M_PI/180;
    m_set_jointvelc.jointPara[3] = 10*M_PI/180;
    m_set_jointvelc.jointPara[4] = 10*M_PI/180;
    m_set_jointvelc.jointPara[5] = 10*M_PI/180;
    robotService.robotServiceSetGlobalMoveJointMaxVelc(m_set_jointvelc);
}

//关节运动
void MainWindow::on_jointmove_clicked()
{
    robotService.robotServiceInitGlobalMoveProfile();//运动属性初始化
    aubo_robot_namespace::wayPoint_S wp0;
    wp0.jointpos[0] = -80.394290*M_PI/180;
    wp0.jointpos[1] = 19.984145*M_PI/180;
    wp0.jointpos[2] = 102.815295*M_PI/180;
    wp0.jointpos[3] = -86.587592*M_PI/180;
    wp0.jointpos[4] = -78.324846*M_PI/180;
    wp0.jointpos[5] = 180.242560*M_PI/180;
    //robotService.robotServiceJointMove(wp0,true);
}


void MainWindow::on_Xplus_clicked()
{
    double startpoint[6] = {};
    for (int i = 0; i < 6; ++i)
    {
        startpoint[i] = currentJointAngles[i];
    }
    double xoffset = ui->offset->value();
    // 目标位置
    aubo_robot_namespace::Pos targetpos;
    targetpos.x = currentWayPoint.cartPos.position.x+xoffset;
    targetpos.y = currentWayPoint.cartPos.position.y;
    targetpos.z = currentWayPoint.cartPos.position.z;
    // 目标姿态
    aubo_robot_namespace::Ori targetori;
    aubo_robot_namespace::Rpy targetrpy;
    targetrpy.rx = currentRpy.rx;
    targetrpy.ry = currentRpy.ry;
    targetrpy.rz = currentRpy.rz;
    robotService.RPYToQuaternion(targetrpy,targetori);
    aubo_robot_namespace::wayPoint_S waypointIK;
    robotService.robotServiceRobotIk(startpoint,targetpos,targetori,waypointIK);
    robotService.robotServiceJointMove(waypointIK,true);
}


void MainWindow::on_Xminus_clicked()
{
    double startpoint[6] = {};
    for (int i = 0; i < 6; ++i)
    {
        startpoint[i] = currentJointAngles[i];
    }
    double xoffset = ui->offset->value();
    // 目标位置
    aubo_robot_namespace::Pos targetpos;
    targetpos.x = currentWayPoint.cartPos.position.x-xoffset;
    targetpos.y = currentWayPoint.cartPos.position.y;
    targetpos.z = currentWayPoint.cartPos.position.z;
    // 目标姿态
    aubo_robot_namespace::Ori targetori;
    aubo_robot_namespace::Rpy targetrpy;
    targetrpy.rx = currentRpy.rx;
    targetrpy.ry = currentRpy.ry;
    targetrpy.rz = currentRpy.rz;
    robotService.RPYToQuaternion(targetrpy,targetori);
    aubo_robot_namespace::wayPoint_S waypointIK;
    robotService.robotServiceRobotIk(startpoint,targetpos,targetori,waypointIK);
    robotService.robotServiceJointMove(waypointIK,true);
}


void MainWindow::on_Yplus_clicked()
{
    double startpoint[6] = {};
    for (int i = 0; i < 6; ++i)
    {
        startpoint[i] = currentJointAngles[i];
    }
    double yoffset = ui->offset->value();
    // 目标位置
    aubo_robot_namespace::Pos targetpos;
    targetpos.x = currentWayPoint.cartPos.position.x;
    targetpos.y = currentWayPoint.cartPos.position.y+yoffset;
    targetpos.z = currentWayPoint.cartPos.position.z;
    // 目标姿态
    aubo_robot_namespace::Ori targetori;
    aubo_robot_namespace::Rpy targetrpy;
    targetrpy.rx = currentRpy.rx;
    targetrpy.ry = currentRpy.ry;
    targetrpy.rz = currentRpy.rz;
    robotService.RPYToQuaternion(targetrpy,targetori);
    aubo_robot_namespace::wayPoint_S waypointIK;
    robotService.robotServiceRobotIk(startpoint,targetpos,targetori,waypointIK);
    robotService.robotServiceJointMove(waypointIK,true);
}


void MainWindow::on_Yminus_clicked()
{
    double startpoint[6] = {};
    for (int i = 0; i < 6; ++i)
    {
        startpoint[i] = currentJointAngles[i];
    }
    double yoffset = ui->offset->value();
    // 目标位置
    aubo_robot_namespace::Pos targetpos;
    targetpos.x = currentWayPoint.cartPos.position.x;
    targetpos.y = currentWayPoint.cartPos.position.y-yoffset;
    targetpos.z = currentWayPoint.cartPos.position.z;
    // 目标姿态
    aubo_robot_namespace::Ori targetori;
    aubo_robot_namespace::Rpy targetrpy;
    targetrpy.rx = currentRpy.rx;
    targetrpy.ry = currentRpy.ry;
    targetrpy.rz = currentRpy.rz;
    robotService.RPYToQuaternion(targetrpy,targetori);
    aubo_robot_namespace::wayPoint_S waypointIK;
    robotService.robotServiceRobotIk(startpoint,targetpos,targetori,waypointIK);
    robotService.robotServiceJointMove(waypointIK,true);
}


void MainWindow::on_Zplus_clicked()
{
    double startpoint[6] = {};
    for (int i = 0; i < 6; ++i)
    {
        startpoint[i] = currentJointAngles[i];
    }
    double zoffset = ui->offset->value();
    // 目标位置
    aubo_robot_namespace::Pos targetpos;
    targetpos.x = currentWayPoint.cartPos.position.x;
    targetpos.y = currentWayPoint.cartPos.position.y;
    targetpos.z = currentWayPoint.cartPos.position.z+zoffset;
    // 目标姿态
    aubo_robot_namespace::Ori targetori;
    aubo_robot_namespace::Rpy targetrpy;
    targetrpy.rx = currentRpy.rx;
    targetrpy.ry = currentRpy.ry;
    targetrpy.rz = currentRpy.rz;
    robotService.RPYToQuaternion(targetrpy,targetori);
    aubo_robot_namespace::wayPoint_S waypointIK;
    robotService.robotServiceRobotIk(startpoint,targetpos,targetori,waypointIK);
    robotService.robotServiceJointMove(waypointIK,true);
}


void MainWindow::on_Zminus_clicked()
{
    double startpoint[6] = {};
    for (int i = 0; i < 6; ++i)
    {
        startpoint[i] = currentJointAngles[i];
    }
    double zoffset = ui->offset->value();
    // 目标位置
    aubo_robot_namespace::Pos targetpos;
    targetpos.x = currentWayPoint.cartPos.position.x;
    targetpos.y = currentWayPoint.cartPos.position.y;
    targetpos.z = currentWayPoint.cartPos.position.z-zoffset;
    // 目标姿态
    aubo_robot_namespace::Ori targetori;
    aubo_robot_namespace::Rpy targetrpy;
    targetrpy.rx = currentRpy.rx;
    targetrpy.ry = currentRpy.ry;
    targetrpy.rz = currentRpy.rz;
    robotService.RPYToQuaternion(targetrpy,targetori);
    aubo_robot_namespace::wayPoint_S waypointIK;
    robotService.robotServiceRobotIk(startpoint,targetpos,targetori,waypointIK);
    robotService.robotServiceJointMove(waypointIK,true);
}

void MainWindow::on_Rxplus_clicked()
{
    double startpoint[6] = {};
    for (int i = 0; i < 6; ++i)
    {
        startpoint[i] = currentJointAngles[i];
    }
    double Rxoffset = ui->offset1->value()*M_PI/180;
    // 目标位置
    aubo_robot_namespace::Pos targetpos;
    targetpos.x = currentWayPoint.cartPos.position.x;
    targetpos.y = currentWayPoint.cartPos.position.y;
    targetpos.z = currentWayPoint.cartPos.position.z;
    // 目标姿态
    aubo_robot_namespace::Ori targetori;
    aubo_robot_namespace::Rpy targetrpy;
    targetrpy.rx = currentRpy.rx+Rxoffset;
    targetrpy.ry = currentRpy.ry;
    targetrpy.rz = currentRpy.rz;
    robotService.RPYToQuaternion(targetrpy,targetori);
    aubo_robot_namespace::wayPoint_S waypointIK;
    robotService.robotServiceRobotIk(startpoint,targetpos,targetori,waypointIK);
    robotService.robotServiceJointMove(waypointIK,true);
}


void MainWindow::on_Rxminus_clicked()
{
    double startpoint[6] = {};
    for (int i = 0; i < 6; ++i)
    {
        startpoint[i] = currentJointAngles[i];
    }
    double Rxoffset = ui->offset1->value()*M_PI/180;
    // 目标位置
    aubo_robot_namespace::Pos targetpos;
    targetpos.x = currentWayPoint.cartPos.position.x;
    targetpos.y = currentWayPoint.cartPos.position.y;
    targetpos.z = currentWayPoint.cartPos.position.z;
    // 目标姿态
    aubo_robot_namespace::Ori targetori;
    aubo_robot_namespace::Rpy targetrpy;
    targetrpy.rx = currentRpy.rx-Rxoffset;
    targetrpy.ry = currentRpy.ry;
    targetrpy.rz = currentRpy.rz;
    robotService.RPYToQuaternion(targetrpy,targetori);
    aubo_robot_namespace::wayPoint_S waypointIK;
    robotService.robotServiceRobotIk(startpoint,targetpos,targetori,waypointIK);
    robotService.robotServiceJointMove(waypointIK,true);
}


void MainWindow::on_Ryplus_clicked()
{
    double startpoint[6] = {};
    for (int i = 0; i < 6; ++i)
    {
        startpoint[i] = currentJointAngles[i];
    }
    double Ryoffset = ui->offset1->value()*M_PI/180;
    // 目标位置
    aubo_robot_namespace::Pos targetpos;
    targetpos.x = currentWayPoint.cartPos.position.x;
    targetpos.y = currentWayPoint.cartPos.position.y;
    targetpos.z = currentWayPoint.cartPos.position.z;
    // 目标姿态
    aubo_robot_namespace::Ori targetori;
    aubo_robot_namespace::Rpy targetrpy;
    targetrpy.rx = currentRpy.rx;
    targetrpy.ry = currentRpy.ry+Ryoffset;
    targetrpy.rz = currentRpy.rz;
    robotService.RPYToQuaternion(targetrpy,targetori);
    aubo_robot_namespace::wayPoint_S waypointIK;
    robotService.robotServiceRobotIk(startpoint,targetpos,targetori,waypointIK);
    robotService.robotServiceJointMove(waypointIK,true);
}


void MainWindow::on_Ryminus_clicked()
{
    double startpoint[6] = {};
    for (int i = 0; i < 6; ++i)
{
        startpoint[i] = currentJointAngles[i];
    }
    double Ryoffset = ui->offset1->value()*M_PI/180;
    // 目标位置
    aubo_robot_namespace::Pos targetpos;
    targetpos.x = currentWayPoint.cartPos.position.x;
    targetpos.y = currentWayPoint.cartPos.position.y;
    targetpos.z = currentWayPoint.cartPos.position.z;
// 目标姿态
    aubo_robot_namespace::Ori targetori;
    aubo_robot_namespace::Rpy targetrpy;
    targetrpy.rx = currentRpy.rx;
    targetrpy.ry = currentRpy.ry-Ryoffset;
    targetrpy.rz = currentRpy.rz;
    robotService.RPYToQuaternion(targetrpy,targetori);
    aubo_robot_namespace::wayPoint_S waypointIK;
    robotService.robotServiceRobotIk(startpoint,targetpos,targetori,waypointIK);
    robotService.robotServiceJointMove(waypointIK,true);
}


void MainWindow::on_Rzplus_clicked()
{
    double startpoint[6] = {};
    for (int i = 0; i < 6; ++i)
    {
        startpoint[i] = currentJointAngles[i];
    }
    double Rzoffset = ui->offset1->value()*M_PI/180;
    // 目标位置
    aubo_robot_namespace::Pos targetpos;
    targetpos.x = currentWayPoint.cartPos.position.x;
    targetpos.y = currentWayPoint.cartPos.position.y;
    targetpos.z = currentWayPoint.cartPos.position.z;
    // 目标姿态
    aubo_robot_namespace::Ori targetori;
    aubo_robot_namespace::Rpy targetrpy;
    targetrpy.rx = currentRpy.rx;
    targetrpy.ry = currentRpy.ry;
    targetrpy.rz = currentRpy.rz+Rzoffset;
    robotService.RPYToQuaternion(targetrpy,targetori);
    aubo_robot_namespace::wayPoint_S waypointIK;
    robotService.robotServiceRobotIk(startpoint,targetpos,targetori,waypointIK);
    robotService.robotServiceJointMove(waypointIK,true);
}


void MainWindow::on_Rzminus_clicked()
{
    double startpoint[6] = {};
    for (int i = 0; i < 6; ++i)
    {
        startpoint[i] = currentJointAngles[i];
    }
    double Rzoffset = ui->offset1->value()*M_PI/180;
    // 目标位置
    aubo_robot_namespace::Pos targetpos;
    targetpos.x = currentWayPoint.cartPos.position.x;
    targetpos.y = currentWayPoint.cartPos.position.y;
    targetpos.z = currentWayPoint.cartPos.position.z;
    // 目标姿态
    aubo_robot_namespace::Ori targetori;
    aubo_robot_namespace::Rpy targetrpy;
    targetrpy.rx = currentRpy.rx;
    targetrpy.ry = currentRpy.ry;
    targetrpy.rz = currentRpy.rz-Rzoffset;
    robotService.RPYToQuaternion(targetrpy,targetori);
    aubo_robot_namespace::wayPoint_S waypointIK;
    robotService.robotServiceRobotIk(startpoint,targetpos,targetori,waypointIK);
    robotService.robotServiceJointMove(waypointIK,true);

}

void MainWindow::movetopoint(double x, double y, double z, double rx, double ry, double rz)
{
    double startpoint[6] = {};
    for (int i = 0; i < 6; ++i)
    {
        startpoint[i] = currentJointAngles[i];
        std::cout<<"当前joint"<<i+1<<": "<<currentJointAngles[i]*180/M_PI<<std::endl;

    }

    // 目标位置
    aubo_robot_namespace::Pos targetpos;
    targetpos.x = x;
    targetpos.y = y;
    targetpos.z = z;
    // 目标姿态
    aubo_robot_namespace::Ori targetori;
    aubo_robot_namespace::Rpy targetrpy;
    targetrpy.rx = rx*M_PI/180;
    targetrpy.ry = ry*M_PI/180;
    targetrpy.rz = rz*M_PI/180;
    robotService.RPYToQuaternion(targetrpy,targetori);
    aubo_robot_namespace::wayPoint_S waypointIK;
    int ret = robotService.robotServiceRobotIk(startpoint,targetpos,targetori,waypointIK);
    robotService.robotServiceJointMove(waypointIK,true);

    std::cout<<"目标x:"<<targetpos.x<<" ";
    std::cout<<"y:"<<targetpos.y<<" ";
    std::cout<<"z:"<<targetpos.z<<std::endl;
    std::cout<<"rx:"<<targetrpy.rx * 180.0/M_PI<<" ";
    std::cout<<"ry:"<<targetrpy.ry * 180.0/M_PI<<" ";
    std::cout<<"rz:"<<targetrpy.rz * 180.0/M_PI<<std::endl;


    if (ret != aubo_robot_namespace::InterfaceCallSuccCode) {
        std::cout << "逆解计算失败，错误码：" << ret << std::endl;
    }

    std::cout<<"x:"<<waypointIK.cartPos.position.x<<" ";
    std::cout<<"y:"<<waypointIK.cartPos.position.y<<" ";
    std::cout<<"z:"<<waypointIK.cartPos.position.z<<std::endl;

    std::cout<<"目标关节信息: "<<std::endl;
    for(int i=0;i<aubo_robot_namespace::ARM_DOF;i++)
    {
    std::cout<<"joint"<<i+1<<": "<<waypointIK.jointpos[i]*180.0/M_PI<<std::endl;
    }
}

void MainWindow::on_pathplan_clicked()
{
    movetopoint(-0.190958, -0.013882, 0.872568, 113.081940, -0.029811, 0.026932);

}

void MainWindow::aubo_speed(int value)
{
    // 将滑块值（0-100）映射到关节速度范围（例如：0.1π到1π 弧度/秒）
    double speedFactor = value / 100.0;
    double minSpeed = 1 * M_PI / 180;  // 最小速度
    double maxSpeed = 10 * M_PI / 180;   // 最大速度（可根据实际需求调整）
    double currentSpeed = minSpeed + (maxSpeed - minSpeed) * speedFactor;

    // 更新关节速度参数
    aubo_robot_namespace::JointVelcAccParam jointVelc;
    for (int i = 0; i < 6; ++i)
    {
        jointVelc.jointPara[i] = currentSpeed;
    }

    // 应用速度设置
    robotService.robotServiceSetGlobalMoveJointMaxVelc(jointVelc);

    // 显示当前速度（可选，需在UI添加显示标签，例如ui->jointSpeedLabel）
    ui->jointSpeedLabel->setText(QString("关节速度：%1°/s").arg(currentSpeed * 180 / M_PI, 0, 'f', 2));

}

void MainWindow::on_J1_clicked()
{
    double offset_j = ui->offset_joint->value();
    double wp1[6] = {};
    wp1[0] = currentJointAngles[0]+ offset_j*M_PI/180;
    wp1[1] = currentJointAngles[1];
    wp1[2] = currentJointAngles[2];
    wp1[3] = currentJointAngles[3];
    wp1[4] = currentJointAngles[4];
    wp1[5] = currentJointAngles[5];
    robotService.robotServiceJointMove(wp1,true);
}


void MainWindow::on_J1_2_clicked()
{
    double offset_j = ui->offset_joint->value();
    double wp1[6] = {};
    wp1[0] = currentJointAngles[0] - offset_j*M_PI/180;
    wp1[1] = currentJointAngles[1];
    wp1[2] = currentJointAngles[2];
    wp1[3] = currentJointAngles[3];
    wp1[4] = currentJointAngles[4];
    wp1[5] = currentJointAngles[5];
    robotService.robotServiceJointMove(wp1,true);
}

void MainWindow::on_J2_clicked()
{
    double offset_j = ui->offset_joint->value();
    double wp1[6] = {};
    wp1[0] = currentJointAngles[0];
    wp1[1] = currentJointAngles[1]+ offset_j*M_PI/180;
    wp1[2] = currentJointAngles[2];
    wp1[3] = currentJointAngles[3];
    wp1[4] = currentJointAngles[4];
    wp1[5] = currentJointAngles[5];
    robotService.robotServiceJointMove(wp1,true);
}

void MainWindow::on_J2_2_clicked()
{
    double offset_j = ui->offset_joint->value();
    double wp1[6] = {};
    wp1[0] = currentJointAngles[0];
    wp1[1] = currentJointAngles[1]- offset_j*M_PI/180;
    wp1[2] = currentJointAngles[2];
    wp1[3] = currentJointAngles[3];
    wp1[4] = currentJointAngles[4];
    wp1[5] = currentJointAngles[5];
    robotService.robotServiceJointMove(wp1,true);
}


void MainWindow::on_J3_clicked()
{
    double offset_j = ui->offset_joint->value();
    double wp1[6] = {};
    wp1[0] = currentJointAngles[0];
    wp1[1] = currentJointAngles[1];
    wp1[2] = currentJointAngles[2]+ offset_j*M_PI/180;
    wp1[3] = currentJointAngles[3];
    wp1[4] = currentJointAngles[4];
    wp1[5] = currentJointAngles[5];
    robotService.robotServiceJointMove(wp1,true);
}


void MainWindow::on_J3_2_clicked()
{
    double offset_j = ui->offset_joint->value();
    double wp1[6] = {};
    wp1[0] = currentJointAngles[0];
    wp1[1] = currentJointAngles[1];
    wp1[2] = currentJointAngles[2]- offset_j*M_PI/180;
    wp1[3] = currentJointAngles[3];
    wp1[4] = currentJointAngles[4];
    wp1[5] = currentJointAngles[5];
    robotService.robotServiceJointMove(wp1,true);
}


void MainWindow::on_J4_clicked()
{
    double offset_j = ui->offset_joint->value();
    double wp1[6] = {};
    wp1[0] = currentJointAngles[0];
    wp1[1] = currentJointAngles[1];
    wp1[2] = currentJointAngles[2];
    wp1[3] = currentJointAngles[3]+ offset_j*M_PI/180;
    wp1[4] = currentJointAngles[4];
    wp1[5] = currentJointAngles[5];
    robotService.robotServiceJointMove(wp1,true);
}


void MainWindow::on_J4_2_clicked()
{
    double offset_j = ui->offset_joint->value();
    double wp1[6] = {};
    wp1[0] = currentJointAngles[0];
    wp1[1] = currentJointAngles[1];
    wp1[2] = currentJointAngles[2];
    wp1[3] = currentJointAngles[3]- offset_j*M_PI/180;
    wp1[4] = currentJointAngles[4];
    wp1[5] = currentJointAngles[5];
    robotService.robotServiceJointMove(wp1,true);
}


void MainWindow::on_J5_clicked()
{
    double offset_j = ui->offset_joint->value();
    double wp1[6] = {};
    wp1[0] = currentJointAngles[0];
    wp1[1] = currentJointAngles[1];
    wp1[2] = currentJointAngles[2];
    wp1[3] = currentJointAngles[3];
    wp1[4] = currentJointAngles[4]+ offset_j*M_PI/180;
    wp1[5] = currentJointAngles[5];
    robotService.robotServiceJointMove(wp1,true);
}


void MainWindow::on_J5_2_clicked()
{
    double offset_j = ui->offset_joint->value();
    double wp1[6] = {};
    wp1[0] = currentJointAngles[0];
    wp1[1] = currentJointAngles[1];
    wp1[2] = currentJointAngles[2];
    wp1[3] = currentJointAngles[3];
    wp1[4] = currentJointAngles[4]-offset_j*M_PI/180;
    wp1[5] = currentJointAngles[5];
    robotService.robotServiceJointMove(wp1,true);
}


void MainWindow::on_J6_clicked()
{
    double offset_j = ui->offset_joint->value();
    double wp1[6] = {};
    wp1[0] = currentJointAngles[0];
    wp1[1] = currentJointAngles[1];
    wp1[2] = currentJointAngles[2];
    wp1[3] = currentJointAngles[3];
    wp1[4] = currentJointAngles[4];
    wp1[5] = currentJointAngles[5]+ offset_j*M_PI/180;
    robotService.robotServiceJointMove(wp1,true);
}


void MainWindow::on_J6_2_clicked()
{
    double offset_j = ui->offset_joint->value();
    double wp1[6] = {};
    wp1[0] = currentJointAngles[0];
    wp1[1] = currentJointAngles[1];
    wp1[2] = currentJointAngles[2];
    wp1[3] = currentJointAngles[3];
    wp1[4] = currentJointAngles[4];
    wp1[5] = currentJointAngles[5]-offset_j*M_PI/180;
    robotService.robotServiceJointMove(wp1,true);
}

void MainWindow::on_Initial_pos_clicked()
{
    double wp1[6] = {};
    wp1[0] = -90*M_PI/180;
    wp1[1] = 45*M_PI/180;
    wp1[2] = 80*M_PI/180;
    wp1[3] = 35*M_PI/180;
    wp1[4] = 90*M_PI/180;
    wp1[5] = 0.0;
    robotService.robotServiceJointMove(wp1,true);
}

void MainWindow::addRosProcess(const QString& name, const QString& command, const QString& workspace)
{
    RosProcessInfo info;
    info.name = name;
    info.command = command;
    info.workspace = workspace.isEmpty() ? qEnvironmentVariable("ROS_WORKSPACE", "") : workspace;
    info.process = new QProcess(this);
    info.startButton = nullptr;
    info.stopButton = nullptr;

    // 连接信号槽，使用lambda表达式传递进程名称
    connect(info.process, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
            this, [=](int exitCode, QProcess::ExitStatus exitStatus) {
                onRosProcessFinished(name, exitCode, exitStatus);
            });
    connect(info.process, &QProcess::errorOccurred,
            this, [=](QProcess::ProcessError error) {
                onRosProcessError(name, error);
            });

    // 连接标准输出和错误输出
    connect(info.process, &QProcess::readyReadStandardOutput,
            this, [=]() { onRosProcessOutput(name); });
    connect(info.process, &QProcess::readyReadStandardError,
            this, [=]() { onRosProcessErrorOutput(name); });

    m_rosProcesses[name] = info;
}

void MainWindow::startRosProcess(const QString& name)
{
    if (!m_rosProcesses.contains(name)) {
        QMessageBox::critical(this, "错误", "未找到名为" + name + "的ROS进程配置！");
        return;
    }

    RosProcessInfo& info = m_rosProcesses[name];

    // 检查进程是否已经在运行
    if (info.process->state() == QProcess::Running) {
        QMessageBox::information(this, "提示", info.name + "已经在运行中！");
        return;
    }

    // 设置要执行的命令，使用stdbuf禁用输出缓冲
    QString fullCommand = QString("cd %1 && source %1/devel/setup.bash && stdbuf -o0 -e0 %2").arg(info.workspace, info.command);

    // 显示启动信息
    appendRosInfo(QString("启动ROS进程: %1").arg(name));

    // 在终端中执行命令
    info.process->start("bash", QStringList() << "-c" << fullCommand);
    if (info.process->waitForStarted()) {
        appendRosInfo(QString("%1 启动成功").arg(name));
        //QMessageBox::information(this, "提示", info.name + "启动命令已发送！");
    } else {
        appendRosInfo(QString("%1 启动失败: %2").arg(name, info.process->errorString()));
        QMessageBox::critical(this, "错误", "无法启动" + info.name + "进程：" + info.process->errorString());
    }
}

void MainWindow::stopRosProcess(const QString& name)
{
    if (!m_rosProcesses.contains(name)) {
        QMessageBox::critical(this, "错误", "未找到名为" + name + "的ROS进程配置！");
        return;
    }

    RosProcessInfo& info = m_rosProcesses[name];

    // 检查进程是否正在运行
    if (info.process->state() != QProcess::Running) {
        QMessageBox::information(this, "提示", info.name + "进程未运行！");
        return;
    }

    // 显示停止信息
    appendRosInfo(QString("停止ROS进程: %1").arg(name));

    // 发送终止信号
    info.process->terminate();

    // 等待进程结束，最多等待3秒
    if (!info.process->waitForFinished(3000)) {
        // 如果3秒内进程未结束，强制结束
        appendRosInfo(QString("%1 强制终止").arg(name));
        info.process->kill();
    } else {
        appendRosInfo(QString("%1 已停止").arg(name));
    }
}

bool MainWindow::isRosProcessRunning(const QString& name)
{
    if (!m_rosProcesses.contains(name)) {
        return false;
    }

    return m_rosProcesses[name].process->state() == QProcess::Running;
}

// 通用进程信号处理函数
void MainWindow::onRosProcessFinished(const QString& name, int exitCode, QProcess::ExitStatus exitStatus)
{
    if (exitStatus == QProcess::CrashExit) {
        QMessageBox::warning(this, "警告", name + "进程意外终止！");
    } else {
        //QMessageBox::information(this, "提示", name + "进程已停止，退出码：" + QString::number(exitCode));
    }
}

void MainWindow::onRosProcessError(const QString& name, QProcess::ProcessError error)
{
    QMessageBox::critical(this, "错误", name + "进程错误：" + m_rosProcesses[name].process->errorString());
}

void MainWindow::appendRosInfo(const QString& message)
{
    QString timestamp = QDateTime::currentDateTime().toString("hh:mm:ss");
    QString formattedMessage = QString("[%1] %2").arg(timestamp, message);
    ui->rosmessage->append(formattedMessage);

    // 自动滚动到底部
    QTextCursor cursor = ui->rosmessage->textCursor();
    cursor.movePosition(QTextCursor::End);
    ui->rosmessage->setTextCursor(cursor);

}

// 处理ROS进程标准输出
void MainWindow::onRosProcessOutput(const QString& name)
{
    if (!m_rosProcesses.contains(name)) {
        return;
    }

    QProcess* process = m_rosProcesses[name].process;
    QByteArray output = process->readAllStandardOutput();
    QString outputText = QString::fromLocal8Bit(output).trimmed();

    if (!outputText.isEmpty()) {
        QString message = QString("[%1] %2").arg(name, outputText);
        appendRosInfo(message);
    }
}

// 处理ROS进程错误输出
void MainWindow::onRosProcessErrorOutput(const QString& name)
{
    if (!m_rosProcesses.contains(name)) {
        return;
    }

    QProcess* process = m_rosProcesses[name].process;
    QByteArray errorOutput = process->readAllStandardError();
    QString errorText = QString::fromLocal8Bit(errorOutput).trimmed();

    if (!errorText.isEmpty()) {
        QString timestamp = QDateTime::currentDateTime().toString("hh:mm:ss");
        QString message = QString("[%1 ERROR] %2").arg(name, errorText);
        QString formattedMessage = QString("[%1] %2").arg(timestamp, message);


        QTextCursor cursor = ui->rosmessage->textCursor();
        cursor.movePosition(QTextCursor::End);

        // 自动滚动到底部
        cursor.movePosition(QTextCursor::End);
        ui->rosmessage->setTextCursor(cursor);
    }
}

// 实现启动雷达的槽函数
void MainWindow::on_startRadarButton_clicked()
{

    startRosProcess("radar");
}

void MainWindow::on_stoptRadarButton_clicked()
{
    stopRosProcess("radar");
}

void MainWindow::on_startlocalization_clicked()
{
    startRosProcess("localization");
}


void MainWindow::on_stoplocalization_clicked()
{
    stopRosProcess("localization");
}


void MainWindow::on_pub_initpos_clicked()
{
    startRosProcess("initpos");
}


void MainWindow::on_startmovebase_clicked()
{
    startRosProcess("movebase");
}


void MainWindow::on_stopmovebase_clicked()
{
    stopRosProcess("movebase");
}


void MainWindow::on_startagv_clicked()
{
    startRosProcess("agv");
}


void MainWindow::on_stopagv_clicked()
{
    stopRosProcess("agv");
}


void MainWindow::on_stopall_clicked()
{
    stopRosProcess("radar");
    stopRosProcess("localization");
    stopRosProcess("movebase");
    stopRosProcess("agv");
}



void MainWindow::on_stopnavigation_clicked()
{
    stopRosProcess("navigation");
    m_navStatusTimer->stop();
    ui->robotstatus->append("🛑 导航已停止");
    m_robotStatusCheckTimer->stop();
}

// 初始化ROS节点
void MainWindow::initRosNode()
{
   try {
       // 检查ROS是否已经初始化
       if (!ros::isInitialized()) {
           int argc = 0;
           char** argv = nullptr;
           ros::init(argc, argv, "agv_ui_node", ros::init_options::NoSigintHandler);
           appendRosInfo("ROS节点初始化成功");
       }

       m_rosNode = std::make_unique<ros::NodeHandle>();
       appendRosInfo("ROS NodeHandle创建成功");
       m_nav_status_subscriber = m_rosNode->subscribe<std_msgs::String>("/navigation_status", 1000, &MainWindow::navStatusCallback, this);

       // 启动ROS后台线程
       m_rosThread = std::thread([this]() {
           ros::Rate rate(10); // 10Hz
           appendRosInfo("ROS后台线程启动");
           while (ros::ok() && m_rosNode) {
               ros::spinOnce();
               rate.sleep();
           }
       });

       m_rosInitialized = true;
       appendRosInfo("ROS节点初始化完成");

   } catch (const std::exception& e) {
       appendRosInfo(QString("ROS节点初始化失败: %1").arg(e.what()));
       m_rosInitialized = false;
   }
}



// 添加TF监听器初始化函数
void MainWindow::initTfListener()
{
    try {
        // 创建TF缓冲区和监听器
        m_tfBuffer = std::make_unique<tf2_ros::Buffer>();
        m_tfListener = std::make_unique<tf2_ros::TransformListener>(*m_tfBuffer);

        // 创建定时器，定期查询TF变换
        m_tfUpdateTimer = new QTimer(this);
        connect(m_tfUpdateTimer, &QTimer::timeout, this, &MainWindow::updateTfTransform);
        m_tfUpdateTimer->start(500); // 每500ms更新一次

        appendRosInfo("TF监听器初始化成功");
    } catch (const std::exception& e) {
        appendRosInfo(QString("TF监听器初始化失败: %1").arg(e.what()));
    }
}


// 添加TF变换更新函数
void MainWindow::updateTfTransform()
{
    if (!m_rosInitialized || !m_rosNode) {
        return;
    }

    try {
        // 查询body_foot相对于map的变换
        geometry_msgs::TransformStamped transformStamped;
        transformStamped = m_tfBuffer->lookupTransform("map", "body_foot", ros::Time(0), ros::Duration(0.5));

        // 提取位置和姿态信息
        double x = transformStamped.transform.translation.x;
        double y = transformStamped.transform.translation.y;
        double z = transformStamped.transform.translation.z;

        double qx = transformStamped.transform.rotation.x;
        double qy = transformStamped.transform.rotation.y;
        double qz = transformStamped.transform.rotation.z;
        double qw = transformStamped.transform.rotation.w;

        // 使用QMetaObject::invokeMethod确保UI更新在主线程执行
        QMetaObject::invokeMethod(this, [this, x, y, z, qx, qy, qz, qw]() {
            // 更新UI显示
            QString posText = QString("AGV位置信息：\n");
            posText += QString("X: %1\n").arg(x, 0, 'f', 3);
            posText += QString("Y: %1\n").arg(y, 0, 'f', 3);
            posText += QString("Z: %1\n").arg(z, 0, 'f', 3);
            posText += QString("\n姿态（四元数）：\n");
            posText += QString("X: %1\n").arg(qx, 0, 'f', 3);
            posText += QString("Y: %1\n").arg(qy, 0, 'f', 3);
            posText += QString("Z: %1\n").arg(qz, 0, 'f', 3);
            posText += QString("W: %1\n").arg(qw, 0, 'f', 3);

            if (ui->agv_pos) {
                ui->agv_pos->setPlainText(posText);
            }
        }, Qt::QueuedConnection);

    } catch (tf2::TransformException &ex) {
        // 忽略TF变换异常，不进行UI更新
        return;
    }
}

void MainWindow::startRoscore()
{
    // 检查是否已经运行了roscore
    QProcess checkProcess;
    checkProcess.start("bash", QStringList() << "-c" << "ps aux | grep roscore | grep -v grep");
    checkProcess.waitForFinished();
    QString output = checkProcess.readAllStandardOutput();

    if (output.isEmpty()) {
        // 没有运行roscore，启动它
        QProcess* roscoreProcess = new QProcess(this);
        roscoreProcess->start("roscore");
        m_rosProcesses["roscore"] = {roscoreProcess, "roscore"};
    }
}

void MainWindow::on_stop_camera_btn_clicked()
{
    if (cameraRunning) {
        cameraTimer->stop();

        // 安全关闭相机，添加异常处理
        try {
            if (zedCamera.isOpened()) {
                // 先停止数据流
                zedCamera.disableRecording();
                // 然后关闭相机
                zedCamera.close();
            }
        } catch (const std::exception& e) {
            std::cout << "相机关闭时出现异常: " << e.what() << std::endl;
        } catch (...) {
            std::cout << "相机关闭时出现未知异常" << std::endl;
        }

        cameraRunning = false;
        statusLabel->setText("状态：相机已停止");
        ui->save_image_btn->setEnabled(false);
        ui->stop_camera_btn->setEnabled(false);
        ui->start_camera_btn->setEnabled(true);
    }
}


void MainWindow::on_save_image_btn_clicked()
{
    if (cameraRunning) {
        saveImages();
    }
}

// 相机初始化函数（基于enter_save.cpp）
bool MainWindow::initZEDCamera()
{
    // 创建输出目录
    std::string output_dir = "output";

    // 使用C++17文件系统库创建目录
    std::error_code ec;
    std::filesystem::create_directories(output_dir + "/left_image", ec);
    std::filesystem::create_directories(output_dir + "/right_image", ec);

    if (ec) {
        std::cout << "创建输出目录失败: " << ec.message() << std::endl;
        statusLabel->setText("状态：创建目录失败");
        return false;
    }

    // 初始化ZED相机参数 - 使用CPU友好模式
    sl::InitParameters init_params;
    init_params.camera_resolution = sl::RESOLUTION::HD720;  // 降低分辨率减轻负载
    init_params.depth_mode = sl::DEPTH_MODE::PERFORMANCE;   // 性能模式而非神经模式
    init_params.depth_minimum_distance = 0.5; // 米
    init_params.depth_maximum_distance = 1.0; // 米
    init_params.coordinate_units = sl::UNIT::METER;
    init_params.sdk_verbose = false;  // 关闭详细日志减少输出
    init_params.sensors_required = false;  // 禁用传感器要求
    init_params.enable_image_enhancement = false;  // 禁用图像增强

    sl::ERROR_CODE status = zedCamera.open(init_params);
    if (status != sl::ERROR_CODE::SUCCESS) {
        std::cout << "无法打开相机: " << status << std::endl;

        // 如果GPU模式失败，尝试更保守的设置
        init_params.depth_mode = sl::DEPTH_MODE::NONE;  // 完全禁用深度计算
        status = zedCamera.open(init_params);

        if (status != sl::ERROR_CODE::SUCCESS) {
            statusLabel->setText("状态：相机打开失败");
            return false;
        }
    }

    std::cout << "ZED相机初始化成功" << std::endl;
    return true;
}

// 保存图像函数（基于enter_save.cpp的保存逻辑）
void MainWindow::saveImages()
{
    if (!cameraRunning) return;

    sl::CameraInformation cam_info = zedCamera.getCameraInformation();
    sl::Resolution image_size = cam_info.camera_configuration.resolution;

    sl::Mat left_image(image_size.width, image_size.height, sl::MAT_TYPE::U8_C4);
    sl::Mat right_image(image_size.width, image_size.height, sl::MAT_TYPE::U8_C4);

    // 获取当前帧
    if (zedCamera.grab() == sl::ERROR_CODE::SUCCESS) {
        zedCamera.retrieveImage(left_image, sl::VIEW::LEFT, sl::MEM::CPU);
        zedCamera.retrieveImage(right_image, sl::VIEW::RIGHT, sl::MEM::CPU);

        // 转换为OpenCV格式
        cv::Mat left_bgr(left_image.getHeight(), left_image.getWidth(), CV_8UC4, left_image.getPtr<uchar>());
        cv::Mat right_bgr(right_image.getHeight(), right_image.getWidth(), CV_8UC4, right_image.getPtr<uchar>());

        // 保存图像
        std::string left_image_filename = "output/left_image/left_" + std::to_string(saveIndex) + ".png";
        std::string right_image_filename = "output/right_image/right_" + std::to_string(saveIndex) + ".png";

        cv::imwrite(left_image_filename, left_bgr);
        cv::imwrite(right_image_filename, right_bgr);

        std::cout << "已保存图像 " << saveIndex << std::endl;
        statusLabel->setText(QString("状态：已保存图像 %1").arg(saveIndex));
        saveIndex++;
    }
}

void MainWindow::on_X_scan_btn_clicked()
{
    // 保存当前位置
    double originalX = currentWayPoint.cartPos.position.x;
    double originalY = currentWayPoint.cartPos.position.y;
    double originalZ = currentWayPoint.cartPos.position.z;

    // 扫描距离：5cm = 0.05m
    double scanDistance = 0.05;

    // 第一步：向X正方向移动5cm
    double targetX1 = originalX + scanDistance;

    double startpoint[6] = {};
    for (int i = 0; i < 6; ++i)
    {
        startpoint[i] = currentJointAngles[i];
    }

    // 目标位置
    aubo_robot_namespace::Pos targetpos1;
    targetpos1.x = targetX1;
    targetpos1.y = originalY;
    targetpos1.z = originalZ;

    // 目标姿态（保持不变）
    aubo_robot_namespace::Ori targetori;
    aubo_robot_namespace::Rpy targetrpy;
    targetrpy.rx = currentRpy.rx;
    targetrpy.ry = currentRpy.ry;
    targetrpy.rz = currentRpy.rz;
    robotService.RPYToQuaternion(targetrpy,targetori);

    aubo_robot_namespace::wayPoint_S waypointIK;
    robotService.robotServiceRobotIk(startpoint,targetpos1,targetori,waypointIK);
    robotService.robotServiceJointMove(waypointIK,true);

    // 等待运动完成
    QThread::msleep(500);

    // 第二步：向X反方向移动5cm（回到原点）
    double targetX2 = originalX;

    for (int i = 0; i < 6; ++i)
    {
        startpoint[i] = currentJointAngles[i];
    }

    aubo_robot_namespace::Pos targetpos2;
    targetpos2.x = targetX2;
    targetpos2.y = originalY;
    targetpos2.z = originalZ;

    robotService.robotServiceRobotIk(startpoint,targetpos2,targetori,waypointIK);
    robotService.robotServiceJointMove(waypointIK,true);

    std::cout << "X轴扫描完成：正方向5cm -> 返回原点" << std::endl;
}


void MainWindow::on_Y_scan_btn_clicked()
{
    // 保存当前位置
    double originalX = currentWayPoint.cartPos.position.x;
    double originalY = currentWayPoint.cartPos.position.y;
    double originalZ = currentWayPoint.cartPos.position.z;

    // 扫描距离：5cm = 0.05m
    double scanDistance = 0.05;

    // 第一步：向Y正方向移动5cm
    double targetY1 = originalY + scanDistance;

    double startpoint[6] = {};
    for (int i = 0; i < 6; ++i)
    {
        startpoint[i] = currentJointAngles[i];
    }

    // 目标位置
    aubo_robot_namespace::Pos targetpos1;
    targetpos1.x = originalX;
    targetpos1.y = targetY1;
    targetpos1.z = originalZ;

    // 目标姿态（保持不变）
    aubo_robot_namespace::Ori targetori;
    aubo_robot_namespace::Rpy targetrpy;
    targetrpy.rx = currentRpy.rx;
    targetrpy.ry = currentRpy.ry;
    targetrpy.rz = currentRpy.rz;
    robotService.RPYToQuaternion(targetrpy,targetori);

    aubo_robot_namespace::wayPoint_S waypointIK;
    robotService.robotServiceRobotIk(startpoint,targetpos1,targetori,waypointIK);
    robotService.robotServiceJointMove(waypointIK,true);

    // 等待运动完成
    QThread::msleep(500);

    // 第二步：向Y反方向移动5cm（回到原点）
    double targetY2 = originalY;

    for (int i = 0; i < 6; ++i)
    {
        startpoint[i] = currentJointAngles[i];
    }

    aubo_robot_namespace::Pos targetpos2;
    targetpos2.x = originalX;
    targetpos2.y = targetY2;
    targetpos2.z = originalZ;

    robotService.robotServiceRobotIk(startpoint,targetpos2,targetori,waypointIK);
    robotService.robotServiceJointMove(waypointIK,true);

    std::cout << "Y轴扫描完成：正方向5cm -> 返回原点" << std::endl;
}

// ==================== 视频处理线程实现 ====================
VideoProcessingThread::VideoProcessingThread(QObject *parent)
    : QThread(parent), m_running(false), m_detectionEnabled(true), m_camera(nullptr)
{
    // 初始化检测参数
    m_detectionParams = { 33, 28, 67, 133, 208, 64 };
}

VideoProcessingThread::~VideoProcessingThread()
{
    stopProcessing();
}

void VideoProcessingThread::startProcessing()
{
    QMutexLocker locker(&m_mutex);
    if (!m_running) {
        m_running = true;
        if (!isRunning()) {
            start();
        } else {
            m_condition.wakeOne();
        }
    }
}

void VideoProcessingThread::stopProcessing()
{
    QMutexLocker locker(&m_mutex);
    m_running = false;
    m_condition.wakeOne();
    locker.unlock();
    wait();
}

void VideoProcessingThread::setDetectionEnabled(bool enabled)
{
    QMutexLocker locker(&m_mutex);
    m_detectionEnabled = enabled;
}

void VideoProcessingThread::setDetectionParams(const DetectionParams& params)
{
    QMutexLocker locker(&m_mutex);
    m_detectionParams = params;
}

// ... existing code ...
void VideoProcessingThread::run()
{
    // 使用主窗口的相机实例，而不是创建新的
    if (!m_camera) {
        emit errorOccurred("相机未初始化");
        return;
    }

    sl::Mat zed_image;
    cv::Mat cv_image;

    while (true) {
        {
            QMutexLocker locker(&m_mutex);
            if (!m_running) break;
        }

        // 获取新图像
        if (m_camera->grab() == sl::ERROR_CODE::SUCCESS) {
            // 获取左眼图像
            m_camera->retrieveImage(zed_image, sl::VIEW::LEFT);

            // 转换为OpenCV格式
            cv_image = slMat2cvMat(zed_image);

            // 如果检测功能开启，进行目标检测
            cv::Mat displayImage;
            int detectedCount = 0;

            bool detectionEnabled;
            DetectionParams detectionParams;
            {
                QMutexLocker locker(&m_mutex);
                detectionEnabled = m_detectionEnabled;
                detectionParams = m_detectionParams;
            }

            if (detectionEnabled) {
                cv::Mat vis_colorMask, vis_glare, vis_mask, vis_morph;
                auto boxes = detectRects(cv_image, vis_colorMask, vis_glare, vis_mask, vis_morph);
                detectedCount = boxes.size();

                // 创建带检测框的可视化图像
                displayImage = cv_image.clone();
                for (auto& b : boxes) {
                    cv::rectangle(displayImage, b, cv::Scalar(0, 255, 255), 2);

                    // 在检测框上方显示检测信息
std::string info = "W:" + std::to_string(b.width) + " H:" + std::to_string(b.height);
                    cv::putText(displayImage, info, cv::Point(b.x, b.y-5),
                               cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
                }
            } else {
                displayImage = cv_image.clone();
            }

            // 调整图像大小以适应显示区域
            cv::Mat resized;
            cv::resize(displayImage, resized, cv::Size(580, 360));

            // 转换为Qt图像格式
            if (resized.channels() == 4) {
                cv::cvtColor(resized, resized, cv::COLOR_BGRA2RGB);
            } else if (resized.channels() == 3) {
                cv::cvtColor(resized, resized, cv::COLOR_BGR2RGB);
            }
            QImage qtImage(resized.data, resized.cols, resized.rows, resized.step, QImage::Format_RGB888);

            // 发送处理后的图像到主线程
            emit frameProcessed(qtImage, detectedCount);
        }

        // 控制帧率，避免过度占用CPU
        msleep(60); // 约30fps
    }
}


cv::Mat VideoProcessingThread::slMat2cvMat(sl::Mat& input)
{
    // 将sl::Mat转换为cv::Mat
    int cv_type = -1;
    switch (input.getDataType()) {
        case sl::MAT_TYPE::F32_C1: cv_type = CV_32FC1; break;
        case sl::MAT_TYPE::F32_C2: cv_type = CV_32FC2; break;
        case sl::MAT_TYPE::F32_C3: cv_type = CV_32FC3; break;
        case sl::MAT_TYPE::F32_C4: cv_type = CV_32FC4; break;
        case sl::MAT_TYPE::U8_C1: cv_type = CV_8UC1; break;
        case sl::MAT_TYPE::U8_C2: cv_type = CV_8UC2; break;
        case sl::MAT_TYPE::U8_C3: cv_type = CV_8UC3; break;
        case sl::MAT_TYPE::U8_C4: cv_type = CV_8UC4; break;
        default: break;
    }

    return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(sl::MEM::CPU));
}

std::vector<cv::Rect> VideoProcessingThread::detectRects(const cv::Mat& img, cv::Mat& vis_colorMask, cv::Mat& vis_glare, cv::Mat& vis_mask, cv::Mat& vis_morph)
{
    // 从shipinliu.cpp复制的检测逻辑
cv::Mat hsv;
    cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);

    // 颜色分割
    cv::Mat colorMask;
    cv::inRange(hsv, cv::Scalar(m_detectionParams.h_min, m_detectionParams.s_min, m_detectionParams.v_min),
                cv::Scalar(m_detectionParams.h_max, 255, 255), colorMask);

    // 高光剔除
    cv::Mat glareMask;
    cv::inRange(hsv, cv::Scalar(0, 0, m_detectionParams.glare_v_min),
                cv::Scalar(180, m_detectionParams.glare_s_max, 255), glareMask);

    // 合并掩码
    cv::Mat finalMask = colorMask & (~glareMask);

    // 形态学处理
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::Mat morphMask;
    cv::morphologyEx(finalMask, morphMask, cv::MORPH_CLOSE, kernel);

    // 连通域分析
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(morphMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<cv::Rect> boxes;
    for (auto& contour : contours) {
        if (cv::contourArea(contour) > 100) { // 面积阈值
            cv::Rect rect = cv::boundingRect(contour);
            boxes.push_back(rect);
        }
    }

    // 可视化图像（可选）
    vis_colorMask = colorMask;
    vis_glare = glareMask;
    vis_mask = finalMask;
    vis_morph = morphMask;

    return boxes;
}

// 视频处理线程信号槽处理
void MainWindow::onVideoFrameProcessed(const QImage& image, int detectedCount)
{
    // 在主线程中安全地更新UI
    if (detectionLabel) {
        detectionLabel->setPixmap(QPixmap::fromImage(image));
        // 使用状态栏或工具提示显示检测数量，避免覆盖图像
        detectionLabel->setToolTip(QString("检测到 %1 个目标").arg(detectedCount));
        ui->statusLabel->setText(QString("实时检测中 - 检测到 %1 个目标").arg(detectedCount));
    }
}

void MainWindow::onVideoError(const QString& error)
{
    // 处理视频处理错误
    ui->statusLabel->setText(error);
}

// 修改相机控制函数
void MainWindow::on_start_camera_btn_clicked()
{
    if (!cameraRunning) {
        // 初始化ZED相机
        sl::InitParameters init_params;
        init_params.camera_resolution = sl::RESOLUTION::HD720;
        init_params.camera_fps = 30;

        sl::ERROR_CODE err = zedCamera.open(init_params);
        if (err != sl::ERROR_CODE::SUCCESS) {
            QString errorStr = QString("无法打开ZED相机: %1").arg(static_cast<int>(err));
            ui->statusLabel->setText(errorStr);
            return;
        }

        // 将相机实例传递给视频处理线程
        m_videoThread->setCamera(&zedCamera);

        // 设置检测参数
        m_videoThread->setDetectionEnabled(detectionEnabled);
        m_videoThread->setDetectionParams(detectionParams);
        m_videoThread->startProcessing();
        cameraRunning = true;
        ui->statusLabel->setText("相机已启动（多线程处理）");

        // 清空显示标签
        if (detectionLabel) {
            detectionLabel->clear();
        }
    }
}

void MainWindow::updateCameraFrame()
{
    if (!cameraRunning) return;

    sl::CameraInformation cam_info = zedCamera.getCameraInformation();
    sl::Resolution image_size = cam_info.camera_configuration.resolution;

    sl::Mat left_image(image_size.width, image_size.height, sl::MAT_TYPE::U8_C4);

    // 获取当前帧
    if (zedCamera.grab() == sl::ERROR_CODE::SUCCESS) {
        zedCamera.retrieveImage(left_image, sl::VIEW::LEFT, sl::MEM::CPU);

        // 转换为OpenCV格式
        cv::Mat cv_image(left_image.getHeight(), left_image.getWidth(), CV_8UC4, left_image.getPtr<uchar>());

        // 如果检测功能开启，进行目标检测
        cv::Mat displayImage = cv_image.clone();

        // 调整图像大小以适应显示区域
        cv::Mat resized;
        cv::resize(displayImage, resized, cv::Size(580, 360));

        // 转换为Qt图像格式并显示
        cv::cvtColor(resized, resized, cv::COLOR_BGRA2RGB);
        QImage qtImage(resized.data, resized.cols, resized.rows, resized.step, QImage::Format_RGB888);

    }
}

void MainWindow::on_savepos_clicked()
{
    // 生成文件名（使用时间戳）
    QDateTime currentTime = QDateTime::currentDateTime();
    QString fileName = QString("./pos/pose_%1.txt").arg(currentTime.toString("yyyyMMdd_hhmmss"));

    // 创建并打开文件
    QFile file(fileName);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        return;
    }

    // 直接使用QIODevice的write方法保存中文，避免QTextStream的编码问题
    QByteArray content;

    // 写入文件头信息
    content.append("# 机械臂位姿数据文件\n");
    content.append(QString("# 生成时间: %1\n").arg(currentTime.toString("yyyy-MM-dd hh:mm:ss")).toUtf8());
    content.append("# 格式: 位置(X,Y,Z) 姿态(Rx,Ry,Rz) 关节角度(J1-J6)\n");
    content.append("========================================\n\n");

    // 写入位置信息
    content.append("位置信息 (单位: 米):\n");
    content.append(QString("X: %1\n").arg(currentWayPoint.cartPos.position.x, 0, 'f', 6).toUtf8());
    content.append(QString("Y: %1\n").arg(currentWayPoint.cartPos.position.y, 0, 'f', 6).toUtf8());
    content.append(QString("Z: %1\n").arg(currentWayPoint.cartPos.position.z, 0, 'f', 6).toUtf8());
    content.append("\n");

    // 写入姿态信息（欧拉角，转换为度）
    content.append("姿态信息 (单位: 度):\n");
    content.append(QString("Rx: %1\n").arg(currentRpy.rx * 180.0 / M_PI, 0, 'f', 6).toUtf8());
    content.append(QString("Ry: %1\n").arg(currentRpy.ry * 180.0 / M_PI, 0, 'f', 6).toUtf8());
    content.append(QString("Rz: %1\n").arg(currentRpy.rz * 180.0 / M_PI, 0, 'f', 6).toUtf8());
    content.append("\n");

    // 写入关节角度信息
    content.append("关节角度 (单位: 度):\n");
    for (int i = 0; i < 6; ++i) {
        content.append(QString("J%1: %2\n").arg(i+1).arg(currentJointAngles[i] * 180.0 / M_PI, 0, 'f', 6).toUtf8());
    }
    content.append("\n");

    // 写入四元数姿态信息（如果需要）
    aubo_robot_namespace::Ori quaternion;
    robotService.RPYToQuaternion(currentRpy, quaternion);
    content.append("四元数姿态:\n");
    content.append(QString("Qx: %1\n").arg(quaternion.x, 0, 'f', 6).toUtf8());
    content.append(QString("Qy: %1\n").arg(quaternion.y, 0, 'f', 6).toUtf8());
    content.append(QString("Qz: %1\n").arg(quaternion.z, 0, 'f', 6).toUtf8());
    content.append(QString("Qw: %1\n").arg(quaternion.w, 0, 'f', 6).toUtf8());
    content.append("\n");

    file.write(content);
    file.close();

    // 更新状态栏
    ui->saveposstatus->setText("位姿保存成功: " + fileName);

    // 显示保存成功的信息
    //QMessageBox::information(this, "保存成功", QString("机械臂位姿已成功保存到文件:\n%1").arg(fileName));
}

void MainWindow::updateNavStatusDisplay()
{

}

void MainWindow::point_move2()
{
    double wp1[6] = {};
    wp1[0] = -126*M_PI/180;
    wp1[1] = 45*M_PI/180;
    wp1[2] = 80*M_PI/180;
    wp1[3] = 23*M_PI/180;
    wp1[4] = 90*M_PI/180;
    wp1[5] = 0.0;
    robotService.robotServiceJointMove(wp1, true);

}

void MainWindow::point_nav3()
{
    double wp1[6] = {};
    wp1[0] = -126*M_PI/180;
    wp1[1] = 45*M_PI/180;
    wp1[2] = 80*M_PI/180;
    wp1[3] = 23*M_PI/180;
    wp1[4] = 90*M_PI/180;
    wp1[5] = 0.0;
    robotService.robotServiceJointMove(wp1, true);

    double wp2[6] = {};
    wp2[0] = -117*M_PI/180;
    wp2[1] = 33.7*M_PI/180;
    wp2[2] = 83*M_PI/180;
    wp2[3] = 25.6*M_PI/180;
    wp2[4] = 76*M_PI/180;
    wp2[5] = 4.1*M_PI/180;
    robotService.robotServiceJointMove(wp2, true);

}

void MainWindow::point_move_b4()
{
    double wpmove_b3[6] = {};
    wpmove_b3[0] = -163*M_PI/180;
    wpmove_b3[1] = -16*M_PI/180;
    wpmove_b3[2] = -11*M_PI/180;
    wpmove_b3[3] = 1*M_PI/180;
    wpmove_b3[4] = 90*M_PI/180;
    wpmove_b3[5] = 0*M_PI/180;
    robotService.robotServiceJointMove(wpmove_b3, true);
}

void MainWindow::point_move_b5()
{
    double wpmove_b3[6] = {};
    wpmove_b3[0] = -170*M_PI/180;
    wpmove_b3[1] = 16*M_PI/180;
    wpmove_b3[2] = 16*M_PI/180;
    wpmove_b3[3] = 1*M_PI/180;
    wpmove_b3[4] = 90*M_PI/180;
    wpmove_b3[5] = 0*M_PI/180;
    robotService.robotServiceJointMove(wpmove_b3, true);
}

void MainWindow::point_move_b8()
{
    double wpmove_b8[6] = {};
    wpmove_b8[0] = -90*M_PI/180;
    wpmove_b8[1] = 45*M_PI/180;
    wpmove_b8[2] = 80*M_PI/180;
    wpmove_b8[3] = 35*M_PI/180;
    wpmove_b8[4] = 90*M_PI/180;
    wpmove_b8[5] = 0.0;
    robotService.robotServiceJointMove(wpmove_b8,true);
}

void MainWindow::point_move_b8_1()
{
    double wpmove_b8[6] = {};
    wpmove_b8[0] = -166*M_PI/180;
    wpmove_b8[1] = -16*M_PI/180;
    wpmove_b8[2] = -11*M_PI/180;
    wpmove_b8[3] = -10*M_PI/180;
    wpmove_b8[4] = 90*M_PI/180;
    wpmove_b8[5] = 0.0;
    robotService.robotServiceJointMove(wpmove_b8,true);
}


void MainWindow::point_move_b12()
{
    double wpmove_b11[6] = {};
    wpmove_b11[0] = -90*M_PI/180;
    wpmove_b11[1] = 45*M_PI/180;
    wpmove_b11[2] = 80*M_PI/180;
    wpmove_b11[3] = 10*M_PI/180;
    wpmove_b11[4] = 90*M_PI/180;
    wpmove_b11[5] = 0.0;
    robotService.robotServiceJointMove(wpmove_b11,true);
}

void MainWindow::point_move_b12_1()
{
    double wpmove_b11[6] = {};
    wpmove_b11[0] = -79*M_PI/180;
    wpmove_b11[1] = -3.67*M_PI/180;
    wpmove_b11[2] = 63.2*M_PI/180;
    wpmove_b11[3] = 35*M_PI/180;
    wpmove_b11[4] = 90*M_PI/180;
    wpmove_b11[5] = 0.0;
    robotService.robotServiceJointMove(wpmove_b11,true);
}

void MainWindow::point_move_b12_2()
{
    double wpmove_b11[6] = {};
    wpmove_b11[0] = -76.0*M_PI/180;
    wpmove_b11[1] = -0.7*M_PI/180;
    wpmove_b11[2] = 60.3*M_PI/180;
    wpmove_b11[3] = 35*M_PI/180;
    wpmove_b11[4] = 90*M_PI/180;
    wpmove_b11[5] = 0.0;
    robotService.robotServiceJointMove(wpmove_b11,true);
}


void MainWindow::point_move_b17()
{
    double wp1[6] = {};
    wp1[0] = -126*M_PI/180;
    wp1[1] = 45*M_PI/180;
    wp1[2] = 80*M_PI/180;
    wp1[3] = 23*M_PI/180;
    wp1[4] = 90*M_PI/180;
    wp1[5] = 0.0;
    robotService.robotServiceJointMove(wp1,true);

    double wp2[6] = {};
    wp2[0] = -117*M_PI/180;
    wp2[1] = 33.7*M_PI/180;
    wp2[2] = 83*M_PI/180;
    wp2[3] = 25.6*M_PI/180;
    wp2[4] = 76*M_PI/180;
    wp2[5] = 4.1*M_PI/180;
    robotService.robotServiceJointMove(wp2,true);
}

void MainWindow::point_move_b18_1()
{
    double wp1[6] = {};
    wp1[0] = -204.73*M_PI/180;
    wp1[1] = 53.4*M_PI/180;
    wp1[2] = 76.1*M_PI/180;
    wp1[3] = 0.662*M_PI/180;
    wp1[4] = 104.68*M_PI/180;
    wp1[5] = 0.0*M_PI/180;
    robotService.robotServiceJointMove(wp1,true);
}

void MainWindow::point_move_b18_2()
{
    double wp18_2[6] = {};
    wp18_2[0] = -185*M_PI/180;
    wp18_2[1] = 60*M_PI/180;
    wp18_2[2] = 80*M_PI/180;
    wp18_2[3] = 0.15*M_PI/180;
    wp18_2[4] = 85*M_PI/180;
    wp18_2[5] = 0.0*M_PI/180;
    robotService.robotServiceJointMove(wp18_2,true);
}

void MainWindow::point_move_b18_3()
{
    double wp18_3[6] = {};
    wp18_3[0] = -185*M_PI/180;
    wp18_3[1] = 60*M_PI/180;
    wp18_3[2] = 80*M_PI/180;
    wp18_3[3] = 0.0*M_PI/180;
    wp18_3[4] = 95*M_PI/180;
    wp18_3[5] = 0.0*M_PI/180;
    robotService.robotServiceJointMove(wp18_3,true);
}

void MainWindow::point_move_c1_1()
{
    double wpc1[6] = {};
    wpc1[0] = -65.0*M_PI/180;
    wpc1[1] = 17.5*M_PI/180;
    wpc1[2] = 56.5*M_PI/180;
    wpc1[3] = 50.8*M_PI/180;
    wpc1[4] = 90.0*M_PI/180;
    wpc1[5] = 0.0*M_PI/180;
    robotService.robotServiceJointMove(wpc1,true);
}

void MainWindow::point_move_c1_2()
{
    double wpc2[6] = {};
    wpc2[0] = -84.57*M_PI/180;
    wpc2[1] = 17.52*M_PI/180;
    wpc2[2] = 56.5*M_PI/180;
    wpc2[3] = 50.73*M_PI/180;
    wpc2[4] = 90.0*M_PI/180;
    wpc2[5] = 0.0*M_PI/180;
    robotService.robotServiceJointMove(wpc2,true);
}

void MainWindow::point_move_c1_3()
{
    double wpc3[6] = {};
    wpc3[0] = -105.08*M_PI/180;
    wpc3[1] = 17.52*M_PI/180;
    wpc3[2] = 56.5*M_PI/180;
    wpc3[3] = 50.73*M_PI/180;
    wpc3[4] = 90.0*M_PI/180;
    wpc3[5] = 0.0*M_PI/180;
    robotService.robotServiceJointMove(wpc3,true);
}

void MainWindow::point_move_c2()
{
    double wpc3[6] = {};
    wpc3[0] = -179.71*M_PI/180;
    wpc3[1] = 21.13*M_PI/180;
    wpc3[2] = 79.98*M_PI/180;
    wpc3[3] = 9.62*M_PI/180;
    wpc3[4] = 90.0*M_PI/180;
    wpc3[5] = 0.0*M_PI/180;
    robotService.robotServiceJointMove(wpc3,true);
}

void MainWindow::point_nav_d1()
{
    double wpc3[6] = {};
    wpc3[0] = -11.078*M_PI/180;
    wpc3[1] = 45.0*M_PI/180;
    wpc3[2] = 80.0*M_PI/180;
    wpc3[3] = 48.0*M_PI/180;
    wpc3[4] = 90.0*M_PI/180;
    wpc3[5] = 0.0*M_PI/180;
    robotService.robotServiceJointMove(wpc3,true);
}

void MainWindow::point_nav_d2()
{
    double wpc3[6] = {};
    wpc3[0] = -146.55*M_PI/180;
    wpc3[1] = 45.0*M_PI/180;
    wpc3[2] = 80.0*M_PI/180;
    wpc3[3] = 42.0*M_PI/180;
    wpc3[4] = 90.0*M_PI/180;
    wpc3[5] = 0.0*M_PI/180;
    robotService.robotServiceJointMove(wpc3,true);
}

void MainWindow::point_nav_d3()
{
    double wpc3[6] = {};
    wpc3[0] = -18.5*M_PI/180;
    wpc3[1] = 45.0*M_PI/180;
    wpc3[2] = 80.0*M_PI/180;
    wpc3[3] = 55.0*M_PI/180;
    wpc3[4] = 90.0*M_PI/180;
    wpc3[5] = 0.0*M_PI/180;
    robotService.robotServiceJointMove(wpc3,true);
}

void MainWindow::point_nav_d4_1()
{
    double wp1[6] = {};
    wp1[0] = -169.78*M_PI/180;
    wp1[1] = 45.0*M_PI/180;
    wp1[2] = 80.0*M_PI/180;
    wp1[3] = 0.1*M_PI/180;
    wp1[4] = 90.0*M_PI/180;
    wp1[5] = 0.0*M_PI/180;
    robotService.robotServiceJointMove(wp1,true);
}

void MainWindow::point_nav_d4_2()
{
    double wp2[6] = {};
    wp2[0] = -169.78*M_PI/180;
    wp2[1] = 3.259*M_PI/180;
    wp2[2] = 42.274*M_PI/180;
    wp2[3] = 34.9*M_PI/180;
    wp2[4] = 90.0*M_PI/180;
    wp2[5] = 0.0*M_PI/180;
    robotService.robotServiceJointMove(wp2,true);
}

void MainWindow::point_nav_d5()
{
    double wp2[6] = {};
    wp2[0] = -132.47*M_PI/180;
    wp2[1] = 19.217*M_PI/180;
    wp2[2] = 75.016*M_PI/180;
    wp2[3] = 24.058*M_PI/180;
    wp2[4] = 90.0*M_PI/180;
    wp2[5] = 0.0*M_PI/180;
    robotService.robotServiceJointMove(wp2,true);
}

void MainWindow::point_nav_d5_1()
{
    double wp22[6] = {};
    wp22[0] = -132.47*M_PI/180;
    wp22[1] = 19.217*M_PI/180;
    wp22[2] = 75.016*M_PI/180;
    wp22[3] = 24.058*M_PI/180;
    wp22[4] = 87.0*M_PI/180;
    wp22[5] = 0.0*M_PI/180;
    robotService.robotServiceJointMove(wp22,true);
}

void MainWindow::point_nav_d5_2()
{
    double wp22[6] = {};
    wp22[0] = -132.47*M_PI/180;
    wp22[1] = 19.217*M_PI/180;
    wp22[2] = 75.016*M_PI/180;
    wp22[3] = 24.058*M_PI/180;
    wp22[4] = 84.0*M_PI/180;
    wp22[5] = 0.0*M_PI/180;
    robotService.robotServiceJointMove(wp22,true);
}

void MainWindow::point_nav_d6()
{
    double wp2[6] = {};
    wp2[0] = -76.265*M_PI/180;
    wp2[1] = 22.5*M_PI/180;
    wp2[2] = 94.762*M_PI/180;
    wp2[3] = 35.0*M_PI/180;
    wp2[4] = 90.0*M_PI/180;
    wp2[5] = 0.0*M_PI/180;
    robotService.robotServiceJointMove(wp2,true);
}

void MainWindow::point_nav_d6_1()
{
    double wp2[6] = {};
    wp2[0] = -76.265*M_PI/180;
    wp2[1] = 22.5*M_PI/180;
    wp2[2] = 94.762*M_PI/180;
    wp2[3] = 35.0*M_PI/180;
    wp2[4] = 93.0*M_PI/180;
    wp2[5] = 0.0*M_PI/180;
    robotService.robotServiceJointMove(wp2,true);
}

void MainWindow::point_nav_d6_2()
{
    double wp2[6] = {};
    wp2[0] = -76.265*M_PI/180;
    wp2[1] = 22.5*M_PI/180;
    wp2[2] = 94.762*M_PI/180;
    wp2[3] = 35.0*M_PI/180;
    wp2[4] = 87.0*M_PI/180;
    wp2[5] = 0.0*M_PI/180;
    robotService.robotServiceJointMove(wp2,true);
}

void MainWindow::point_nav_d7()
{
    double wp2[6] = {};
    wp2[0] = -74.417*M_PI/180;
    wp2[1] = 45.0*M_PI/180;
    wp2[2] = 80.0*M_PI/180;
    wp2[3] = 43.57*M_PI/180;
    wp2[4] = 90.0*M_PI/180;
    wp2[5] = 0.0*M_PI/180;
    robotService.robotServiceJointMove(wp2,true);
}

void MainWindow::point_nav_d8()
{
    double wp2[6] = {};
    wp2[0] = -102.023*M_PI/180;
    wp2[1] = 14.73*M_PI/180;
    wp2[2] = 52.057*M_PI/180;
    wp2[3] = 35.0*M_PI/180;
    wp2[4] = 90.0*M_PI/180;
    wp2[5] = 0.0*M_PI/180;
    robotService.robotServiceJointMove(wp2,true);
}

void MainWindow::point_nav_d9()
{
    double wp2[6] = {};
    wp2[0] = -135.336*M_PI/180;
    wp2[1] = 39.116*M_PI/180;
    wp2[2] = 88.78*M_PI/180;
    wp2[3] = 12.618*M_PI/180;
    wp2[4] = 90.0*M_PI/180;
    wp2[5] = 0.0*M_PI/180;
    robotService.robotServiceJointMove(wp2,true);
}

void MainWindow::point_nav_e1()
{
    double wp2[6] = {};
    wp2[0] = -94.0*M_PI/180;
    wp2[1] = -4.368*M_PI/180;
    wp2[2] = 42.464*M_PI/180;
    wp2[3] = 48.656*M_PI/180;
    wp2[4] = 90.0*M_PI/180;
    wp2[5] = 0.0*M_PI/180;
    robotService.robotServiceJointMove(wp2,true);
}

void MainWindow::point_nav_e2()
{
    double wp2[6] = {};
    wp2[0] = -79.958*M_PI/180;
    wp2[1] = 26.967*M_PI/180;
    wp2[2] = 65.074*M_PI/180;
    wp2[3] = 35.0*M_PI/180;
    wp2[4] = 90.0*M_PI/180;
    wp2[5] = 0.0*M_PI/180;
    robotService.robotServiceJointMove(wp2,true);
}
void MainWindow::point_nav_e3()
{
    double wp2[6] = {};
    wp2[0] = -8.033*M_PI/180;
    wp2[1] = -30.389*M_PI/180;
    wp2[2] = 22.550*M_PI/180;
    wp2[3] = 35*M_PI/180;
    wp2[4] = 90.0*M_PI/180;
    wp2[5] = 0.0*M_PI/180;
    robotService.robotServiceJointMove(wp2,true);
}

void MainWindow::point_nav_e3_1()
{
    double wp2[6] = {};
    wp2[0] = -8.032832*M_PI/180;
    wp2[1] = -10.587358*M_PI/180;
    wp2[2] = 22.550*M_PI/180;
    wp2[3] = 35*M_PI/180;
    wp2[4] = 90.0*M_PI/180;
    wp2[5] = 0.0*M_PI/180;
    robotService.robotServiceJointMove(wp2,true);
}

void MainWindow::point_nav_e4()
{
    double wp2[6] = {};
    wp2[0] = -3.132052*M_PI/180;
    wp2[1] = -49.674436*M_PI/180;
    wp2[2] = 5.311648*M_PI/180;
    wp2[3] = 35.0*M_PI/180;
    wp2[4] = 90.0*M_PI/180;
    wp2[5] = 0.0*M_PI/180;
    robotService.robotServiceJointMove(wp2,true);
}

void MainWindow::point_nav_e5()
{
    double wp2[6] = {};
    wp2[0] = 3.723449*M_PI/180;
    wp2[1] = 9.076289*M_PI/180;
    wp2[2] = 62.985498*M_PI/180;
    wp2[3] = 34.998996*M_PI/180;
    wp2[4] = 90.0*M_PI/180;
    wp2[5] = 0.0*M_PI/180;
    robotService.robotServiceJointMove(wp2,true);
}

void MainWindow::point_nav_e5_1()
{
    double wp2[6] = {};
    wp2[0] = 3.723449*M_PI/180;
    wp2[1] = 25.743859*M_PI/180;
    wp2[2] = 62.985498*M_PI/180;
    wp2[3] = 34.998996*M_PI/180;
    wp2[4] = 90.0*M_PI/180;
    wp2[5] = 0.0*M_PI/180;
    robotService.robotServiceJointMove(wp2,true);
}


void MainWindow::point_nav_e6()
{
    double wp2[6] = {};
    wp2[0] = 2.793060*M_PI/180;
    wp2[1] = -35.103446*M_PI/180;
    wp2[2] = 16.123459*M_PI/180;
    wp2[3] = 34.998787*M_PI/180;
    wp2[4] = 90.0*M_PI/180;
    wp2[5] = 0.0*M_PI/180;
    robotService.robotServiceJointMove(wp2,true);
}
void MainWindow::point_nav_e7()
{
    double wp2[6] = {};
    wp2[0] = -71.027998*M_PI/180;
    wp2[1] = 20.001589*M_PI/180;
    wp2[2] = 54.998271*M_PI/180;
    wp2[3] = 34.998576*M_PI/180;
    wp2[4] = 90.0*M_PI/180;
    wp2[5] = 0.0*M_PI/180;
    robotService.robotServiceJointMove(wp2,true);
}
void MainWindow::point_nav_e8()
{
    double wp2[6] = {};
    wp2[0] = -71.027158*M_PI/180;
    wp2[1] = 20.005371*M_PI/180;
    wp2[2] = 78.231532*M_PI/180;
    wp2[3] = 35.0*M_PI/180;
    wp2[4] = 90.0*M_PI/180;
    wp2[5] = 0.0*M_PI/180;
    robotService.robotServiceJointMove(wp2,true);
}

void MainWindow::point_nav_e8_1()
{
    double wp2[6] = {};
    wp2[0] = -94.277012*M_PI/180;
    wp2[1] = 20.004320*M_PI/180;
    wp2[2] = 78.231532*M_PI/180;
    wp2[3] = 35.0*M_PI/180;
    wp2[4] = 90.0*M_PI/180;
    wp2[5] = 0.0*M_PI/180;
    robotService.robotServiceJointMove(wp2,true);
}

void MainWindow::point_nav_e9()
{
    double wp2[6] = {};
    wp2[0] = -94.274075*M_PI/180;
    wp2[1] = 15.030613*M_PI/180;
    wp2[2] = 79.227278*M_PI/180;
    wp2[3] = 35.0*M_PI/180;
    wp2[4] = 90.0*M_PI/180;
    wp2[5] = 0.0*M_PI/180;
    robotService.robotServiceJointMove(wp2,true);
}
void MainWindow::point_nav_e10()
{
    double wp2[6] = {};
    wp2[0] = -12.724711*M_PI/180;
    wp2[1] = 30.507194*M_PI/180;
    wp2[2] = 82.99718*M_PI/180;
    wp2[3] = 27.117060*M_PI/180;
    wp2[4] = 100.851746*M_PI/180;
    wp2[5] = 0.0*M_PI/180;
    robotService.robotServiceJointMove(wp2,true);
}

void MainWindow::point_nav_e11_1()
{
    double wp2[6] = {};
    wp2[0] = -39*M_PI/180;
    wp2[1] = 45.0*M_PI/180;
    wp2[2] = 80.0*M_PI/180;
    wp2[3] = 35.0*M_PI/180;
    wp2[4] = 90.0*M_PI/180;
    wp2[5] = 0.0*M_PI/180;
    robotService.robotServiceJointMove(wp2,true);
}

void MainWindow::point_nav_e11()
{
    double wp2[6] = {};
    wp2[0] = -36.0*M_PI/180;
    wp2[1] = 45.0*M_PI/180;
    wp2[2] = 59.24*M_PI/180;
    wp2[3] = -10.73*M_PI/180;
    wp2[4] = 96.0*M_PI/180;
    wp2[5] = 0.0*M_PI/180;
    robotService.robotServiceJointMove(wp2,true);
}


void MainWindow::point_nav_e12()
{
    double wp2[6] = {};
    wp2[0] = 8.012236*M_PI/180;
    wp2[1] = 27.000841*M_PI/180;
    wp2[2] = 79.997949*M_PI/180;
    wp2[3] = 34.999416*M_PI/180;
    wp2[4] = 90.0*M_PI/180;
    wp2[5] = 0.0*M_PI/180;
    robotService.robotServiceJointMove(wp2,true);
}
void MainWindow::point_nav_e13()
{
    double wp2[6] = {};
    wp2[0] = -1.06*M_PI/180;
    wp2[1] = 45.0*M_PI/180;
    wp2[2] = 80.0*M_PI/180;
    wp2[3] = 40.0*M_PI/180;
    wp2[4] = 90.0*M_PI/180;
    wp2[5] = 0.0*M_PI/180;
    robotService.robotServiceJointMove(wp2,true);
}
//无点
void MainWindow::point_nav_e14()
{
    double wp2[6] = {};
    wp2[0] = -1.06*M_PI/180;
    wp2[1] = 45.0*M_PI/180;
    wp2[2] = 80.0*M_PI/180;
    wp2[3] = 40.0*M_PI/180;
    wp2[4] = 90.0*M_PI/180;
    wp2[5] = 0.0*M_PI/180;
    robotService.robotServiceJointMove(wp2,true);
}

void MainWindow::point_nav_e15()
{
    double wp2[6] = {};
    wp2[0] = -70.047*M_PI/180;
    wp2[1] = 3.21*M_PI/180;
    wp2[2] = 55.016*M_PI/180;
    wp2[3] = 35.0*M_PI/180;
    wp2[4] = 90.0*M_PI/180;
    wp2[5] = 0.0*M_PI/180;
    robotService.robotServiceJointMove(wp2,true);
}

void MainWindow::point_nav_e16()
{
    double wp2[6] = {};
    wp2[0] = -56.038*M_PI/180;
    wp2[1] = 3.211*M_PI/180;
    wp2[2] = 43.018*M_PI/180;
    wp2[3] = 35.0*M_PI/180;
    wp2[4] = 90.0*M_PI/180;
    wp2[5] = 0.0*M_PI/180;
    robotService.robotServiceJointMove(wp2,true);
}

//无点
void MainWindow::point_nav_e17()
{
    double wp2[6] = {};
    wp2[0] = -56.038*M_PI/180;
    wp2[1] = 3.211*M_PI/180;
    wp2[2] = 43.018*M_PI/180;
    wp2[3] = 35.0*M_PI/180;
    wp2[4] = 90.0*M_PI/180;
    wp2[5] = 0.0*M_PI/180;
    robotService.robotServiceJointMove(wp2,true);
}

//无点
void MainWindow::point_nav_e18()
{
    double wp2[6] = {};
    wp2[0] = -56.038*M_PI/180;
    wp2[1] = 3.211*M_PI/180;
    wp2[2] = 43.018*M_PI/180;
    wp2[3] = 35.0*M_PI/180;
    wp2[4] = 90.0*M_PI/180;
    wp2[5] = 0.0*M_PI/180;
    robotService.robotServiceJointMove(wp2,true);
}

void MainWindow::on_test_point2_clicked()
{
    point_nav3();
}

void MainWindow::on_point1_clicked()
{
    //测试点11
    QTimer::singleShot(1000, this, [this]() {
            point_nav_e11_1();
        });

    QTimer::singleShot(6000, this, [this]() {
            point_nav_e11();
        });


    QTimer::singleShot(25000, this, [this]() {
            point_nav_e11_1();
        });

    QTimer::singleShot(30000, this, [this]() {
        point_move_b8();//机械臂复位
    });
}

void MainWindow::navStatusCallback(const std_msgs::String::ConstPtr& msg)
{
    // 使用QMetaObject::invokeMethod确保UI更新在主线程执行
    QMetaObject::invokeMethod(this, [this, msg]() {
        QString statusText = QString("📡 %1").arg(QString::fromStdString(msg->data));
        ui->robotstatus->append(statusText);
    }, Qt::QueuedConnection);
}

void MainWindow::on_startnavigation_clicked()
{
    startRosProcess("navigation");
    // 启动导航状态定时器，每1秒更新一次
    m_navStatusTimer->start(1000);
    ui->robotstatus->append("🚀 导航已启动，开始实时状态监控...");
    // 新增：启动robotstatus内容检查定时器
    m_robotStatusCheckTimer->start(1000);
    m_taskmove2 = false;
    m_tasknav3 = false;

    m_taskmove_b4 = false;
    m_taskmove_b5 = false;
    m_taskmove_b8 = false;
    m_taskmove_b11 = false;
    m_taskmove_b17 = false;
    m_taskmove_b18 = false;
    m_taskmove_b19 = false;

    m_taskmove_c1 = false;
    m_taskmove_c2 = false;

    m_taskmove_d1 = false;
    m_taskmove_d2 = false;
    m_taskmove_d3 = false;
    m_taskmove_d4 = false;
    m_taskmove_d5 = false;
    m_taskmove_d6 = false;
    m_taskmove_d7 = false;
    m_taskmove_d8 = false;
    m_taskmove_d9 = false;

    m_taskmove_e1 = false;
    m_taskmove_e2 = false;
    m_taskmove_e3 = false;
    m_taskmove_e4 = false;
    m_taskmove_e4_1 = false;

    m_taskmove_e5 = false;
    m_taskmove_e6 = false;
    m_taskmove_e7 = false;
    m_taskmove_e8 = false;
    m_taskmove_e9 = false;
    m_taskmove_e10 = false;
    m_taskmove_e11 = false;
    m_taskmove_e12 = false;
    m_taskmove_e13 = false;
    m_taskmove_e14 = false;
    m_taskmove_e15 = false;
    m_taskmove_e16 = false;
    m_taskmove_e17 = false;
    m_taskmove_e18 = false;

}

void MainWindow::checkRobotStatus()
{
    QString statusText = ui->robotstatus->toPlainText();
    //20251229黄埔实验室-全局-a(机械臂动作代号)
    /*if (statusText.contains("移动任务2完成") && !m_taskmove2) {
        m_taskmove2 = true;
        ui->robotstatus->append("⏳ 检测到移动任务2完成，等待1秒后执行机械臂任务...");
        QTimer::singleShot(1000, this, [this]() {
            point_move2();
            ui->robotstatus->append("🤖 等待结束，开始执行机械臂任务...");
        });
    }

    if (statusText.contains("导航到点2成功") && !m_tasknav3) {
        m_tasknav3 = true;
        ui->robotstatus->append("⏳ 检测到导航到点2，等待1秒后执行机械臂任务...");
        QTimer::singleShot(1000, this, [this]() {
            point_nav3();
            ui->robotstatus->append("🤖 等待结束，开始执行机械臂任务...");
        });
    }*/

    //20260122新路线，实验室A-连接门起点-b(机械臂动作代号),任务4为第一个测试点
//    if (statusText.contains("移动任务7完成") && !m_taskmove_b4) {
//            m_taskmove_b4 = true;
//            ui->robotstatus->append("⏳ 检测到移动任务7完成，等待1秒后执行机械臂任务...");
//            QTimer::singleShot(1000, this, [this]() {
//                point_move_b4();
//            });

//            //机械臂复位
//            QTimer::singleShot(30000, this, [this]() {
//                point_move_b8();
//            });
//        }

        //检测完成后机械臂复位
//        if (statusText.contains("移动任务8完成") && !m_taskmove_b8) {
//                m_taskmove_b8 = true;
//                ui->robotstatus->append("⏳ 检测到移动任务8完成，等待1秒后执行机械臂任务...");
//                QTimer::singleShot(1000, this, [this]() {
//                    point_move_b8_1();
//                });

//                QTimer::singleShot(25000, this, [this]() {
//                    point_move_b8();
//                });
//            }

//        if (statusText.contains("移动任务12完成") && !m_taskmove_b11) {
//                m_taskmove_b11 = true;
//                ui->robotstatus->append("⏳ 检测到移动任务12完成，等待1秒后执行机械臂任务...");
//                QTimer::singleShot(1000, this, [this]() {
//                    point_move_b12();
//                });

//                //等待10s检测点1，完成检测任务后，机械臂检测点2
//                QTimer::singleShot(1000, this, [this]() {
//                    point_move_b12_1();
//                });

//                //等待10s检测点2，完成检测任务后，机械臂检测点2
//                QTimer::singleShot(1000, this, [this]() {
//                    point_move_b12_2();
//                });

//                //等待60s完成检测任务后，机械臂复位
//                QTimer::singleShot(25000, this, [this]() {
//                    point_move_b8();
//                });
//            }

        //桌子旁边
//        if (statusText.contains("移动任务17完成") && !m_taskmove_b17) {
//                m_taskmove_b17 = true;
//                ui->robotstatus->append("⏳ 检测到移动任务17完成，等待1秒后执行机械臂任务...");
//                QTimer::singleShot(1000, this, [this]() {
//                    point_move_b17();
//                });
//                //等待10s完成检测任务后，机械臂复位
//                QTimer::singleShot(25000, this, [this]() {
//                    point_move_b8();
//                });
//            }

        //墙壁
//        if (statusText.contains("移动任务18完成") && !m_taskmove_b18) {
//                m_taskmove_b18 = true;
//                ui->robotstatus->append("⏳ 检测到移动任务18完成，等待1秒后执行机械臂任务...");
//                //第9检测点，中间
//                QTimer::singleShot(1000, this, [this]() {
//                    point_move_b18_1();
//                });

//                //第10检测点,右下角
//                QTimer::singleShot(1000, this, [this]() {
//                    point_move_b18_2();
//                });

                //第11检测点，左上角
//                QTimer::singleShot(1000, this, [this]() {
//                    point_move_b18_3();
//                });
//            }


//        //墙壁出来
//        if (statusText.contains("移动任务19完成") && !m_taskmove_b19) {
//                m_taskmove_b19 = true;
//                ui->robotstatus->append("⏳ 检测到移动任务19完成，等待1秒后执行机械臂任务..." );
//                QTimer::singleShot(1000, this, [this]() {
//                    point_move_b8();//机械臂复位
//                });
//            }

        //20260130黄埔实验室-房间B-c(机械臂动作代号)
        //储氢装置
//         if (statusText.contains("导航到点1成功") && !m_taskmove_c2) {
//            m_taskmove_c2 = true;
//            ui->robotstatus->append("⏳ 检测到导航到点1成功，等待1秒后执行机械臂任务..." );

//            //测试点12
//            QTimer::singleShot(1000, this, [this]() {
//                    point_move_c2();
//                });


//            QTimer::singleShot(25000, this, [this]() {
//                point_move_b8();//机械臂复位
//            });
//        }



//           //气柜
//        if (statusText.contains("移动任务2完成") && !m_taskmove_c1) {
//                m_taskmove_c1 = true;
//                ui->robotstatus->append("⏳ 检测到移动任务2完成，等待1秒后执行机械臂任务..." );

//                //测试点13
//                QTimer::singleShot(1000, this, [this]() {
//                    point_move_c1_1();
//                });

//                //测试点14
//                QTimer::singleShot(1000, this, [this]() {
//                    point_move_c1_3();
//                });

//                QTimer::singleShot(28000, this, [this]() {
//                    point_move_b8();//机械臂复位
//                });
//            }

         //20260203-小虎岛2期-d
         //RSOC装置-高温电堆模组区
//         if (statusText.contains("导航到点1成功") && !m_taskmove_d1) {
//            m_taskmove_d1 = true;
//            ui->robotstatus->append("⏳ 检测到导航到点1成功，等待1秒后执行机械臂任务..." );

//            //测试点1
//            QTimer::singleShot(1000, this, [this]() {
//                    point_nav_d1();
//                });

//            QTimer::singleShot(23000, this, [this]() {
//                point_move_b8();//机械臂复位
//            });
//        }

//          //加热增压区
//          if (statusText.contains("导航到点6成功") && !m_taskmove_d2) {
//             m_taskmove_d2 = true;
//             ui->robotstatus->append("⏳ 检测到导航到点6成功，等待1秒后执行机械臂任务..." );

//             //测试点2
//             QTimer::singleShot(1000, this, [this]() {
//                     point_nav_d2();
//                 });

//             QTimer::singleShot(23000, this, [this]() {
//                 point_move_b8();//机械臂复位
//             });
//         }

//          //RSOC靠近加热增压区
//          if (statusText.contains("移动任务7完成") && !m_taskmove_d3) {
//             m_taskmove_d3 = true;
//             ui->robotstatus->append("⏳ 检测到移动任务7完成，等待1秒后执行机械臂任务..." );

//             //测试点3
//             QTimer::singleShot(1000, this, [this]() {
//                     point_nav_d3();
//                 });

//             QTimer::singleShot(23000, this, [this]() {
//                 point_move_b8();//机械臂复位
//             });
//         }

//          //加热增压区
//          if (statusText.contains("导航到点12成功") && !m_taskmove_d4) {
//             m_taskmove_d4 = true;
//             ui->robotstatus->append("⏳ 检测到导航到点12，等待1秒后执行机械臂任务..." );

//             //测试点4
//             QTimer::singleShot(1000, this, [this]() {
//                     point_nav_d4_1();
//                 });

//             QTimer::singleShot(4000, this, [this]() {
//                     point_nav_d4_2();
//                 });

//             QTimer::singleShot(28000, this, [this]() {
//                     point_nav_d4_1();
//                 });

//             QTimer::singleShot(32000, this, [this]() {
//                 point_move_b8();//机械臂复位
//             });
//         }

//          //纯化储存区
//          if (statusText.contains("移动任务16完成") && !m_taskmove_d5) {
//             m_taskmove_d5 = true;
//             ui->robotstatus->append("⏳ 检测到导航到点12，等待1秒后执行机械臂任务..." );

//             //测试点5,摆头动作
//             QTimer::singleShot(1000, this, [this]() {
//                     point_nav_d5();
//                 });

//             QTimer::singleShot(4000, this, [this]() {
//                     point_nav_d5_1();
//                 });

//             QTimer::singleShot(7000, this, [this]() {
//                     point_nav_d5_2();
//                 });

//             QTimer::singleShot(10000, this, [this]() {
//                     point_nav_d5();
//                 });

//             QTimer::singleShot(13000, this, [this]() {
//                     point_nav_d5_1();
//                 });

//             QTimer::singleShot(16000, this, [this]() {
//                     point_nav_d5_2();
//                 });

//             QTimer::singleShot(19000, this, [this]() {
//                     point_nav_d5();
//                 });


//             //测试点6
//             QTimer::singleShot(40000, this, [this]() {
//                     point_nav_d6();
//                 });

//             QTimer::singleShot(43000, this, [this]() {
//                     point_nav_d6_1();
//                 });

//             QTimer::singleShot(46000, this, [this]() {
//                     point_nav_d6_2();
//                 });

//             QTimer::singleShot(49000, this, [this]() {
//                     point_nav_d6();
//                 });

//             QTimer::singleShot(52000, this, [this]() {
//                     point_nav_d6_1();
//                 });

//             QTimer::singleShot(55000, this, [this]() {
//                     point_nav_d6_2();
//                 });

//             QTimer::singleShot(58000, this, [this]() {
//                     point_nav_d6();
//                 });

//             //测试点7
//             QTimer::singleShot(85000, this, [this]() {
//                 point_nav_d7();//机械臂复位
//             });


//             QTimer::singleShot(110000, this, [this]() {
//                 point_move_b8();//机械臂复位
//             });

//         }

//          //储存罐
//          if (statusText.contains("导航到点26成功") && !m_taskmove_d8) {
//             m_taskmove_d8 = true;
//             ui->robotstatus->append("⏳ 检测到导航到点26成功，等待1秒后执行机械臂任务..." );

//             //测试点8
//             QTimer::singleShot(1000, this, [this]() {
//                     point_nav_d8();
//                 });

//             QTimer::singleShot(23000, this, [this]() {
//                 point_move_b8();//机械臂复位
//             });
//         }

//          //储存罐
//          if (statusText.contains("导航到点29成功") && !m_taskmove_d9) {
//             m_taskmove_d9 = true;
//             ui->robotstatus->append("⏳ 检测到导航到点29，等待1秒后执行机械臂任务..." );

//             //测试点9
//             QTimer::singleShot(1000, this, [this]() {
//                     point_nav_d9();
//                 });

//             QTimer::singleShot(23000, this, [this]() {
//                 point_move_b8();//机械臂复位
//             });
//         }


    //小虎岛1期，机械臂代号e


    //测试点1，加氢枪，氢气
    if (statusText.contains("导航到点1成功") && !m_taskmove_e1) {
       m_taskmove_e1 = true;
       ui->robotstatus->append("⏳ 检测到导航到点1成功，等待1秒后执行机械臂任务..." );

       //测试点1
       QTimer::singleShot(1000, this, [this]() {
               point_nav_e1();
           });

       QTimer::singleShot(27000, this, [this]() {
           point_move_b8();//机械臂复位
       });
   }

    //测试点2，加氢枪背面，氢气
    if (statusText.contains("移动任务5完成") && !m_taskmove_e2) {
       m_taskmove_e2 = true;
       ui->robotstatus->append("⏳ 检测到移动任务5完成，等待1秒后执行机械臂任务..." );

       //测试点2
       QTimer::singleShot(1000, this, [this]() {
               point_nav_e2();
           });

       QTimer::singleShot(27000, this, [this]() {
           point_move_b8();//机械臂复位
       });
   }

    //测试点3，事字对面
    if (statusText.contains("移动任务11完成") && !m_taskmove_e3) {
       m_taskmove_e3 = true;
       ui->robotstatus->append("⏳ 检测到移动任务11完成，等待1秒后执行机械臂任务..." );

       //测试点3
       QTimer::singleShot(1000, this, [this]() {
               point_nav_e3();
           });

       QTimer::singleShot(27000, this, [this]() {
               point_nav_e3_1();
           });

   }

    //测试点4，1期故字对面
    if (statusText.contains("移动任务12完成") && !m_taskmove_e4) {
       m_taskmove_e4 = true;
       ui->robotstatus->append("⏳ 检测到移动任务12完成，等待1秒后执行机械臂任务..." );

       QTimer::singleShot(1000, this, [this]() {
               point_nav_e4();
           });

       QTimer::singleShot(20000, this, [this]() {
               point_nav_e5_1();//机械臂抬起来
           });
   }

    //测试点4，1期故字对面,机械臂复位
    if (statusText.contains("移动任务13完成") && !m_taskmove_e4_1) {
       m_taskmove_e4_1 = true;
       ui->robotstatus->append("⏳ 检测到移动任务13完成，等待1秒后执行机械臂任务..." );

       QTimer::singleShot(1000, this, [this]() {
               point_move_b8();//机械臂复位
           });
   }

    //测试点5，1期都字对面
    if (statusText.contains("移动任务15完成") && !m_taskmove_e5) {
       m_taskmove_e5 = true;
       ui->robotstatus->append("⏳ 检测到移动任务15完成，等待1秒后执行机械臂任务..." );

       QTimer::singleShot(1000, this, [this]() {
               point_nav_e5();
           });

       QTimer::singleShot(27000, this, [this]() {
               point_nav_e5_1();//机械臂复位
           });
   }

    //1期可字对面，检测点6
    if (statusText.contains("移动任务16完成") && !m_taskmove_e6) {
       m_taskmove_e6 = true;
       ui->robotstatus->append("⏳ 检测到移动任务16完成，等待1秒后执行机械臂任务..." );

       QTimer::singleShot(1000, this, [this]() {
               point_nav_e6();
           });

       QTimer::singleShot(27000, this, [this]() {
               point_move_b8();//机械臂复位
           });
   }

    //1期卸氢区，检测点7、8、9
    if (statusText.contains("移动任务17完成") && !m_taskmove_e7) {
       m_taskmove_e7 = true;
       ui->robotstatus->append("⏳ 检测到移动任务17完成，等待1秒后执行机械臂任务..." );

       QTimer::singleShot(1000, this, [this]() {
               point_nav_e7();
           });

       QTimer::singleShot(30000, this, [this]() {
               point_nav_e8();
           });

       QTimer::singleShot(33000, this, [this]() {
               point_nav_e8_1();
           });

       QTimer::singleShot(63000, this, [this]() {
               point_nav_e9();
           });

       QTimer::singleShot(90000, this, [this]() {
               point_move_b8();//机械臂复位
           });
   }

    //1期卸氢区，检测点10
    if (statusText.contains("移动任务20完成") && !m_taskmove_e10) {
       m_taskmove_e10 = true;
       ui->robotstatus->append("⏳ 检测到移动任务20完成，等待1秒后执行机械臂任务..." );

       QTimer::singleShot(1000, this, [this]() {
               point_nav_e10();
           });

       QTimer::singleShot(27000, this, [this]() {
               point_move_b8();//机械臂复位
           });
   }


              //1期与2期交界处，测试点11，氢气
              if (statusText.contains("导航到点25成功") && !m_taskmove_e11) {
                 m_taskmove_e11 = true;
                 ui->robotstatus->append("⏳ 检测到导航到点25，等待1秒后执行机械臂任务..." );

                 //测试点11
                 QTimer::singleShot(1000, this, [this]() {
                         point_nav_e11_1();
                     });

                 QTimer::singleShot(6000, this, [this]() {
                         point_nav_e11();
                     });


                 QTimer::singleShot(25000, this, [this]() {
                         point_nav_e11_1();
                     });

                 QTimer::singleShot(30000, this, [this]() {
                     point_move_b8();//机械臂复位
                 });
             }

              //1期与2期交界处，测试点12，氢气
              if (statusText.contains("移动任务26完成") && !m_taskmove_e12) {
                 m_taskmove_e12 = true;
                 ui->robotstatus->append("⏳ 检测到移动任务26完成，等待1秒后执行机械臂任务..." );

                 //测试点12,停留35s
                 QTimer::singleShot(1000, this, [this]() {
                         point_nav_e12();
                     });

                 QTimer::singleShot(23000, this, [this]() {
                     point_move_b8();//机械臂复位
                 });
             }


              //1期后面正中间，测试点13，火焰
              if (statusText.contains("移动任务33完成") && !m_taskmove_e13) {
                 m_taskmove_e13 = true;
                 ui->robotstatus->append("⏳ 检测到移动任务33完成，等待1秒后执行机械臂任务..." );

                 //测试点13,停留35s
                 QTimer::singleShot(1000, this, [this]() {
                         point_nav_e13();
                     });

                 QTimer::singleShot(23000, this, [this]() {
                     point_move_b8();//机械臂复位
                 });
             }

              //1期后面，测试点14，火焰
              if (statusText.contains("移动任务34完成") && !m_taskmove_e14) {
                 m_taskmove_e14 = true;
                 ui->robotstatus->append("⏳ 检测到移动任务34完成，等待1秒后执行机械臂任务..." );

                 //测试点14,停留35s
                 QTimer::singleShot(1000, this, [this]() {
                         point_nav_e14();
                     });

                 QTimer::singleShot(23000, this, [this]() {
                     point_move_b8();//机械臂复位
                 });
             }

              //储存室旁边，测试点15，氢气，16、17、18
              if (statusText.contains("移动任务40完成") && !m_taskmove_e15) {
                 m_taskmove_e15 = true;
                 ui->robotstatus->append("⏳ 检测到移动任务34完成，等待1秒后执行机械臂任务..." );

                 //测试点14,停留35s
                 QTimer::singleShot(1000, this, [this]() {
                         point_nav_e15();
                     });

                 QTimer::singleShot(23000, this, [this]() {
                     point_nav_e16();//测试点16
                 });

                 QTimer::singleShot(45000, this, [this]() {
                     point_move_b8();//机械臂复位
                 });
             }

}




void MainWindow::on_pushButton_clicked()
{
    point_move_b12_1();
    point_move_b12_2();
}


void MainWindow::on_pos_test_clicked()
{
    double wpc3[6] = {};
    wpc3[0] = 53.657225*M_PI/180;
    wpc3[1] = -2.061064*M_PI/180;
    wpc3[2] = 153.193766*M_PI/180;
    wpc3[3] = -1.542173*M_PI/180;
    wpc3[4] = 174.501102*M_PI/180;
    wpc3[5] = 10.276528*M_PI/180;
    robotService.robotServiceJointMove(wpc3,true);
}


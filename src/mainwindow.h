#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include <QLabel>
#include <QPixmap>
#include"LED.h"

//AGV
#include "zcan.h"  // CAN 库头文件
#include <QMainWindow>
#include <QMessageBox>
#include <QTimer>
#include <QVector>
#include <QSerialPort>
#include <QSerialPortInfo>
#include<QProgressBar>
#include <QThread>
#include <QProgressDialog>

//aubo_robot
#include"AuboRobotMetaType.h"
#include"serviceinterface.h"

//ROS
#include <QProcess>
#include <QMap>
#include <QString>
#include <QMessageBox>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <thread>
#include <mutex>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/Odometry.h> 
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
//zed
#include <sl/Camera.hpp>
#include <opencv2/opencv.hpp>
#include <QWaitCondition>
#include <QMutex>

class MainWindow;// 前置声明MainWindow


// 检测参数结构体（从shipinliu.cpp复制）
struct DetectionParams {
    int h_min, h_max;
    int s_min;
    int v_min;
    int glare_v_min;
    int glare_s_max;
};
extern DetectionParams detectionParams;

// 视频处理线程类
class VideoProcessingThread : public QThread
{
    Q_OBJECT
public:
    explicit VideoProcessingThread(QObject *parent = nullptr);
    ~VideoProcessingThread();
    
    void startProcessing();
    void stopProcessing();
    void setDetectionEnabled(bool enabled);
    void setDetectionParams(const DetectionParams& params);
    void setCamera(sl::Camera* camera) { m_camera = camera; }
    
signals:
    void frameProcessed(const QImage& image, int detectedCount);
    void errorOccurred(const QString& error);
    
protected:
    void run() override;
    
private:
    bool m_running;
    bool m_detectionEnabled;
    DetectionParams m_detectionParams;
    QMutex m_mutex;
    QWaitCondition m_condition;
    sl::Camera* m_camera;
    
    cv::Mat slMat2cvMat(sl::Mat& input);
    std::vector<cv::Rect> detectRects(const cv::Mat& img, cv::Mat& vis_colorMask, cv::Mat& vis_glare, cv::Mat& vis_mask, cv::Mat& vis_morph);
};


QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void setCurrentWayPoint(const aubo_robot_namespace::wayPoint_S &wayPoint);//位置
    void setCurrentRpy(const aubo_robot_namespace::Rpy &rpy);//姿态
    void setJointAngles(const double angles[6]);//关节角度

signals:
    void batteryLevelUpdated(int level);// 电量更新信号

private slots:
    //传感器
    void readH2SerialData();         // 串口数据接收槽函数
    void on_H2display_clicked();
    void on_Flamedisplay_clicked();
    void readFlameSerialData();
    void on_jointmove_clicked();
    void H2write();
    void Flamewrite();

    //AGV
    void on_connectButton_clicked();
    void on_initializeButton_clicked();
    void on_closeButton_clicked();
    void sendMovementCommand(const U8 data[8]);//发送小车运动数据函数
    void sendForward();
    void sendLeft();
    void sendBack();
    void sendRight();
    void stopMove();
    void onAutoSequenceTimeout();//索引小车下一个动作
    void on_forwardButton_clicked();
    void on_leftButton_clicked();
    void on_backButton_clicked();
    void on_rightButton_clicked();
    void updateBatteryLevel(int level);

    //aubo_robot
    void on_login_clicked();
    void on_poweron_clicked();
    void updatePowerOnProgress();//上电进度条
    void on_getpos_clicked();
    void updatePositionDisplay();
    void on_powerout_clicked();
    void setjointacc();
    void on_Xplus_clicked();    //运动控制
    void on_Xminus_clicked();
    void on_Yplus_clicked();
    void on_Yminus_clicked();
    void on_Zplus_clicked();
    void on_Zminus_clicked();
    void on_Rxplus_clicked();
    void on_Rxminus_clicked();
    void on_Ryplus_clicked();
    void on_Ryminus_clicked();
    void on_Rzplus_clicked();
    void on_Rzminus_clicked();
    void aubo_speed(int value);
    void on_J1_clicked();
    void on_J1_2_clicked();
    void on_J2_clicked();
    void on_J2_2_clicked();
    void on_J3_clicked();
    void on_J3_2_clicked();
    void on_J4_clicked();
    void on_J4_2_clicked();
    void on_J5_clicked();
    void on_J5_2_clicked();
    void on_J6_clicked();
    void on_J6_2_clicked();
    void on_Initial_pos_clicked();
    void on_auto_move_clicked();
    void on_pathplan_clicked();

    //ROS
    void on_startRadarButton_clicked();
    void on_stoptRadarButton_clicked();
    void on_startlocalization_clicked();
    void on_stoplocalization_clicked();
    void on_pub_initpos_clicked();
    void on_startmovebase_clicked();
    void on_stopmovebase_clicked();
    void on_startagv_clicked();
    void on_stopagv_clicked();
    void on_startnavigation_clicked();
    void on_stopnavigation_clicked();
    void on_stopall_clicked();

    //相机
    void on_start_camera_btn_clicked();
    void on_stop_camera_btn_clicked();
    void on_save_image_btn_clicked();
    void on_savepos_clicked();
    void onVideoFrameProcessed(const QImage& image, int detectedCount);
    void onVideoError(const QString& error);

    //扫描
    void on_X_scan_btn_clicked();
    void on_Y_scan_btn_clicked();
    void on_point1_clicked();
    void on_test_point2_clicked();


    void on_pushButton_clicked();

    void on_pos_test_clicked();

private:
    Ui::MainWindow *ui;

    //AGV
    QTimer *m_forwardTimer;  // 直行定时器
    QTimer *m_backTimer;     // 后退定时器
    QTimer *m_leftTimer;     // 左转定时器
    QTimer *m_rightTimer;    // 右转定时器
    QTimer *m_stopTimer;     // 停止所有动作定时器
    QTimer *m_autoSequenceTimer; // 自动序列定时器
    QTimer *m_navStatusTimer;    // 导航状态定时器

    // 新增：robotstatus内容检查定时器
    QTimer *m_robotStatusCheckTimer;
    void checkRobotStatus();     // 检查robotstatus内容

    bool m_isforwardSending = false; // 发送状态标记
    bool m_isbackSending = false;
    bool m_isleftSending = false;
    bool m_isrightSending = false;
    bool m_isautoSending = false;

    int m_currentActionIndex = 0; // 当前动作索引
    QVector<QPair<void (MainWindow::*)(), int>> m_autoActionSequence; // 自动动作序列
    void initAutoActionSequence(); //小车自动巡检动作列表

    //aubo_robot
    QTimer *m_posTimer;
    aubo_robot_namespace::wayPoint_S currentWayPoint; // 保存当前位姿数据
    aubo_robot_namespace::Rpy currentRpy;//欧拉角
    double currentJointAngles[6];//关节角
    double targetJointAngles[6];//关节角
    static void RealTimeEndSpeedCallback(double speed, void *arg);
    QProgressDialog *powerOnProgress;// 上电进度条弹窗
    QTimer *progressTimer;// 进度更新定时器
    int currentProgress;// 当前进度值
    bool m_login = false;//登录状态
    bool m_power = false;//上电状态
    QTimer *m_powerCheckTimer;//上电状态检测
    void powerstarts();//上电状态
    void movetopoint(double x, double y, double z, double rx, double ry, double rz);


    //传感器
    QSerialPort *serial;          // 氢气传感器串口对象
    QTimer *H2Timer;
    QSerialPort *serial1;          // 火焰传感器串口对象
    QTimer *FlameTimer;
    LED led;
    
    // 定义ROS进程信息结构体
    struct RosProcessInfo {
        QProcess* process;
        QString name;
        QString command;
        QString workspace;
        QPushButton* startButton;
        QPushButton* stopButton;
    };
    
    // ROS进程管理映射
    QMap<QString, RosProcessInfo> m_rosProcesses;
    
    // 通用ROS进程管理函数
    void addRosProcess(const QString& name, const QString& command, const QString& workspace = "");  // 默认从环境变量 ROS_WORKSPACE 读取
    void startRosProcess(const QString& name);
    void stopRosProcess(const QString& name);
    bool isRosProcessRunning(const QString& name);
    
    // 通用进程信号处理函数
    void onRosProcessFinished(const QString& name, int exitCode, QProcess::ExitStatus exitStatus);
    void onRosProcessError(const QString& name, QProcess::ProcessError error);
    
    //ROS信息显示相关
    void appendRosInfo(const QString& message);  // 添加ROS信息到显示控件
    
    //ROS进程标准输出处理
    void onRosProcessOutput(const QString& name);
    void onRosProcessErrorOutput(const QString& name);

    // ROS订阅者相关
    std::unique_ptr<ros::NodeHandle> m_rosNode;
    ros::Subscriber m_agvPosSubscriber;
    ros::Subscriber m_odomSubscriber;
    std::thread m_rosThread;
    std::mutex m_posMutex;
    geometry_msgs::PoseStamped m_currentAgvPos;
    bool m_rosInitialized;
    ros::Subscriber m_nav_status_subscriber; // 导航状态订阅者
    void navStatusCallback(const std_msgs::String::ConstPtr& msg);

    // TF监听器相关
    std::unique_ptr<tf2_ros::Buffer> m_tfBuffer;
    std::unique_ptr<tf2_ros::TransformListener> m_tfListener;
    QTimer* m_tfUpdateTimer; // 用于定期更新TF变换的定时器
    void initRosNode();
    void initTfListener();
    void updateTfTransform();
    void startRoscore();

    void updateNavStatusDisplay();//导航状态更新

    // 新增相机相关成员变量
    QTimer *cameraTimer;
    QLabel *statusLabel;
    sl::Camera zedCamera;
    bool cameraRunning;
    int saveIndex;
    
    // 新增：相机初始化函数
    bool initZEDCamera();
    void saveImages();  // 保存图像函数
    
    bool detectionEnabled;  // 检测开关
    
    // 检测函数（从shipinliu.cpp复制）
    void updateDetectionFrame();
    // 视频处理线程相关
    VideoProcessingThread *m_videoThread;
    QLabel *m_displayLabel; 
    QLabel *detectionLabel;
    void updateCameraFrame();

    void point_move2();//巡检点1
    void point_nav3();//巡检点2
    void point_move_b4();
    void point_move_b5();
    void point_move_b8();
    void point_move_b8_1();
    void point_move_b12();
    void point_move_b12_1();
    void point_move_b12_2();
    void point_move_b17();
    void point_move_b18_1();
    void point_move_b18_2();
    void point_move_b18_3();

    void point_move_c1_1();
    void point_move_c1_2();
    void point_move_c1_3();
    void point_move_c2();
    void point_nav_d1();
    void point_nav_d2();
    void point_nav_d3();
    void point_nav_d4_1();
    void point_nav_d4_2();
    void point_nav_d5();//摆头
    void point_nav_d5_1();
    void point_nav_d5_2();

    void point_nav_d6();//摆头
    void point_nav_d6_1();
    void point_nav_d6_2();

    void point_nav_d7();
    void point_nav_d8();
    void point_nav_d9();


    void point_nav_e1();
    void point_nav_e2();
    void point_nav_e3();
    void point_nav_e3_1();

    void point_nav_e4();
    void point_nav_e5();
    void point_nav_e5_1();

    void point_nav_e6();
    void point_nav_e7();
    void point_nav_e8();
    void point_nav_e8_1();

    void point_nav_e9();
    void point_nav_e10();
    void point_nav_e11();
    void point_nav_e11_1();

    void point_nav_e12();
    void point_nav_e13();
    void point_nav_e14();
    void point_nav_e15();
    void point_nav_e16();
    void point_nav_e17();
    void point_nav_e18();




    //巡检任务标志
    bool m_taskmove2 = false;
    bool m_tasknav3 = false;

    bool m_taskmove_b4 = false;
    bool m_taskmove_b5 = false;
    bool m_taskmove_b8 = false;
    bool m_taskmove_b11 = false;
    bool m_taskmove_b17 = false;
    bool m_taskmove_b18 = false;
    bool m_taskmove_b19 = false;

    bool m_taskmove_c1 = false;
    bool m_taskmove_c2 = false;

    bool m_taskmove_d1 = false;
    bool m_taskmove_d2 = false;
    bool m_taskmove_d3 = false;
    bool m_taskmove_d4 = false;
    bool m_taskmove_d5 = false;
    bool m_taskmove_d6 = false;
    bool m_taskmove_d7 = false;
    bool m_taskmove_d8 = false;
    bool m_taskmove_d9 = false;

    bool m_taskmove_e1 = false;
    bool m_taskmove_e2 = false;
    bool m_taskmove_e3 = false;
    bool m_taskmove_e4 = false;
    bool m_taskmove_e4_1 = false;

    bool m_taskmove_e5 = false;
    bool m_taskmove_e6 = false;
    bool m_taskmove_e7 = false;
    bool m_taskmove_e8 = false;
    bool m_taskmove_e9 = false;
    bool m_taskmove_e10 = false;
    bool m_taskmove_e11 = false;
    bool m_taskmove_e12 = false;
    bool m_taskmove_e13 = false;
    bool m_taskmove_e14 = false;
    bool m_taskmove_e15 = false;
    bool m_taskmove_e16 = false;
    bool m_taskmove_e17 = false;
    bool m_taskmove_e18 = false;



};



#endif// MAINWINDOW_H

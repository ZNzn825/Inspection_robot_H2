#include "sensormonitor.h"
#include <QPainter>
#include <QDateTime>

SensorMonitor::SensorMonitor(QChartView *chartView, QObject *parent)
    : QObject(parent), m_chartView(chartView), m_maxPoints(60)
{
    // 1. 初始化图表核心组件
    m_chart = new QChart();
    m_series = new QLineSeries();
    m_axisX = new QValueAxis();
    m_axisY = new QValueAxis();

    // 深蓝色取值可自定义，示例为暗深蓝色（R:0, G:30, B:60），也可改用命名颜色如Qt::darkBlue
    //QColor bgColor(100, 149, 237);
    //m_chart->setBackgroundBrush(QBrush(bgColor)); // 设置背景画刷
    m_chart->setBackgroundRoundness(0); // 可选：取消背景圆角，适配界面风格

    // 2. 配置图表和坐标轴
    m_chart->addSeries(m_series);
    m_chart->addAxis(m_axisX, Qt::AlignBottom);  // X轴居下
    m_chart->addAxis(m_axisY, Qt::AlignLeft);    // Y轴居左
    m_series->attachAxis(m_axisX);
    m_series->attachAxis(m_axisY);

    // 3. X轴基础配置（关键：从0开始，固定显示60秒范围）
    m_axisX->setTitleText("时间/s");
    m_axisX->setRange(0, 60);  // 固定显示0-60秒，新数据向右推进
    m_axisX->setLabelFormat("%.1f"); // 时间显示保留1位小数

    // 4. 图表美化
    m_chartView->setChart(m_chart);
    m_chartView->setRenderHint(QPainter::Antialiasing); // 抗锯齿
    m_chart->legend()->hide(); // 隐藏图例（简化显示）

    // 5. 多线程配置（对象移到工作线程）
    moveToThread(&m_workerThread);
    connect(&m_workerThread, &QThread::finished, this, &QObject::deleteLater);
}

SensorMonitor::~SensorMonitor()
{
    stopMonitoring();
    // 释放资源（chartView管理m_chart，无需手动删）
    delete m_series;
    delete m_axisX;
    delete m_axisY;
}

void SensorMonitor::startMonitoring()
{
    if (!m_workerThread.isRunning()) {
        m_workerThread.start();
    }
    // 重置数据和基准时间（关键：启动时记录当前时间戳）
    m_values.clear();
    m_times.clear();
    m_series->clear();
    // 修复QDateTime编译错误：兼容所有Qt版本的时间戳写法
    m_startTimestamp = QDateTime::currentDateTime().toMSecsSinceEpoch();
}

void SensorMonitor::stopMonitoring()
{
    if (m_workerThread.isRunning()) {
        m_workerThread.quit();
        m_workerThread.wait();
    }
}

void SensorMonitor::setYRange(double min, double max)
{
    m_axisY->setRange(min, max);
}

void SensorMonitor::setTitle(const QString &title)
{
    m_chart->setTitle(title);
    m_axisY->setTitleText(title);
}

void SensorMonitor::addData(double value)
{
    // 计算相对时间（从监测启动到现在的秒数，关键：确保时间递增）
    qint64 currentTimestamp = QDateTime::currentDateTime().toMSecsSinceEpoch();
    double elapsedTime = (currentTimestamp - m_startTimestamp) / 1000.0; // 转秒

    // 缓存数据（时间+数值）
    m_times.append(elapsedTime);
    m_values.append(value);

    // 限制数据点数量（只保留最近60个）
    while (m_times.size() > m_maxPoints) {
        m_times.removeFirst();
        m_values.removeFirst();
    }

    // 跨线程触发图表更新（Qt::QueuedConnection确保主线程执行）
    QMetaObject::invokeMethod(this, "updateChart", Qt::QueuedConnection);
}

void SensorMonitor::updateChart()
{
    // 清空旧曲线，重新添加数据（确保顺序从左到右）
    m_series->clear();
    for (int i = 0; i < m_times.size(); ++i) {
        m_series->append(m_times[i], m_values[i]);
    }

    // 调整X轴可视范围（关键：始终显示最新60秒，新数据在右侧）
    if (!m_times.isEmpty()) {
        double latestTime = m_times.last();
        if (latestTime > 60) { // 超过60秒后，X轴跟随右移
            m_axisX->setRange(latestTime - 60, latestTime);
        }
        // 未超过60秒时，保持0-60秒范围，数据从左到右填充
    }
}

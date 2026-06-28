#ifndef SENSORMONITOR_H
#define SENSORMONITOR_H

#include <QObject>
#include <QtCharts>
#include <QThread>
#include <QVector>
#include <QDateTime>

QT_CHARTS_USE_NAMESPACE

class SensorMonitor : public QObject
{
    Q_OBJECT
public:
    explicit SensorMonitor(QChartView *chartView, QObject *parent = nullptr);
    ~SensorMonitor();

    void startMonitoring();  // 启动监测
    void stopMonitoring();   // 停止监测
    void setYRange(double min, double max); // 设置Y轴范围
    void setTitle(const QString &title);    // 设置图表标题

public slots:
    void addData(double value); // 添加传感器数据

private slots:
    void updateChart(); // 更新图表（主线程执行）

private:
    QChartView *m_chartView;    // 关联的图表视图
    QChart *m_chart;            // 图表核心对象
    QLineSeries *m_series;      // 曲线系列
    QValueAxis *m_axisX;        // X轴（时间，秒）
    QValueAxis *m_axisY;        // Y轴（传感器数值）

    QThread m_workerThread;     // 工作线程（避免阻塞UI）
    QVector<double> m_values;   // 数值缓存
    QVector<double> m_times;    // 时间缓存（从0开始递增）
    int m_maxPoints;            // 最大数据点（默认60个）
    qint64 m_startTimestamp;    // 监测启动的基准时间戳（毫秒）
};

#endif // SENSORMONITOR_H

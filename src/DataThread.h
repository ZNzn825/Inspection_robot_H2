#ifndef DATATHREAD_H
#define DATATHREAD_H

#include <QObject>
#include <QThread>
#include <QDateTime>

// 数据结构体：包含时间、氢气浓度、火焰大小
struct MonitorData {
    QDateTime time;  // 时间
    double h2Concentration;  // 氢气浓度 (0-100)
    double flameSize;  // 火焰大小 (4-20)
};

class DataThread : public QThread
{
    Q_OBJECT
public:
    DataThread();
    explicit DataThread(QObject *parent = nullptr) : QThread(parent)
    {
        m_running = true;
    }

    void stop()
    {
        m_running = false;
        wait();
    }

signals:
    // 发送新数据信号
    void newData(const MonitorData &data);

protected:
    void run() override
    {
        while (m_running)
        {
            // 模拟数据（实际项目中替换为传感器读取逻辑）
            MonitorData data;
            data.time = QDateTime::currentDateTime();
            // 氢气浓度：在30-70之间随机波动
            data.h2Concentration = 30 + qrand() % 40 + (qrand() % 100) / 100.0;
            // 火焰大小：在10-40之间随机波动
            data.flameSize = 10 + qrand() % 30 + (qrand() % 100) / 100.0;

            emit newData(data);  // 发送数据到主线程
            msleep(100);  // 100ms采集一次
        }
    }

private:
    bool m_running;


};

#endif // DATATHREAD_H

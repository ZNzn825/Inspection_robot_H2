#ifndef AUBO_ROBOT_H
#define AUBO_ROBOT_H

#include <QThread>
#include"AuboRobotMetaType.h"
#include"serviceinterface.h"

class aubo_robot : public QThread
{
    Q_OBJECT
public:
    explicit aubo_robot(QObject *parent = nullptr);

protected:

signals:

};

#endif // AUBO_ROBOT_H

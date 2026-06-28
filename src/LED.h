#ifndef LED_H
#define LED_H

#include <QObject>
#include <QWidget>
#include <QLabel>

class LED
{
public:
    LED();
    void setLED(QLabel* label, int color, int size);

private:

};

#endif // LED_H

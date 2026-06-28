#ifndef PARTICLEPROGRESS_H
#define PARTICLEPROGRESS_H

#include <QWidget>
#include <QPainter>
#include <QVector>
#include <QPropertyAnimation>

class particleprogress : public QWidget
{
     Q_OBJECT
public:
    ParticleProgress(QWidget *parent = nullptr);
    void setValue(int value);
    void setText(const QString &text);


protected:
    void paintEvent(QPaintEvent *event) override;


private:
    void initParticles();
    void updateParticles();


private:
    int m_value;
    QString m_text;
    QColor m_progressColor;
    QColor m_particleColor;
    QVector<QPointF> m_particles;
    QPropertyAnimation *m_particleAnim;
    int m_particleCount;
    int m_particleSpeed;
};

#endif // PARTICLEPROGRESS_H

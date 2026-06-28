// particleprogress.cpp
#include "particleprogress.h"
#include <QRandomGenerator>

ParticleProgress::ParticleProgress(QWidget *parent)
    : QWidget(parent), m_value(0), m_particleCount(100),
      m_particleSpeed(2), m_progressColor(Qt::blue),
      m_particleColor(Qt::cyan)
{
    // 设置样式
    setStyleSheet("background-color: transparent;");
    setFixedSize(400, 30);

    // 初始化粒子系统
    initParticles();

    // 粒子动画
    m_particleAnim = new QPropertyAnimation(this, "m_particleSpeed");
    m_particleAnim->setLoopCount(-1);
    m_particleAnim->setDuration(2000);
    m_particleAnim->setStartValue(1);
    m_particleAnim->setKeyValueAt(0.25, 1);
    m_particleAnim->setKeyValueAt(0.5, 3);
    m_particleAnim->setKeyValueAt(0.75, 5);
    m_particleAnim->setKeyValueAt(1.0, 3);
    m_particleAnim->setLoopCount(-1);
    connect(m_particleAnim, &QPropertyAnimation::valueChanged, this, [this]() {
        updateParticles();
    });
    m_particleAnim->start();
}

void ParticleProgress::setValue(int value)
{
    m_value = value;
    update();
}

void ParticleProgress::setText(const QString &text)
{
    m_text = text;
    update();
}

void ParticleProgress::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    // 绘制背景
    painter.fillRect(rect(), QColor(40, 40, 50, 200));

    // 绘制进度条
    QPainterPath path;
    path.addRoundedRect(rect(), 15, 15);

    QLinearGradient gradient(0, 0, width(), 0);
    gradient.setColorAt(0, QColor(50, 150, 255));
    gradient.setColorAt(1, QColor(0, 200, 255));

    painter.fillPath(path, gradient);

    // 绘制粒子
    painter.setBrush(m_particleColor);
    for (const QPointF &point : m_particles) {
        painter.drawEllipse(point, 2, 2);
    }

    // 绘制文本
    painter.setPen(Qt::white);
    QFont font;
    font.setBold(true);
    font.setPixelSize(12);
    painter.setFont(font);
    painter.drawText(rect(), Qt::AlignCenter, m_text);

    // 绘制完成百分比
    painter.drawText(rect().adjusted(0, 0, -20, 0), Qt::AlignRight, QString("%1%").arg(m_value));
}

void ParticleProgress::initParticles()
{
    m_particles.clear();

    qsrand(QTime::currentTime().msec());

    for (int i = 0; i < m_particleCount; ++i) {
        // 随机位置
        qreal posX = qrand() % (width() - 20) + 10;
        qreal posY = qrand() % height();

        // 随机大小
        qreal size = 1.0 + (qrand() % 3) / 2.0;

        m_particles.append(QPointF(posX, posY));
    }
}

void ParticleProgress::updateParticles()
{
    // 更新粒子位置
    for (int i = 0; i < m_particles.size(); ++i) {
        QPointF &point = m_particles[i];

        // X方向移动
        point.setX(point.x() + m_particleSpeed);

        // 边界处理
        if (point.x() < 0 || point.x() > width()) {
            // 反转方向
            m_particleSpeed = -m_particleSpeed;
            point.setX(point.x() - m_particleSpeed);
        }

        // Y方向轻微波动
        point.setY(point.y() + (qrand() % 3 - 1));

        // 如果粒子移出底部边界，重置到顶部
        if (point.y() > height()) {
            point.setY(0);
            point.setX(qrand() % (width() - 20) + 10);
        }
    }

    update();
}

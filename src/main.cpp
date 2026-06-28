#include "mainwindow.h"
#include <QApplication>
#include <QTextCodec>
int main(int argc, char *argv[])
{
    // 开启高DPI自动缩放（Qt 5.6+支持）
    QApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    // 开启高分屏控件渲染优化（可选，Qt 5.10+支持）
    QApplication::setAttribute(Qt::AA_UseHighDpiPixmaps);

    
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    QTextCodec::setCodecForLocale(QTextCodec::codecForName("UTF-8"));
    return a.exec();
}

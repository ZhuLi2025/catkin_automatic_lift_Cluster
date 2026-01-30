#ifndef QLABLEEX_H
#define QLABLEEX_H

#include <QObject>
#include <QLabel>
#include <QPainter>
#include <QPoint>
#include <QColor>
#include <QPen>
#include <QMouseEvent>
#include <QDebug>

namespace zl_common {
namespace app {

class QLabelEx:public QLabel
{
    Q_OBJECT
    public:
      QLabelEx();

    public:
     QLabelEx(QWidget *parent);
     //重写绘制事件
     virtual void paintEvent(QPaintEvent *event) override;
     //鼠标按下
     void mousePressEvent(QMouseEvent *e)override;
     //鼠标移动
     void mouseMoveEvent(QMouseEvent *e)override;
     //鼠标抬起
     void mouseReleaseEvent(QMouseEvent *e)override;
     //得到画框的起点和终点
     QPoint getStartPoint();
     QPoint getEndPoint();
     void setLineColor(const QColor lineColor);
     void setLineSize(const int lineSize);
     void clear();
     void PaintRect(QPoint start,QPoint stop);

    private:
     QPoint StartPoint;          //画线起点
     QPoint EndPoint;            //画线终点
     QColor lineColor;               //线条颜色
     int lineSize;                  //5种线型
     bool isPressed;
    signals:
     void StartPointSignal(QPoint p);  //开始坐标的信号
     void StopPointSignal(QPoint p);  //结束坐标的信号
     void ClickPointSignal(QPoint p);  //结束坐标的信号
    public slots:
     void eraseRectRectBox();
};
}
}
#endif // QLABLEEX_H

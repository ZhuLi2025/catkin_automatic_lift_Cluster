#include "QLabelEx.hpp"
namespace zl_common
{
namespace app
{
QLabelEx::QLabelEx(QWidget *parent):QLabel (parent)
{
    this->StartPoint = QPoint(0,0);
    this->EndPoint = QPoint(0,0);
    this->lineColor = QColor(Qt::red);
    this->lineSize = 3;
}
void QLabelEx::PaintRect(QPoint start,QPoint stop){
    //外部类绘制的函数
    StartPoint=start;
    EndPoint=stop;
    update();  //更新label显示 调用paintEvent事件
}

//绘制事件
void QLabelEx::paintEvent(QPaintEvent *event)
{
    QLabel::paintEvent(event);//必须有，才能让背景图片显示出来
    QPainter painter(this);
    QPen pen;
    pen.setColor(lineColor);
    pen.setWidth(lineSize);
    painter.setPen(pen);
    painter.drawRect(QRect(StartPoint.x(), StartPoint.y()
                           , EndPoint.x()-StartPoint.x(), EndPoint.y()-StartPoint.y()));
    painter.drawPoint(StartPoint.x(),StartPoint.y());
}
//鼠标按下
void QLabelEx::mousePressEvent(QMouseEvent *e)
{
//    StartPoint = e->pos();
//    EndPoint = e->pos();
    StartPoint = this->mapFromGlobal(e->globalPos());
    EndPoint = this->mapFromGlobal(e->globalPos());
    //设置按下flag为true
    isPressed = true;
    //发送开始位置坐标信号给主窗体
    emit StartPointSignal(StartPoint);
    emit ClickPointSignal(StartPoint);
    //    qDebug()<<"开始"<<StartPoint;
}

//鼠标移动
void QLabelEx::mouseMoveEvent(QMouseEvent *e)
{
    if(isPressed)
    {
        //获得结束点的坐标
//        EndPoint=e->pos();
          EndPoint = this->mapFromGlobal(e->globalPos());
        //发送结束位置坐标信号给主窗体
        //emit StopPointSignal(EndPoint);
        update();  //更新label显示 调用paintEvent事件
        //        qDebug()<<"按下移动"<<EndPoint;
    }

}

//鼠标抬起
void QLabelEx::mouseReleaseEvent(QMouseEvent *e)
{
    isPressed=false;
    update();
    emit StopPointSignal(EndPoint);
}

void QLabelEx::eraseRectRectBox()
{
    StartPoint = QPoint(0,0);
    EndPoint = QPoint(0,0);
    update();
}

void QLabelEx::setLineColor(const QColor lineColor)
{
    this->lineColor = lineColor;
}

void QLabelEx::setLineSize(const int lineSize)
{
    this->lineSize = lineSize;
}

QPoint QLabelEx::getStartPoint()
{
    return StartPoint;
}

QPoint QLabelEx::getEndPoint()
{
    return EndPoint;
}

void QLabelEx::clear()
{
    StartPoint = QPoint(0,0);
    EndPoint = QPoint(0,0);
    update();
}
}
}

#include "QTimerThread.hpp"
#include <QDebug>
QTimerThread::QTimerThread(int iCount)
{
    m_iThreadCount = iCount;
}

QTimerThread::~QTimerThread()
{
  //对象的销毁顺序也很重要
    for(int i = 0; i < m_iThreadCount; i++)
    {
        m_threadList.value(i)->quit();
        m_threadList.value(i)->wait();
        m_qTimerList.value(i)->deleteLater();
        m_threadList.value(i)->deleteLater();
    }
}

void QTimerThread::createItem()
{
    for(int i = 0;i < m_iThreadCount;i++)
    {
        QTimer *timer = new QTimer();
        QThread  *thread = new QThread();
        m_qTimerList.append(timer);
        m_threadList.append(thread);
    }
}

void QTimerThread::startMultThread()
{
    for(int i = 0; i < m_qTimerList.size(); i++)
    {
      //划重点的部分
      /*注：好像还有其他方式可以实现比如把定时器的开启放在connect里面，
      感兴趣的小伙伴可以试一试换换顺序这些的是什么效果，
      我当时好像都试了一遍，
      但是可能是这种方法比较容易理解且不出错才采用了，
      实践出真知。*/
        m_qTimerList.value(i)->start(5000);
        m_qTimerList.value(i)->moveToThread(m_threadList.value(i));
        QObject::connect(m_qTimerList.value(i),SIGNAL(timeout()),this,SLOT(update()),Qt::QueuedConnection);
        m_threadList.value(i)->start();
    }
}

void QTimerThread::update()
{
    //这里放需要耗时的操作
}

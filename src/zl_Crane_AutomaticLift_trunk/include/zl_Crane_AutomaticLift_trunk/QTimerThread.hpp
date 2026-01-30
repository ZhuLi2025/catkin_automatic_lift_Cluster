#ifndef QTIMERTHREAD_H
#define QTIMERTHREAD_H
#include <QThread>
#include <QList>
#include <QTimer>

class QTimerThread : public QObject
{
    Q_OBJECT
public:
    QTimerThread(int iCount);
    ~QTimerThread();
    void createItem();
    void startMultThread();
public slots:
    void update();
private:
    int m_iThreadCount;//开启的线程个数
    QList<QTimer*> m_qTimerList;
    QList<QThread*> m_threadList;
};

#endif // QTIMERTHREAD_H

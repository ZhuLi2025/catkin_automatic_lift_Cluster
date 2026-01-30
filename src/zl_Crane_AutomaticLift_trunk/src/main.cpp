/**
 * @file /src/main.cpp
 *
 * @brief Qt based gui.
 *
 * @date November 2010
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QApplication>
#include<QTextCodec>
#include "main_window.hpp"
/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {

    /*********************
    ** Qt
    **********************/
    QApplication app(argc, argv);
    ros::init(argc, argv, "zl_Crane_AutomaticLift_trunk");
    ros::start();

   QTextCodec::setCodecForLocale(QTextCodec::codecForName("UTF-8"));//解决汉字乱码问题
    zl_Crane_AutomaticLift_trunk::MainWindow w(argc,argv);
    w.show();
    w.setWindowTitle(QString::fromLocal8Bit("起重机智能吊装辅助系统"));

//    w.connect_Camera_Pcan();
    int result = app.exec();

	return result;
}

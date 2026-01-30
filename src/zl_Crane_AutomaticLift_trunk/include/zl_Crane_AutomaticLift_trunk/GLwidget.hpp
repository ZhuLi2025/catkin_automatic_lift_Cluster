#define _USE_MATH_DEFINES
#include <cmath>
#include <GL/glu.h>
#include <QColor>
#include <QFont>
#include <QObject>
#include <QOpenGLFunctions>
#include <QOpenGLWidget>
#include <QPainter>
#include <QVector3D>
#include <iostream>
#include <mutex>

namespace ompl
{
namespace app
{
class GLWidget: public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT
    public:
    explicit GLWidget(QWidget *parent = 0);
    ~GLWidget();

    void setRobotGLList(const GLint& gl_list_robot);
    void setEnvironmentGLList(const GLint& gl_list_environment);
    void setSolutionTrajectoryEETipPath(const GLint& gl_list);
    void roll(const double angle);
    void pitch(const double angle);

    protected:
    //Opengl stuff
    void initializeGL() Q_DECL_OVERRIDE;
    void resizeGL(int w, int h) Q_DECL_OVERRIDE;
    void paintGL() Q_DECL_OVERRIDE;

    //some event
    virtual void keyPressEvent(QKeyEvent *event) Q_DECL_OVERRIDE;
    virtual void mousePressEvent(QMouseEvent *event) Q_DECL_OVERRIDE;
    virtual void mouseMoveEvent(QMouseEvent *event) Q_DECL_OVERRIDE;
    virtual void wheelEvent(QWheelEvent *event) Q_DECL_OVERRIDE;

    void initScene();
    void goToDefaultView();

    GLuint generateGridList(const int size);
    void drawWorldFrame();

    private:
    struct Camera
    {
        QVector3D eye = QVector3D(0, 0, 10);
        QVector3D center = QVector3D(0, 0, 0);
        QVector3D up = QVector3D(0, 1, 0);
        QVector3D view()
        {
            return center - eye;
        }

        void zoom(float viewDistance)
        {
            eye = center - view().normalized() * viewDistance;
        }
    };

    private:
    Camera camera_;
    QPoint mouse_position_;
    std::mutex mutex_;

    protected:
    GLint ground_grid_;
    GLint gl_list_robot_;
    GLint gl_list_environment_;
    GLint gl_list_solution_trajectory_ee_tip_path_;
};
} // namespace app
} // namespace ompl

#include <QKeyEvent>
#include <QMatrix4x4>
#include <QMessageBox>
#include <QMouseEvent>
#include <QVector4D>
#include <QWheelEvent>

#include "GLwidget.hpp"

namespace ompl
{
namespace app
{
    GLWidget::GLWidget(QWidget *parent):
        QOpenGLWidget(parent)
    {
//        QSurfaceFormat format = QSurfaceFormat::defaultFormat();
//        format.setSamples(16);
//        format.setSwapInterval(0);
//        QSurfaceFormat::setDefaultFormat(format);
//        this->setFormat(format);
        //setAttribute(Qt::WA_DontCreateNativeAncestors);
        ground_grid_ = 0;
        gl_list_robot_ = 0;
        gl_list_environment_ = 0;
        gl_list_solution_trajectory_ee_tip_path_ = 0;

        //init eye
        camera_.eye = QVector3D(0, 40, 10);
        camera_.center = QVector3D(0, 0, 12);
        camera_.up = QVector3D(0, -1, 1);
    }

    GLWidget::~GLWidget()
    {
    }

    void GLWidget::setRobotGLList(const GLint& gl_list_robot)
    {
        //this->makeCurrent();
        if (gl_list_robot_)
        {
            glDeleteLists(gl_list_robot_, 1);
        }
        gl_list_robot_ = gl_list_robot;
        update();
    }

    void GLWidget::setEnvironmentGLList(const GLint& gl_list_environment)
    {
        //this->makeCurrent();
        if (gl_list_environment_)
        {
            glDeleteLists(gl_list_environment_, 1);
        }
        gl_list_environment_ = gl_list_environment;
        update();
    }

    void GLWidget::setSolutionTrajectoryEETipPath(const GLint& gl_list)
    {
        mutex_.lock();
        //this->makeCurrent();
        if (gl_list_solution_trajectory_ee_tip_path_)
        {
            glDeleteLists(gl_list_solution_trajectory_ee_tip_path_, 1);
        }
        gl_list_solution_trajectory_ee_tip_path_ = gl_list;
        mutex_.unlock();
        update();
    }

    void GLWidget::initializeGL()
    {
        this->makeCurrent();
        initializeOpenGLFunctions();
        glClearColor(0.25, 0.25, 0.25, 1.);
        glEnable(GL_LIGHTING);
        glEnable(GL_LIGHT0);
        glEnable(GL_LIGHT1);
        glEnable(GL_DEPTH_TEST);
        glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);

        glEnable(GL_NORMALIZE);
        glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);

        glEnable(GL_LINE_SMOOTH);
        glShadeModel(GL_FLAT);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        //std::cout << "THHHHHHHHHHHHHHH: " << glGetString(GL_VERSION) << std::endl;
        //init the scene
        initScene();
    }

    void GLWidget::initScene()
    {
        ground_grid_ = generateGridList(10);
    }

    void GLWidget::resizeGL(int w, int h)
    {
        int side = std::min(w, h);
        if (side <= 0)
            return;
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        gluPerspective(45., float(w) / float(h), 1., 1000.);
        glViewport(0, 0, w, h);
    }

    void GLWidget::paintGL()
    {
        if (!isVisible())
        {
            return;
        }
        //this->makeCurrent();
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        // lookat
        QVector3D zhat = -camera_.view().normalized();
        QVector3D xhat = QVector3D::crossProduct(camera_.up, zhat).normalized();
        QVector3D yhat = QVector3D::crossProduct(zhat, xhat).normalized();
        float eyeX = QVector3D::dotProduct(camera_.eye, xhat);
        float eyeY = QVector3D::dotProduct(camera_.eye, yhat);
        float eyeZ = QVector3D::dotProduct(camera_.eye, zhat);
        float lookat[16] = {
            xhat.x(), yhat.x(), zhat.x(), 0,
            xhat.y(), yhat.y(), zhat.y(), 0,
            xhat.z(), yhat.z(), zhat.z(), 0,
            -eyeX,    -eyeY,    -eyeZ,    1
        };
        glMatrixMode(GL_MODELVIEW);
        glLoadMatrixf(lookat);
        if (ground_grid_)
            glCallList(ground_grid_);

        if (gl_list_robot_ && glIsList(gl_list_robot_))
            glCallList(gl_list_robot_);

        if (gl_list_environment_ && glIsList(gl_list_environment_))
            glCallList(gl_list_environment_);

        if (gl_list_solution_trajectory_ee_tip_path_ && glIsList(gl_list_solution_trajectory_ee_tip_path_))
            glCallList(gl_list_solution_trajectory_ee_tip_path_);

        drawWorldFrame();
    }

    GLuint GLWidget::generateGridList(const int size)
    {
        GLuint grid_list = glGenLists(1);
        glNewList(grid_list, GL_COMPILE);
        glDisable(GL_LIGHTING);
        glDisable(GL_COLOR_MATERIAL);
        glLineWidth(1);
        glColor3f(0.50f, 0.50f, 0.50f);//white color
        glBegin(GL_LINES);
        for(int i=-size;i<=size;i++)
        {
            glVertex3f((float)i,(float)-size, 0);
            glVertex3f((float)i,(float)size, 0);

            glVertex3f((float)-size,(float)i, 0);
            glVertex3f((float)size,(float)i, 0);
        }
        glEnd();
        glEndList();
        return grid_list;
    }

    void GLWidget::drawWorldFrame()
    {
        glPushAttrib(GL_COLOR_BUFFER_BIT);
        glPushAttrib(GL_LINE_BIT);
        glLineWidth(8);
        glBegin(GL_LINES);
        // x
        glColor3f(1, 0, 0);
        glVertex3f(0, 0, 0);
        glVertex3f(1, 0, 0);
        // y
        glColor3f(0, 1, 0);
        glVertex3f(0, 0, 0);
        glVertex3f(0, 1, 0);
        // z
        glColor3f(0, 0, 1);
        glVertex3f(0, 0, 0);
        glVertex3f(0, 0, 1);
        glEnd();
        glPopAttrib(); // GL_LINE_BIT
        glPopAttrib(); // GL_COLOR_BUFFER_BIT
    }

    void GLWidget::keyPressEvent(QKeyEvent *event)
    {
        switch(event->key())
        {
            case Qt::Key_Backspace:
            case Qt::Key_Delete:
                return;

            case Qt::Key_A:
                goToDefaultView();
                return;
        }
        QOpenGLWidget::keyPressEvent(event);
    }

    void GLWidget::goToDefaultView()
    {
        camera_.eye = QVector3D(0, 0, 10);
        camera_.center = QVector3D(0, 0, 0);
        camera_.up = QVector3D(0, 1, 0);
        repaint();
    }

    void GLWidget::mousePressEvent(QMouseEvent *event)
    {
        mouse_position_ = event->pos();
        QOpenGLWidget::mousePressEvent(event);
    }

    void GLWidget::mouseMoveEvent(QMouseEvent *event)
    {
        // Transform scene.
        bool rotate_xy = event->buttons() & Qt::RightButton;
        bool rotate_z = event->buttons() & Qt::LeftButton;
        bool pan = event->buttons() & Qt::MiddleButton;

        float dx = event->x() - mouse_position_.x();
        float dy = -(event->y() - mouse_position_.y());
        mouse_position_ = event->pos();
        QVector3D zhat = -camera_.view().normalized();
        QVector3D xhat = QVector3D::crossProduct(camera_.up, zhat).normalized();
        QVector3D yhat = QVector3D::crossProduct(zhat, xhat).normalized();
        if(rotate_xy)
        {
            QVector3D rotationAxis = yhat * dx - xhat * dy;
            float n = rotationAxis.length();
            if(n > 1e-5)
            {
                rotationAxis.normalize();
                float radians = n / width() * M_PI;
                // Rotate eye about center around rotation axis.
                float a = cos(radians / 2);
                float s = -sin(radians / 2);
                float b = rotationAxis.x() * s;
                float c = rotationAxis.y() * s;
                float d = rotationAxis.z() * s;
                float rotation[9] = {
                    a*a+b*b-c*c-d*d, 2*(b*c-a*d),     2*(b*d+a*c),
                    2*(b*c+a*d),     a*a+c*c-b*b-d*d, 2*(c*d-a*b),
                    2*(b*d-a*c),     2*(c*d+a*b),     a*a+d*d-b*b-c*c
                };
                camera_.eye -= camera_.center; // Shift center to origin so can rotate eye about axis through the origin.
                // Rotate eye around rotation axis.
                float ex = camera_.eye.x() * rotation[0] + camera_.eye.y() * rotation[1] + camera_.eye.z() * rotation[2];
                float ey = camera_.eye.x() * rotation[3] + camera_.eye.y() * rotation[4] + camera_.eye.z() * rotation[5];
                float ez = camera_.eye.x() * rotation[6] + camera_.eye.y() * rotation[7] + camera_.eye.z() * rotation[8];
                camera_.eye = QVector3D(ex, ey, ez);
                camera_.eye += camera_.center; // shift back to center.
                repaint();
                return;
            }
        }
        else if (rotate_z)
        {
            QVector3D rotationAxis = QVector3D(0, 0, 1);
            float radians = dx > 0? 0.05: -0.05;
            //std::cout << "dy: " << dy << std::endl;
            // Rotate eye about center around rotation axis.
            float a = cos(radians / 2);
            float s = -sin(radians / 2);
            float b = rotationAxis.x() * s;
            float c = rotationAxis.y() * s;
            float d = rotationAxis.z() * s;
            float rotation[9] = {
                a*a+b*b-c*c-d*d, 2*(b*c-a*d),     2*(b*d+a*c),
                2*(b*c+a*d),     a*a+c*c-b*b-d*d, 2*(c*d-a*b),
                2*(b*d-a*c),     2*(c*d+a*b),     a*a+d*d-b*b-c*c
            };
            camera_.eye -= camera_.center; // Shift center to origin so can rotate eye about axis through the origin.
            // Rotate eye around rotation axis.
            float ex = camera_.eye.x() * rotation[0] + camera_.eye.y() * rotation[1] + camera_.eye.z() * rotation[2];
            float ey = camera_.eye.x() * rotation[3] + camera_.eye.y() * rotation[4] + camera_.eye.z() * rotation[5];
            float ez = camera_.eye.x() * rotation[6] + camera_.eye.y() * rotation[7] + camera_.eye.z() * rotation[8];

            float ux = camera_.up.x() * rotation[0] + camera_.up.y() * rotation[1] + camera_.up.z() * rotation[2];
            float uy = camera_.up.x() * rotation[3] + camera_.up.y() * rotation[4] + camera_.up.z() * rotation[5];
            float uz = camera_.up.x() * rotation[6] + camera_.up.y() * rotation[7] + camera_.up.z() * rotation[8];

            camera_.eye = QVector3D(ex, ey, ez);
            camera_.up = QVector3D(ux, uy, uz);
            camera_.eye += camera_.center; // shift back to center.
            repaint();
            return;
        }
        else if(pan)
        {
            float zoom = camera_.view().length();
            QVector3D translation = xhat * (dx / width() * zoom) + yhat * (dy / height() * zoom);
            camera_.center -= translation;
            camera_.eye -= translation;
            repaint();
            return;
        }
        mouse_position_ = event->pos();
        QOpenGLWidget::mouseMoveEvent(event);
    }

    void GLWidget::wheelEvent(QWheelEvent *event)
    {
        #if QT_VERSION >= 0x050000
        float degrees = event->angleDelta().y() / 8;
        #else
        float degrees = event->delta() / 8;
        #endif
        float steps = degrees / 15;  // Most mouse types work in steps of 15 degrees.
        if(steps == 0) return;

        float zoom = camera_.view().length();
        zoom += zoom * steps * 0.05; //mouse wheel sensitivity
        if(zoom < 1e-5)
        {
            zoom = 1e-5;
        }
        camera_.zoom(zoom);
        repaint();
    }

    void GLWidget::roll(const double angle)
    {
        QVector3D zhat = -camera_.view().normalized();
        QVector3D xhat = QVector3D::crossProduct(camera_.up, zhat).normalized();
        QVector3D yhat = QVector3D::crossProduct(zhat, xhat).normalized();
        QVector3D rotationAxis = QVector3D(0, 0, 1);
        float radians = angle;//dx > 0? 0.05: -0.05;
        //std::cout << "dy: " << dy << std::endl;
        // Rotate eye about center around rotation axis.
        float a = cos(radians / 2);
        float s = -sin(radians / 2);
        float b = rotationAxis.x() * s;
        float c = rotationAxis.y() * s;
        float d = rotationAxis.z() * s;
        float rotation[9] = {
            a*a+b*b-c*c-d*d, 2*(b*c-a*d),     2*(b*d+a*c),
            2*(b*c+a*d),     a*a+c*c-b*b-d*d, 2*(c*d-a*b),
            2*(b*d-a*c),     2*(c*d+a*b),     a*a+d*d-b*b-c*c
        };
        camera_.eye -= camera_.center; // Shift center to origin so can rotate eye about axis through the origin.
        // Rotate eye around rotation axis.
        float ex = camera_.eye.x() * rotation[0] + camera_.eye.y() * rotation[1] + camera_.eye.z() * rotation[2];
        float ey = camera_.eye.x() * rotation[3] + camera_.eye.y() * rotation[4] + camera_.eye.z() * rotation[5];
        float ez = camera_.eye.x() * rotation[6] + camera_.eye.y() * rotation[7] + camera_.eye.z() * rotation[8];

        float ux = camera_.up.x() * rotation[0] + camera_.up.y() * rotation[1] + camera_.up.z() * rotation[2];
        float uy = camera_.up.x() * rotation[3] + camera_.up.y() * rotation[4] + camera_.up.z() * rotation[5];
        float uz = camera_.up.x() * rotation[6] + camera_.up.y() * rotation[7] + camera_.up.z() * rotation[8];

        camera_.eye = QVector3D(ex, ey, ez);
        camera_.up = QVector3D(ux, uy, uz);
        camera_.eye += camera_.center; // shift back to center.
        repaint();
        return;
    }

    void GLWidget::pitch(const double angle)
    {
        float dx;
        if (angle < 0)
            dx = -1;
        else
            dx = 1;
        float dy = angle * 100;
        QVector3D zhat = -camera_.view().normalized();
        QVector3D xhat = QVector3D::crossProduct(camera_.up, zhat).normalized();
        QVector3D yhat = QVector3D::crossProduct(zhat, xhat).normalized();
        QVector3D rotationAxis = yhat * dx - xhat * dy;
        float n = rotationAxis.length();
        if(n > 1e-5)
        {
            rotationAxis.normalize();
            float radians = n / width() * M_PI;
            // Rotate eye about center around rotation axis.
            float a = cos(radians / 2);
            float s = -sin(radians / 2);
            float b = rotationAxis.x() * s;
            float c = rotationAxis.y() * s;
            float d = rotationAxis.z() * s;
            float rotation[9] = {
                a*a+b*b-c*c-d*d, 2*(b*c-a*d),     2*(b*d+a*c),
                2*(b*c+a*d),     a*a+c*c-b*b-d*d, 2*(c*d-a*b),
                2*(b*d-a*c),     2*(c*d+a*b),     a*a+d*d-b*b-c*c
            };
            camera_.eye -= camera_.center; // Shift center to origin so can rotate eye about axis through the origin.
            // Rotate eye around rotation axis.
            float ex = camera_.eye.x() * rotation[0] + camera_.eye.y() * rotation[1] + camera_.eye.z() * rotation[2];
            float ey = camera_.eye.x() * rotation[3] + camera_.eye.y() * rotation[4] + camera_.eye.z() * rotation[5];
            float ez = camera_.eye.x() * rotation[6] + camera_.eye.y() * rotation[7] + camera_.eye.z() * rotation[8];
            camera_.eye = QVector3D(ex, ey, ez);
            camera_.eye += camera_.center; // shift back to center.
            repaint();
            return;
        }
    }
} // namespace app
} // namespace ompl

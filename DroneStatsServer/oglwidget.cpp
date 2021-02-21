#include "oglwidget.h"
#include "iostream"
#include <QtOpenGL/qgl.h>
#include <QtOpenGL/QtOpenGL>
#include <GL/glu.h>
#include <GL/gl.h>
//#include <Utils/sout.h>

OGLWidget::OGLWidget(QWidget *parent)
    : QOpenGLWidget(parent)
{

}

OGLWidget::~OGLWidget()
{

}

void OGLWidget::initializeGL()
{
    //glfwWindowHint(GLFW_SAMPLES, 4);
    //QSurfaceFormat fmt;
    //fmt.setSamples(4);
    //setFormat(fmt);
    glClearColor(0,0,0,1);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHTING);
    //glEnable(GL_MULTISAMPLE);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    glEnable(GL_COLOR_MATERIAL);

}


void OGLWidget::paintGL()
{

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    pointsMutex.lock();
    GLfloat fSizes [20];
    glGetFloatv(GL_LINE_WIDTH_RANGE,fSizes);
    GLfloat fCurrentSize = fSizes[0];
    fCurrentSize+=1.0f;
    glLineWidth(fCurrentSize);
    glBegin(GL_LINE_LOOP);
    glColor3f(1.0, 0.0, 0.0);
    glVertex3f(0,0,0);
    glVertex3f(400,0,0);
    glEnd();
    glBegin(GL_LINE_LOOP);
    glColor3f(0.0, 1.0, 0.0);
    glVertex3f(0,0,0);
    glVertex3f(0,400,0);
    glEnd();
    glBegin(GL_LINE_LOOP);
    glColor3f(0.0, 0.0, 1.0);
    glVertex3f(0,0,0);
    glVertex3f(0,0,400);
    glEnd();
    fCurrentSize+=2.0f;
    glLineWidth(fCurrentSize);
    glColor3f(1.0, 0.56, 0.0);
    int iii=0;
    glBegin(GL_LINE_STRIP);
    for(int i=1;i<points.size();i++){
        //glVertex3f((GLfloat)points[i-1].x,(GLfloat)points[i-1].z,(GLfloat)points[i-1].y);
        glVertex2f((double )points[i].x,(double)points[i].z);

        iii++;
    }
    glEnd();
    //std::cout<<"repaint "<<iii<<" points"<<std::endl;
    pointsMutex.unlock();
}


void OGLWidget::renderText(double x, double y, const QString &str, bool bold, QColor color)
{
    // Identify x and y locations to render text within widget
    int height = this->height();
    GLdouble textPosX = 0, textPosY = 0, textPosZ = 0;
    //project(x, y, 0, &textPosX, &textPosY, &textPosZ);
    textPosY = height - textPosY; // y is inverted
    // Retrieve last OpenGL color to use as a font color
    // Render text
    QPainter painter(this);
    //painter.setPen(color);
    painter.setPen(color);
    QFont font("Helvetica", 8);
    font.setBold(bold);
    painter.setFont(font);
    painter.setRenderHints(QPainter::Antialiasing | QPainter::TextAntialiasing);
    painter.drawText(x, y, str);
    painter.end();
}


void OGLWidget::resizeGL(int w, int h)
{
    //glViewport(0,0,w,h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    //gluPerspective(45, (float)9/16, 1.0, 1.0);
    gluOrtho2D(-w*2,w*2,h*2,-h*2);
    //gluOrtho2D(0,1600,900,0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    //gluLookAt(0,0,5,0,0,0,0,1,0);
}

void OGLWidget::mousePressEvent(QMouseEvent *event)
{

}

void OGLWidget::mouseMoveEvent(QMouseEvent *event)
{

}



void OGLWidget::mouseReleaseEvent(QMouseEvent *event) {

}

void OGLWidget::mouseDoubleClickEvent(QMouseEvent *event)
{

}

void OGLWidget::getCoordinatespoint(CvPoint3D32f point)
{
    bool hasNewPoint = points.empty();
    pointsMutex.lock();
    if (!hasNewPoint){
        CvPoint3D32f lastpoint = points[points.size() - 1];
        CvPoint3D32f diff (lastpoint.x - point.x,lastpoint.y - point.y,lastpoint.z - point.z);
        hasNewPoint = diff.x * diff.x + diff.y * diff.y + diff.z * diff.z > 1.0;
    }
    if (hasNewPoint){
        points.push_back(point);
    }
    pointsMutex.unlock();
    if (hasNewPoint){
        update();
    }
}
#ifndef OGLWIDGET_H
#define OGLWIDGET_H

#include <QWidget>
#include <QOpenGLWidget>

class OGLWidget : public QOpenGLWidget
 {
    Q_OBJECT
    void mouseMoveEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
    void mouseDoubleClickEvent(QMouseEvent *event);
public:
    OGLWidget(QWidget *parent = 0);
    ~OGLWidget();
    void mousePressEvent(QMouseEvent *event);
    //oglFont *glFont;
    void renderText(double x, double y, const QString &str, bool bold = false, QColor color = Qt::black);
protected:
    void initializeGL();
    void resizeGL(int w, int h);
    void paintGL();

signals:


};

#endif // OGLWIDGET_H

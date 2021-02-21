#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "statsclient.h"

#include <QMainWindow>
#include <mutex>
#include "../Utils/MutexBool.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void setTestImage();
public slots:
    void setWarningText(QString text);
    void setLeftImage(QImage value);
    void setRightImage(QImage value);
    void setOnboardVideoCaptureMode(bool mode);
    void setVideoStreamMode(bool mode);
    void setConnected(bool connected);                   //on server tcp connection on/off changes
    void setPing(QString q);

private slots:
    void on_connectButton_released();
    void on_getImageStreamButton_pressed();
    void on_onBoardVideoCapture_released();
    void getCoordinatespoint(CvPoint3D32f point);
    void onnnTargetXEditingFinished();
    void onnnTargetYEditingFinished();
    void onnnTargetZEditingFinished();
    void checkTargetPosition();

    void on_setTargetPointButton_released();

    void on_ConnectToPX4_released();

    void on_startFlightButton_released();

    void on_SetCurrentPointAsZerroButton_released();

private:
    Ui::MainWindow *ui;
    StatsClient client;
    QImage leftImage;
    QImage rightImage;
    MutexBool hasVideoStream{false};
    MutexBool onboardVideoRecordingMode{false};

    std::mutex imageMutex;


    QImage getLeftImage();
    QImage getRightImage();
};
#endif // MAINWINDOW_H

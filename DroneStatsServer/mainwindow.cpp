#include <thread>
#include <zconf.h>
#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "iostream"
#include "../Utils/Messages.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    qRegisterMetaType<CvPoint3D32f>("CvPoint3D32f");             //now we can use this messages in signal/slot system as QObjects
    connect(&client,SIGNAL(transmit_to_gui(QString)),this,SLOT(setWarningText(QString)));
    connect(&client,SIGNAL(transmit_to_left_image(QImage)),this,SLOT(setLeftImage(QImage)));
    connect(&client,SIGNAL(transmit_to_right_image(QImage)),this,SLOT(setRightImage(QImage)));
    connect(&client,SIGNAL(transmitOnboardVideoCaptureStatus(bool)),this,SLOT(setOnboardVideoCaptureMode(bool)));
    connect(&client,SIGNAL(transmitVideoStreamStatus(bool)),this,SLOT(setVideoStreamMode(bool)));
    connect(&client,SIGNAL(transmitConnectionStatus(bool)),this,SLOT(setConnected(bool)));
    connect(&client,SIGNAL(transmitPing(QString)),this,SLOT(setPing(QString)));
    connect(&client,SIGNAL(transmitCoordinates(CvPoint3D32f)),this,SLOT(getCoordinatespoint(CvPoint3D32f)));

    std::thread thr([this]()
                    {
                        int cnt = 0;
                        while (true) {
                            if (!client.closeConnectionThreadBool.get()) {
                                cnt++;
                                //std::cout << "image updated "<<cnt<<std::endl;
                                //QString name = "im";
                                //name+=QString::number(cnt);
                                //getLeftImage().save(name, "JPG");
                                ui->leftImageLabel->setPixmap(QPixmap::fromImage(getLeftImage()));
                                ui->rightImageLabel->setPixmap(QPixmap::fromImage(getRightImage()));

                            }
                            usleep(40000);
                        }
                    });
    thr.detach();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::setTestImage()
{
    //QPixmap pic("/home/nickolay/DroneStatsServer/testim.jpg");
    QPixmap pic("../../testim.jpg");
    pic = pic.scaled(320,240);
    ui->leftImageLabel->setPixmap(pic);
    ui->rightImageLabel->setPixmap(pic);
}

void MainWindow::getCoordinatespoint(CvPoint3D32f point) {
    ui->sysLabel->setText("Point x:"+QString::number(point.x)+" y:"+QString::number(point.y)+" z:"+QString::number(point.z));
    //ui->openGLWidget->getCoordinatespoint(point);
}

void MainWindow::setPing(QString q)
{

    //QString q = "to Raspberry: " + QString::number(to) + "\n" + "from Raspberry: " + QString::number(from);
    ui->infoLabelRPI->setText(q);
}

void MainWindow::on_connectButton_released()
{

    client.connectToDroneServer(ui->ipEdit->text().toStdString());
    //setConnected(true);
    ui->getImageStreamButton->setEnabled(true);
    ui->onBoardVideoCapture->setEnabled(true);
}

void MainWindow::setWarningText(QString text)
{
    ui->sysLabel->setText(text);
}

void MainWindow::setLeftImage(QImage value)
{
    imageMutex.lock();
    leftImage = value.scaled(320,240);
    imageMutex.unlock();
}

void MainWindow::setConnected(bool b)
{
    ui->connectButton->setEnabled(!b);
    ui->ipEdit->setEnabled(!b);
}

void MainWindow::setRightImage(QImage value)
{
    imageMutex.lock();
    rightImage = value.scaled(320,240);
    imageMutex.unlock();
}

QImage MainWindow::getLeftImage()
{
    imageMutex.lock();
    //leftImage = value.scaled(320,240);
    QImage result(leftImage);
    //ui->leftImageLabel->setPixmap(QPixmap::fromImage(value));
    imageMutex.unlock();
    return result;
}

QImage MainWindow::getRightImage()
{
    imageMutex.lock();
    //leftImage = value.scaled(320,240);
    QImage result(rightImage);
    //ui->leftImageLabel->setPixmap(QPixmap::fromImage(value));
    imageMutex.unlock();
    return result;
}


void MainWindow::on_getImageStreamButton_pressed()
{
    if (!hasVideoStream.get()) {
        SystemMessage m;
        m.type = SystemMessage::START_VIDEO_STREAM;
        client.sendMessage(m);
    } else{
        SystemMessage m;
        m.type = SystemMessage::STOP_VIDEO_STREAM;
        client.sendMessage(m);
    }
}

void MainWindow::on_onBoardVideoCapture_released()
{
    if (!onboardVideoRecordingMode.get()){
        SystemMessage m;
        m.type = SystemMessage::START_IMAGE_CAPTURE;
        client.sendMessage(m);
    }
    else{
        SystemMessage m;
        m.type = SystemMessage::STOP_IMAGE_CAPTURE;
        client.sendMessage(m);
    }
}

void MainWindow::setOnboardVideoCaptureMode(bool mode)
{
    onboardVideoRecordingMode.set(mode);
    if (mode)
    {
        ui->onBoardVideoCapture->setText("Stop video record ");
    } else{
        ui->onBoardVideoCapture->setText("Record video on board ");
    }
}

void MainWindow::setVideoStreamMode(bool mode)
{
    hasVideoStream.set(mode);
    if (mode)
    {
        ui->getImageStreamButton->setText("Turn off video");
    } else{
        ui->getImageStreamButton->setText("Turn on video");
    }

}

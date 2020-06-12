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
    connect(&client,SIGNAL(transmit_to_gui(QString)),this,SLOT(setWarningText(QString)));
    connect(&client,SIGNAL(transmit_to_left_image(QImage)),this,SLOT(setLeftImage(QImage)));
    connect(&client,SIGNAL(transmit_to_right_image(QImage)),this,SLOT(setRightImage(QImage)));
    connect(&client,SIGNAL(transmitOnboardVideoCaptureStatus(bool)),this,SLOT(setOnboardVideoCaptureMode(bool)));
    connect(&client,SIGNAL(transmitVideoStreamStatus(bool)),this,SLOT(setVideoStreamMode(bool)));


    std::thread thr([this]()
                    {
                        int cnt = 0;
                        while (true) {
                            if (getConnected()) {
                                cnt++;
                                //std::cout << "image updated "<<cnt<<std::endl;
                                QString name = "im";
                                name+=QString::number(cnt);
                                getLeftImage().save(name, "JPG");
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
    QPixmap pic("/home/nickolay/DroneStatsServer/testim.jpg");
    pic = pic.scaled(320,240);
    ui->leftImageLabel->setPixmap(pic);
    ui->rightImageLabel->setPixmap(pic);
}

void MainWindow::on_connectButton_released()
{
    client.connectToDroneServer();
    setConnected(true);
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
    imageSendMode = 1;
    leftImage = value.scaled(320,240);
    //ui->leftImageLabel->setPixmap(QPixmap::fromImage(value));
    imageMutex.unlock();
}

void MainWindow::setRightImage(QImage value)
{
    imageMutex.lock();
    imageSendMode = 1;
    rightImage = value.scaled(320,240);
    imageMutex.unlock();
    //ui->leftImageLabel->setPixmap(QPixmap::fromImage(value));
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

void MainWindow::setConnected(bool conn)
{
    connectedMutex.lock();
    connected = conn;
    connectedMutex.unlock();
}

bool MainWindow::getConnected()
{
    bool result;
    connectedMutex.lock();
    result = connected;
    connectedMutex.unlock();
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

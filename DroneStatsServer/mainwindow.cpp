#include <thread>
#include <zconf.h>
#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "iostream"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    connect(&client,SIGNAL(transmit_to_gui(QString)),this,SLOT(setWarningText(QString)));
    connect(&client,SIGNAL(transmit_to_left_image(QImage)),this,SLOT(setLeftImage(QImage)));
    std::thread thr([this]()
                    {
                        int cnt = 0;
                        while (true) {
                            if (getConnected()) {
                                cnt++;
                                std::cout << "image updated "<<cnt<<std::endl;
                                QString name = "im";
                                name+=QString::number(cnt);
                                getLeftImage().save(name, "JPG");
                                ui->leftImageLabel->setPixmap(QPixmap::fromImage(getLeftImage()));
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
}

void MainWindow::setWarningText(QString text)
{
    ui->sysLabel->setText(text);
}

void MainWindow::setLeftImage(QImage value)
{
    imageMutex.lock();
    leftImage = value.scaled(320,240);
    //ui->leftImageLabel->setPixmap(QPixmap::fromImage(value));
    imageMutex.unlock();
}

void MainWindow::setRightImage(QImage value)
{
    rightImage = value.scaled(320,240);
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


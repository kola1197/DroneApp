#include <thread>
#include <zconf.h>
#include <QMessageBox>
#include <linux/prctl.h>
#include <sys/prctl.h>
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
    //qRegisterMetaType<CvPoint3D32f>("std::vector<double>");
    connect(&client,SIGNAL(transmit_to_gui(QString)),this,SLOT(setWarningText(QString)));
    connect(&client,SIGNAL(transmit_to_left_image(QImage)),this,SLOT(setLeftImage(QImage)));
    connect(&client,SIGNAL(transmit_to_right_image(QImage)),this,SLOT(setRightImage(QImage)));
    connect(&client,SIGNAL(transmitOnboardVideoCaptureStatus(bool)),this,SLOT(setOnboardVideoCaptureMode(bool)));
    connect(&client, SIGNAL(transmitRamData(double, double)), this,SLOT(setRamData(double, double)) );
    connect(&client, SIGNAL(transmitCPUData(double, double,QVector<double>)), this,SLOT(setCPUData(double, double, QVector<double>)));
    connect(&client,SIGNAL(transmitVideoStreamStatus(bool)),this,SLOT(setVideoStreamMode(bool)));
    connect(&client,SIGNAL(transmitConnectionStatus(bool)),this,SLOT(setConnected(bool)));
    connect(&client,SIGNAL(transmitPing(QString)),this,SLOT(setPing(QString)));
    connect(&client,SIGNAL(transmitCoordinates(CvPoint3D32f)),this,SLOT(getCoordinatespoint(CvPoint3D32f)));
    connect(&client,SIGNAL(transmitTargetpointUpdated()),this,SLOT(setTargetPosition()));
    connect(&client,SIGNAL(transmitVehicleModeValue()),this,SLOT(setVehicleModeValue()));
    connect(&client,SIGNAL(transmitUpdatePX4Data()),this,SLOT(updatePX4Data()));
    connect(this,SIGNAL(transmitUpdateLabels()),this,SLOT(updateImages()));

    std::thread thr([this]()
                    {
                        std::string s = "GroundStationTHR";
                        prctl(PR_SET_NAME,(char *)s.c_str());
                        int cnt = 1;
                        while (cnt > 0) {
                            if (!client.closeConnectionThreadBool.get()) {
                                cnt++;
                                //std::cout << "image updated "<<cnt<<std::endl;
                                //QString name = "im";
                                //name+=QString::number(cnt);
                                //getLeftImage().save(name, "JPG");
                                //QPixmap pix;
                                //QPixmap pMap = QPixmap::fromImage(Image).scaled(label.size(),Qt::KeepAspectRatio);
                                /*ui->leftImageLabel->setPixmap(QPixmap::fromImage(getLeftImage()));
                                ui->rightImageLabel->setPixmap(QPixmap::fromImage(getRightImage()));
                                ui->leftImageLabel->resize(getLeftImage().size());
                                ui->rightImageLabel->resize(getRightImage().size());*/
                                emit(transmitUpdateLabels());
                            }
                            usleep(40000);
                        }
                    });
    thr.detach();

    checkTargetPosition();
}

void MainWindow::updateImages()
{
    ui->leftImageLabel->setPixmap(QPixmap::fromImage(getLeftImage()));
    ui->rightImageLabel->setPixmap(QPixmap::fromImage(getRightImage()));
    ui->leftImageLabel->resize(getLeftImage().size());
    ui->rightImageLabel->resize(getRightImage().size());
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
    ui->openGLWidget->getCoordinatespoint(point);
}

void MainWindow::setPing(QString q)
{
    //QString q = "to Raspberry: " + QString::number(to) + "\n" + "from Raspberry: " + QString::number(from);
    //ui->infoLabelRPI->setText(q);
    client.vehicleData.ping.set(q);
    updatePX4Data();
}

void MainWindow::on_connectButton_released()
{

    client.connectToDroneServer(ui->ipEdit->text().toStdString());
    //setConnected(true);
    //ui->getImageStreamButton->setEnabled(true);
    //ui->onBoardVideoCapture->setEnabled(true);
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
    ui->getImageStreamButton->setEnabled(b);
    ui->onBoardVideoCapture->setEnabled(b);
    ui->ConnectToPX4->setEnabled(b);
    //ui->setTargetPointButton->setEnabled(b);
    ui->SetCurrentPointAsZerroButton->setEnabled(b);
    //ui->startFlightButton->setEnabled(b);      //after px  connection
}

void MainWindow::setRamData(double ramSize, double ramFree) {
    client.vehicleData.ramTotal.set(ramSize);
    client.vehicleData.ramFree.set(ramFree);
    updatePX4Data();
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

void MainWindow::setCPUData(double CPUTemp, double avgLoad, QVector<double> coreLoad) {
    client.vehicleData.cpuTemp.set(CPUTemp);
    client.vehicleData.cpuAVGLoad.set(avgLoad);
    client.vehicleData.cpuCoreLoad.clear();
    for (int i=0; i<coreLoad.size(); i++){
        client.vehicleData.cpuCoreLoad.push_back(coreLoad[i]);
    }
    updatePX4Data();
}

void MainWindow::onnnTargetXEditingFinished()
{
    checkTargetPosition();
}

void MainWindow::onnnTargetYEditingFinished()
{
    checkTargetPosition();
}

void MainWindow::onnnTargetZEditingFinished()
{
    checkTargetPosition();
}

void MainWindow::setTargetPosition(){
    ui->targetX->setText(QString::number(client.vehicleData.targetpoint.get().x));
    ui->targetY->setText(QString::number(client.vehicleData.targetpoint.get().y));
    ui->targetZ->setText(QString::number(client.vehicleData.targetpoint.get().z));
    checkTargetPosition();
}

void MainWindow::checkTargetPosition() {
    bool xOk = false;
    float x = ui->targetX->text().toFloat(&xOk);

    bool yOk = false;
    float y = ui->targetY->text().toFloat(&yOk);

    bool zOk = false;
    float z = ui->targetZ->text().toFloat(&zOk);

    if (!xOk) {
        ui->targetX->setStyleSheet(
                "QLineEdit { background: rgb(255, 65, 65); selection-background-color: rgb(233, 99, 0); }");
    } else {
        ui->targetX->setStyleSheet(
                "QLineEdit { background: rgb(255, 255, 255); selection-background-color: rgb(233, 99, 0); }");
    }
    if (!yOk) {
        ui->targetY->setStyleSheet(
                "QLineEdit { background: rgb(255, 65, 65); selection-background-color: rgb(233, 99, 0); }");
    } else {
        ui->targetY->setStyleSheet(
                "QLineEdit { background: rgb(255, 255, 255); selection-background-color: rgb(233, 99, 0); }");
    }
    if (!zOk) {
        ui->targetZ->setStyleSheet(
                "QLineEdit { background: rgb(255, 65, 65); selection-background-color: rgb(233, 99, 0); }");
    } else {
        ui->targetZ->setStyleSheet(
                "QLineEdit { background: rgb(255, 255, 255); selection-background-color: rgb(233, 99, 0); }");
    }
    CvPoint3D32f targetpoint = client.vehicleData.targetpoint.get();
    if (xOk && yOk && zOk && (x != targetpoint.x || y != targetpoint.y || z != targetpoint.z))
    {
        ui->setTargetPointButton->setStyleSheet("QPushButton { background: rgb(255, 65, 65); selection-background-color: rgb(233, 99, 0); }");
        ui->setTargetPointButton->setEnabled(true);
    } else{
        ui->setTargetPointButton->setStyleSheet("QPushButton { background: rgb(65, 255, 65); selection-background-color: rgb(99, 233, 0); }");
        ui->setTargetPointButton->setEnabled(false);
    }



}
void MainWindow::on_setTargetPointButton_released()
{
    bool xOk = false;
    float x = ui->targetX->text().toFloat(&xOk);

    bool yOk = false;
    float y = ui->targetY->text().toFloat(&yOk);

    bool zOk = false;
    float z = ui->targetZ->text().toFloat(&zOk);

    if (xOk && yOk && zOk){
        if (client.connected.get()){
            CommandMessage c{};
            c.type = CommandMessage::SET_TARGET;
            c.f[0] = x;
            c.f[1] = y;
            c.f[2] = z;
            client.sendMessage(c);
        }
        else{
            QMessageBox msgBox;
            msgBox.setText("Please connect to vehicle first");
            msgBox.exec();
        }
    } else{
        checkTargetPosition();
    }
}

void MainWindow::on_ConnectToPX4_released()
{
    if (client.connected.get()){
        SystemMessage s{};
        s.type = SystemMessage::CONNECT_TO_PX;
        s.i[0] =1;
        client.sendMessage(s);
    }
    else{
        QMessageBox msgBox;
        msgBox.setText("Please connect to vehicle first");
        msgBox.exec();
    }
}

void MainWindow::on_startFlightButton_released()
{
    if (client.connected.get()){
        CommandMessage s{};
        s.type = CommandMessage::START;
        s.i[0] = 1;
        client.sendMessage(s);
    }
    else{
        QMessageBox msgBox;
        msgBox.setText("Please connect to vehicle first");
        msgBox.exec();
    }
}

void MainWindow::on_SetCurrentPointAsZerroButton_released()
{
    if (client.connected.get()){
        CommandMessage s{};
        s.type = CommandMessage::SET_THIS_POINT_AS_ZERO;
        s.i[0] = 1;
        client.sendMessage(s);
        ui->openGLWidget->clearPoints();
    }
    else{
        QMessageBox msgBox;
        msgBox.setText("Please connect to vehicle first");
        msgBox.exec();
    }
}

void MainWindow::on_horizontalSlider_valueChanged(int value)
{
    QMessageBox::StandardButton reply;
    reply = QMessageBox::question(this, "Vehicle mode", "Do you really want to change vehicle mode?",
                                  QMessageBox::Yes|QMessageBox::No);
    ui->horizontalSlider->setEnabled(false);
    if (reply == QMessageBox::Yes) {
        SettingsMessage s{};
        s.type = SettingsMessage::VEHICLE_MODE;
        s.i[0] = value;
        client.sendMessage(s);
    } else {
        setVehicleModeValue();
    }

}

void MainWindow::setVehicleModeValue()
{
    ui->horizontalSlider->setEnabled(true);
    ui->horizontalSlider->setValue(client.vehicleData.vehicleMode.get());
}

void MainWindow::on_horizontalSlider_rangeChanged(int min, int max)
{

}

void MainWindow::updatePX4Data()
{
    QString text = "";
    text += client.vehicleData.connectedToPx.get()? "Connected" : "NOT Connected to PX4\n";
    text += client.vehicleData.connectionCounter.get()>2 && !client.vehicleData.connectedToPx.get() ?  "Trying to connect "+QString::number(client.vehicleData.connectionCounter.get())+" seconds remain\n" : "\n";
    ui->PXInfoLabel->setText(text);
    QString raspberryText = "";
    raspberryText += client.vehicleData.ping.get()+"\n";
    //std::cout<<client.vehicleData.ramTotal.get()<<"   "<<client.vehicleData.ramFree.get()<<std::endl;
    raspberryText += "Ram Usage: " + QString::number(client.vehicleData.ramTotal.get() - client.vehicleData.ramFree.get())+"/"+QString::number(client.vehicleData.ramTotal.get());
    ui->infoLabelRPI->setText(raspberryText);
    QString CPUText = "";
    CPUText += "CPU temp: " + QString::number(client.vehicleData.cpuTemp.get())+"            AVG Load: "+QString::number(client.vehicleData.cpuAVGLoad.get())+"\n";
    for (int i = 0; i<client.vehicleData.cpuCoreLoad.size(); i++){
        QString value = QString::number(client.vehicleData.cpuCoreLoad[i]);
        CPUText+= QString::number(i+1)+"] "+value+"     ";
        for (int j=0;j<3-value.length();j++){
            CPUText+=" ";
        }
        if ( i % 4 == 3){
            CPUText+= "\n";
        }
    }
    ui->CPULabel->setText(CPUText);
}

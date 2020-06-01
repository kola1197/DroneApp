#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "statsclient.h"

#include <QMainWindow>
#include <mutex>

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

private slots:
    void on_connectButton_released();


private:
    Ui::MainWindow *ui;
    StatsClient client;
    QImage leftImage;
    QImage rightImage;

    std::mutex connectedMutex;

    std::mutex imageMutex;
    bool connected = false;
    void setConnected(bool conn);
    bool getConnected();

    QImage getLeftImage();
    QImage getRightImage();
};
#endif // MAINWINDOW_H

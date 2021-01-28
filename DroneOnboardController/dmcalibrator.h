#ifndef DMCALIBRATOR_H
#define DMCALIBRATOR_H

#include <QMainWindow>

namespace Ui {
class dmCalibrator;
}

class dmCalibrator : public QMainWindow
{
    Q_OBJECT

public:
    explicit dmCalibrator(QWidget *parent = nullptr);
    ~dmCalibrator();

private:
    Ui::dmCalibrator *ui;
};

#endif // DMCALIBRATOR_H

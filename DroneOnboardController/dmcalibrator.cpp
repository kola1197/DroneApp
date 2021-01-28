#include "dmcalibrator.h"
#include "ui_dmcalibrator.h"

dmCalibrator::dmCalibrator(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::dmCalibrator)
{
    ui->setupUi(this);
}

dmCalibrator::~dmCalibrator()
{
    delete ui;
}

#ifndef DEPTHMAPCALIBRATOR_H
#define DEPTHMAPCALIBRATOR_H

#include <QMainWindow>

QT_BEGIN_NAMESPACE
namespace Ui {
class DepthMapCalibrator;
}
QT_END_NAMESPACE

class DepthMapCalibrator : public QMainWindow
{
    Q_OBJECT

public:
    explicit DepthMapCalibrator(QWidget *parent = nullptr);
    ~DepthMapCalibrator();

private:
    Ui::DepthMapCalibrator *ui;
public slots:
    void updateValues();
};

#endif // DEPTHMAPCALIBRATOR_H

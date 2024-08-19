#ifndef MOTORKERNEL_H
#define MOTORKERNEL_H
#include<memory>
#include<QMap>
#include <QTimer>
#include <QThread>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QCoreApplication>
#include<QDateTime>
#include <QDir>

enum MotorStatus
{
    MOTOR_OPENLOOP,
    MOTOR_CLOSEDLOOP,
    MOTOR_WAIT_READ_SENSOR,
    MOTOR_FREE
};

enum KernelStatus
{
    KERNEL_RUNNING,
    KERNEL_FREE,
    KERNEL_ABORT
};

class Motor;
using MotorMap = QMap<int,Motor*>;
using MotorStatusMap = QMap<int,MotorStatus>;

class MotorKernel : public QObject
{
    Q_OBJECT
public:
    MotorKernel(QString executableDir);
    ~MotorKernel(){}

//    void RefreshMotorList();
    QList<int> GetMotorList();
    void SendHeartBeat(int motorId);
    void RunMotorOnePos(int motorId, double pos, double vel);
    void RunMotorOneCmd(int motorId, QString cmd);
    void RunMotorsHome();
    void EmitDummy(QString message);
    void EmitOpenLoop(int motorId, QList<double> timePoints, QList<double> posPoints, QList<double> velPoints, int real_id=-1);
    void EmitClosedLoop(int motorId, QList<double> timePoints, QList<double> posPoints, int real_id=-1);
    void EmitAbort(int motorId);
    void StopMotor(int id);
    void StopAll();
    void RunMotorClosedLoop(int motorId, QList<double> timePoints, QList<double> posPoints);

    void SetKernelStatus(KernelStatus s){status_ = s;}
    KernelStatus GetKernelStatus(){return status_;}
    double GetPos(int motorId, int real_id=-1);
    double GetPosFast(int motorId, int real_id=-1);

    double GetHome(int id);
    QList<int> GetMotorId(){return motorMap_.keys();}
    void PrintStatus(int id);
    QString PrintStatusShort(int id);
    bool IsAllMotorFree();

private:
    QThread workerThread_;
    MotorMap motorMap_;
    MotorStatusMap motorStatusMap_;
    QSerialPortInfo info;
    QTimer timer_;
    QString executableDir_;
    QDateTime creationTime_;
    QDir logDir_;
    KernelStatus status_;


public slots:
    void RefreshMotorList();

signals:
    void DummySignal(QString message);
    void ReadSensorSignal(int id, int real_id=-1);
    void OpenLoopSignal(int motorId, QList<double> timePoints, QList<double> posPoints, QList<double> velPoints, int real_id=-1);
    void ClosedLoopSignal(int motorId, QList<double> timePoints, QList<double> posPoints, int real_id=-1);
    void AbortSingal(int id);
};



#endif // MOTORKERNEL_H

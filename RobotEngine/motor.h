#ifndef MOTOR_H
#define MOTOR_H
#include <QSerialPort>
#include <QObject>
#include<QList>
#include<memory>
#include <QThread>
#include<QMutex>
#include<QDateTime>
#include <algorithm>
#include<QDir>
enum MotorInternalStatus
{
    MOTOR_RUNNING,
    MOTOR_IDLE,
    MOTOR_READING,
    MOTOR_ABORT
};

enum ControlType
{
    CTLPID,
    CTLPIDONEPOINT,
    CTLFIXEDINPUT,
    CTLFIXEDONEPOINT,
    CTLTWOSPEEDONEPOINT
};

enum MotorType
{
    MTR_NHINC,
    MTR_NHABS,
    MTR_PIEZO,
    MTR_NHABSBIG
};


class Controller
{
public:
    Controller(double p1,double p2,double p3,double p4):p1_(p1),p2_(p2),p3_(p3),p4_(p4){}
    virtual double Law(double e1, double e2, double e3,double e4)=0;
    void print() const
    {
        qDebug() << "Controller Parameters:";
        qDebug() << "p1: " << p1_;
        qDebug() << "p2: " << p2_;
        qDebug() << "p3: " << p3_;
        qDebug() << "p4: " << p4_;
    }
protected:
    double p1_;
    double p2_;
    double p3_;
    double p4_;
};

class ControllerPID : public Controller
{
public:
    ControllerPID(double p1,double p2,double p3,double p4): Controller(p1,p2,p3,p4){}
    virtual double Law(double e1, double e2, double e3,double e4) override
    {
        double output = p1_*e1+p2_*e2+p3_*e3;
        if (std::abs(output) <= std::abs(p4_))
        {
            if (output >= 0) {
                output = p4_ + 0.1; // Adjust this value as needed
            } else {
                output = -p4_ - 0.1; // Adjust this value as needed
            }
        }
        return output;
    }
};

class ControllerFixed : public Controller
{
public:
    ControllerFixed(double p1,double p2,double p3,double p4): Controller(p1,p2,p3,p4){}
    virtual double Law(double e1, double e2, double e3, double e4) override
    {
        return p1_;
    }
};

class Motor: public QObject
{
    Q_OBJECT
public:
    Motor(const QString& portName, int id, const QDir& logDirParrent,ControlType type, Controller* controller);
    ~Motor();
    void execute(double pos, double vel, int real_id=-1);
    void executeCmd(QString cmd);
    void stop();
    void readSensor(int real_id=-1);
    virtual bool heartbeat();

    int getId(){return id_;}
    double getPos(){return pos_;}
    double getVel(){return vel_;}
    MotorInternalStatus getStatue(){return status_;}
    QString getPortName() {return port_->portName();}
    double getHome(){return home_;}
    void setStatus(MotorInternalStatus s){status_=s;}
    void setHome(double home){home_=home;}
    bool isPortOpen(){return port_->isOpen();}
    void print();
    QString printShort(int real_id=-1);
    MotorType getMotorType(){return type_;}

    virtual QByteArray generateCommand(int motorId, QString mode, double angle, double speed, bool query, bool stop);
    virtual double parseSensor(QByteArray dataBA);

private:
    QString controlTypeToString(ControlType type);
    QString motorStatusToString(MotorInternalStatus status);
    QString motorTypeToString(MotorType motor);

public slots:
    void dummy(QString message)
    {
        qDebug()<<"slot"<<message;
    }
    void openLoopTrajectory(int id, const QList<double>& timePoints, const QList<double>& pathPoints, const QList<double>& velPoints, int real_id=-1);
    void colsedLoopTrajectory(int id, QList<double> timePoints, QList<double> pathPoints, int real_id=-1);
    void readSensorSlot(int id, int real_id=-1);
    void stopSlot(int id);
protected:
    int heartbeatLen_=8;
    double home_=0;
    int id_;
    int ratio_;
    MotorInternalStatus status_;
    QString unit_;
    QString name_;
    QString logFileName_;
    QDateTime lastReadTime_;
    QDateTime lastVelTime_;
    double pos_;
    double posVel_;
    double vel_;
    QString portName_;
    QSerialPort* port_;
    QMutex port_mtx_;
    QMutex task_mtx_;
    ControlType control_type_;
    Controller* controller_;
    double margin_;
    MotorType type_=MTR_NHINC;
};

class AbsMotor:public Motor
{
public:
    AbsMotor(const QString& portName, int id, const QDir& logDirParrent,ControlType type,
            Controller* controller)
        : Motor(portName, id, logDirParrent, type,controller) {heartbeatLen_=13;home_=180;type_=MTR_NHABS;margin_=10;}

    QByteArray generateCommand(int motorId, QString mode, double angle, double speed, bool query, bool stop) override ;
    double parseSensor(QByteArray dataBA) override ;
protected:
    QString angleToHexStr(double value);

};

class AbsHeavyMotor:public AbsMotor
{
public:
    AbsHeavyMotor(const QString& portName, int id, const QDir& logDirParrent,ControlType type,
             Controller* controller)
        : AbsMotor(portName, id, logDirParrent, type,controller) {type_=MTR_NHABSBIG;}

    QByteArray generateCommand(int motorId, QString mode, double angle, double speed, bool query, bool stop) override ;
};

class PiezoMotor:public Motor
{
public:
    PiezoMotor(const QString& portName, int id, const QDir& logDirParrent,ControlType type,
               Controller* controller)
        : Motor(portName, id, logDirParrent, type,controller){home_=0;type_=MTR_PIEZO;}

    QByteArray generateCommand(int motorId, QString mode, double angle, double speed, bool query, bool stop) override ;
    double parseSensor(QByteArray dataBA) override ;
    bool heartbeat() override;

};


struct Waypoint
{
    double time;  // Time at which the waypoint should be reached
    double position;  // Desired position at the waypoint
};

class Trajectory
{
public:
    void addWaypoint(double time, double position)
    {
        waypoints_.push_back({time, position});
    }
    double getDesiredPosition(double currentTime);
    double getNextPosition(double currentTime);
    double getDesiredVelocity(double currentTime);

    double getLastTime(){return waypoints_.back().time;}
    double getLastPos(){return waypoints_.back().position;}
private:
    QList<Waypoint> waypoints_;
};



#endif // MOTOR_H

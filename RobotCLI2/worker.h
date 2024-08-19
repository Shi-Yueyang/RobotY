#ifndef WORKER_H
#define WORKER_H

#include <QObject>
#include <QDebug>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QThread>
#include <QMap>
#include "MotorKernel.h"
#include "RobotEngine.h"

class Worker : public QObject
{
    Q_OBJECT
public:
    Worker(QString exe_dir);
    ~Worker();

public slots:
    void scan_motor_conf();
    void run_motors_open();
    void run_one_motor_one_pos_open(int id, double pos, double vel, int real_id);
    void run_one_motor_one_pos_closed(int id, double pos, double duration, int real_id);
    void run_motors_close(double duration);
    void run_motors_home(double duration);
    void run_one_motor_close(int id, double target, double duration);

    void printStatus();
    void printStatus(int id, int real_id=-1);
    void printRobotAxis();
    void printStatusLong();
    void printMap(const QMap<int, double>& map, QString mapname);
    void printHelp();
    void run_robot(double x1, double y1, double z1, double x2, double y2, double z2,QString order);
    void send_cmd(int id, QString cmd);
    void stop()
    {
        stopFlag_=true;
        kernel_->StopAll();
    }
    QMap<QString,double> robotIK2P(double x1, double y1, double z1, double x2, double y2, double z2)
    {
        return robot_->robotIK2P(x1,y1,z1,x2,y2,z2);
    }
    void set_open_vel(int id,double vel)
    {
        openVel_[id] = vel;
    }
    void set_pos(int id,double vel)
    {
        targets_[id]=vel;
        hasJob_.push_back(id);
    }
    bool hasMotor(int id)
    {
        return targets_.contains(id);
    }
signals:
    void scan_motor_conf_request();
    void run_motors_open_request();
    void run_motors_close_request(double duration);
    void run_one_motor_close_request(int id, double target, double duration);
    void printStatus_request();
    void printStatusLong_request();
    void printMap_request(const QMap<int, double>& map, QString mapname);
    void printHelp_request();
    void run_robot_request(double x1, double y1, double z1, double x2, double y2, double z2,QString order);
    void run_one_motor_one_pos_open_request(int id, double pos, double vel, int real_id=-1);
    void run_one_motor_one_pos_closed_request(int id, double pos, double vel, int real_id=-1);

    void run_motors_home_request(double duration);
    void send_cmd_request(int id, QString cmd);

private:
    QString exe_dir_;
    MotorKernel* kernel_;
    RobotEngine* robot_;
    QMap<int,double> targets_;
    QMap<int,double> openVel_;
    QVector<int> hasJob_;
    bool stopFlag_=false;

};

#endif // WORKER_H

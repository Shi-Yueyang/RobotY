#include "motor.h"
#include "utils.h"
#include <QDebug>
#include <thread>
#include <algorithm>
#include <cmath>

Motor::Motor(const QString& portName, int id, const QDir& logDirParrent,ControlType type,
             Controller* controller)
    :
    id_(id),
    status_(MOTOR_IDLE),
    pos_(0),
    vel_(0),
    control_type_(type),
    controller_(controller),
    port_(new QSerialPort(this))
//    task_(new QThread(this))
{

    setObjectName(QString("motor%1").arg(id));
    if(portName!="none")
    {
        // Set port properties
        port_->setPortName(portName);
        port_->setBaudRate(QSerialPort::Baud115200);
        port_->setDataBits(QSerialPort::Data8);
        port_->setParity(QSerialPort::NoParity);
        port_->setStopBits(QSerialPort::OneStop);

        // Retry opening the port
        int retries = 3;
        while (retries > 0 && !port_->open(QIODevice::ReadWrite)) {
            qDebug() << "Motor: Can't open port" << port_->portName()
                     << "Retries left:" << retries;
            QThread::msleep(500);
            --retries;
        }

        if (!port_->isOpen())
        {
            qDebug() << "Motor: Failed to open port" << port_->portName();
        }
    }
    // set time
    lastReadTime_ = QDateTime::currentDateTime();
    lastVelTime_ = QDateTime::currentDateTime();

    // Set logfile
    logFileName_ = logDirParrent.filePath(QString("m-%1.csv").arg(id_));

}

Motor::~Motor()
{
    // close port
    if (port_ && port_->isOpen()) {
        port_->close();
    }

    // Delete
    delete port_;
    delete controller_;
}

void Motor::print()
{
    qDebug()<<"--------------------------";
    qDebug() << "Motor ID: " << id_;
    qDebug() << "type: " << motorTypeToString(type_);
    qDebug() << "Port Name: " << port_->portName();
    qDebug() << "Status: " << motorStatusToString(status_);
    qDebug() << "Position: " << pos_;
    qDebug() << "Velocity: " << vel_;
    qDebug() << "Control Type: " << controlTypeToString(control_type_);
    if (controller_)
    {
        controller_->print();
    }
}

QString Motor::printShort(int real_id)
{
    QString logtxt=real_id==-1 ? QString("id[%1], pos[%2], state[%3]").arg(id_).arg(pos_).arg(motorStatusToString(status_))
                                   : QString("id[%1], pos[%2], state[%3]").arg(real_id).arg(pos_).arg(motorStatusToString(status_));
    qDebug()<<logtxt;
    return logtxt;
}

QByteArray Motor::generateCommand(int motorId, QString mode, double angle, double speed, bool query, bool stop)
{
    QString idstr = QString::number(motorId);
    if(stop)
    {
        QString command = idstr+" 53 00 00 00 00 00 0A";
        //        qDebug()<<"generating cmd: "<<command;
        return toByteArray(command);
    }

    if(query)
    {
        QString command = idstr+" 51 00 00 00 00 00 0A";
        //        qDebug()<<QString("[%1] cmd: %2").arg(QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss.zzz")).arg(command);

        return toByteArray(command);
    }
    // mode
    QString modestr;
    if (mode == "continues")
    {
        modestr = "02";
    }
    else
    {
        modestr = "03";
    }
    // target
    int turns = static_cast<int>(angle / 360)+0x8000;
    if(angle<0 && angle/360!=0)
    {
        int relative_turn = -static_cast<int>(angle / 360)+1;
        turns -= 1;
        angle += relative_turn*360;
    }
    double remainingAngle = fmod(angle, 360.0);
    QString turnsHexString = QString("%1").arg(turns, 4, 16, QChar('0'));
    turnsHexString = turnsHexString.right(4);
    int tmp = static_cast<int>((remainingAngle / 360.0) * 4000);
    QString remainingAngleString = QString("%1").arg(tmp, 4, 10, QChar('0'));
    QString targetString = turnsHexString.mid(0,2)+" "+turnsHexString.mid(2,2)
                           +" "+remainingAngleString.mid(0,2)+" "+remainingAngleString.mid(2,2);

    int hexSpeed = static_cast<int>((speed / 100.0) * 255);
    QString speedString = QString("%1").arg(hexSpeed, 2, 16, QChar('0')).toUpper();
    QString Command=idstr+" "+modestr+" "+targetString+" "+speedString+" 0A";

    //    qDebug()<<QString("[%1] cmd: %2").arg(QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss.zzz")).arg(Command);

    return toByteArray(Command);
}

double Motor::parseSensor(QByteArray dataBA)
{
    if(dataBA.size()<heartbeatLen_)
    {
        return 0;
    }
    QVector<uint8_t> data;
    for (int i = 0; i < dataBA.size(); ++i)
    {
        int character = static_cast<uint8_t>(dataBA.at(i));
        data.append(character);
    }
    QString message;
    for (int i = 0; i < data.size(); ++i)
    {
        if (i > 0)
        {
            message += " ";  // Add space between elements
        }
        message += QString::number(data.at(i),16).rightJustified(2, '0');;
    }

    QStringList hexNumbers = message.split(" ");
    double angleDouble = 0;

    QString turn1 = hexNumbers[2];
    QString turn2 = hexNumbers[3];
    QString turn = turn1+turn2;
    double turnInt = turn.toInt(nullptr,16)-0x8000;

    QString angle1 = hexNumbers[4];
    QString angle2 = hexNumbers[5];
    QString angle = angle1+angle2;
    angleDouble = angle.toDouble()/4000.0*360.0 + 360.0*turnInt;
    return angleDouble;
}
QString AbsMotor::angleToHexStr(double value)
{
    // process value
    if(value<0) value = -value;
    if(value>(360-margin_)) value = 350;
    if(value<margin_) value = 10;
    value = fmod(value, 360.0);

    // Convert value to integer in the range [0, 03FFFF]
    int intValue = static_cast<int>((value / 360.0) * 0x3FFFF);
    if(intValue==0)
    {
        intValue+=1;
    }
    // Convert integer to hexadecimal string with leading zeros
    QString hexString = QString("%1").arg(intValue, 6, 16, QChar('0')).toUpper();

    return hexString;
}
QByteArray AbsMotor::generateCommand(int motorId, QString mode, double angle, double speed, bool query, bool stop)
{
    QString idstr = QString::number(motorId);
    if(stop)
    {
        QString command = idstr+" 53 00 00 00 00 00 0A";
        return toByteArray(command);
    }

    if(query)
    {
        QString command = idstr+" 51 00 00 00 00 00 0A";
        return toByteArray(command);
    }

    // mode
    QString modestr;
    if (mode == "continues")
    {
        modestr = "02";
    }
    else
    {
        modestr = "03";
    }

    // target
    angle += home_;
    QString targetString = angleToHexStr(angle);
    QString targetStringWithSpace = targetString.mid(0,2)+" "+targetString.mid(2,2)+" "+targetString.mid(4,2);

    // speed
    int hexSpeed = static_cast<int>((speed / 100.0) * 255);
    QString speedString = QString("%1").arg(hexSpeed, 2, 16, QChar('0')).toUpper();
    QString impulseString = "80";

    // get command
    QString Command = idstr+" "+modestr+" "+targetStringWithSpace+" "+impulseString+" "+speedString+" 0A";
    return toByteArray(Command);
}


double AbsMotor::parseSensor(QByteArray dataBA)
{
    if(dataBA.size()<heartbeatLen_)
    {
        return 0;
    }
    QVector<uint8_t> data;
    for (int i = 0; i < dataBA.size(); ++i)
    {
        int character = static_cast<uint8_t>(dataBA.at(i));
        data.append(character);
    }
    QString message;
    for (int i = 0; i < data.size(); ++i)
    {
        if (i > 0)
        {
            message += " ";  // Add space between elements
        }
        message += QString::number(data.at(i),16).rightJustified(2, '0');;
    }
    QStringList hexNumbers = message.split(" ");
    QString angle = hexNumbers[6]+hexNumbers[7]+hexNumbers[8];
    bool ok;
    int intAngle = angle.toInt(&ok,16);
    double angleDouble = static_cast<double>(intAngle) / 0x3FFFF * 360.0;  // 0x3FFFF = 262143
    angleDouble -= home_;
    return angleDouble;
}

QByteArray AbsHeavyMotor::generateCommand(int motorId, QString mode, double angle, double speed, bool query, bool stop)
{
    QString idstr = QString::number(motorId);
    if(stop)
    {
        QString command = idstr+" 53 00 00 00 00 00 0A";
        return toByteArray(command);
    }

    if(query)
    {
        QString command = idstr+" 51 00 00 00 00 00 0A";
        return toByteArray(command);
    }

    // mode
    QString modestr = "03";
    // target


    // target
    angle += home_ ;
    QString targetString = angleToHexStr(angle);
    QString targetStringWithSpace = targetString.mid(0,2)+" "+targetString.mid(2,2)+" "+targetString.mid(4,2);

    // speed
    int hexSpeed = static_cast<int>((speed / 100.0) * 255);
    QString speedString = QString("%1").arg(hexSpeed, 2, 16, QChar('0')).toUpper();
    QString impulseString = "80";

    // get command
    QString Command = idstr+" "+modestr+" "+targetStringWithSpace+" "+speedString+" "+impulseString+" 0A";
    return toByteArray(Command);
}

QByteArray PiezoMotor::generateCommand(int motorId, QString mode, double angle, double speed, bool query, bool stop)
{
    QByteArray ans;
    if(stop)
    {
        return QString("X%1S;").arg(motorId).toUtf8();
    }
    if(query)
    {
        return QString("X%1E\r").arg((motorId)).toUtf8();
    }
    int angle2 = (int)(angle/180* 8192);
    int speed2 = static_cast<int>((speed / 100.0) * 1500);
    ans =QString("X%1T%2,%3\r").arg(motorId).arg(angle2).arg(speed2).toUtf8();
    return ans;
}

double PiezoMotor::parseSensor(QByteArray dataBA)
{
    QString str(dataBA);
    QStringList parts = str.split(":");

    if (parts.size() >= 2)
    {
        QString someNumberStr = parts.at(1).trimmed(); // Extract the second part and remove leading/trailing whitespaces
        double someNumber = someNumberStr.toDouble();
        double angle = someNumber*180/8192;
        return angle;
    }
    return 0;
}

bool PiezoMotor::heartbeat()
{
    QByteArray cmd = generateCommand(id_,"continues",0,0,true,false);
    QMutexLocker locker(&port_mtx_);
    port_->clear();
    if(port_->isOpen())
    {
        port_->write(cmd);
        port_->waitForBytesWritten(100);
        if(port_->waitForReadyRead(100))
        {
            QByteArray response = port_->readAll();
            qDebug() << "Response from" << port_->portName() << " len:" << response.size();
            if (response.at(0) == 'X')
            {
                qDebug()<<"receive heart beat piezo";
                QFile logFile(logFileName_);
                if (!logFile.exists() && logFile.open(QIODevice::Append | QIODevice::Text))
                {
                    QTextStream textStream(&logFile);
                    textStream << "time,id,pos,val,target,target_vel,err,err_next,err_int,err_diff,control\n";
                    logFile.close();
                }
                else
                {
                    qDebug() << "Error opening log file:" << logFile.errorString();
                }

                // init command example
                // X[id]E0;X[id]Y25,1;X[id]Y3,-1638400;;X[id]Y4,1638400;
                qDebug()<<"piezo init";
                port_->clear();
                QString cmd_str = QString("X%1Y25,1;X%1Y3,-1638400;X%1Y4,1638400;").arg(id_);
                QByteArray cmd = cmd_str.toUtf8();
                qDebug()<<cmd;
                port_->write(cmd);

                cmd_str = QString("X%1Y25,1;X%1Y3,-1638400;X%1Y4,1638400;").arg(id_+1);
                cmd = cmd_str.toUtf8();
                qDebug()<<cmd;
                port_->write(cmd);
                return true;
            }
        }
    }
    return false;
}

bool Motor::heartbeat()
{
    QByteArray cmd = generateCommand(id_,"continues",0,0,true,false);
    QMutexLocker locker(&port_mtx_);
    port_->clear();
    if(port_->isOpen())
    {
        port_->write(cmd);
        port_->waitForBytesWritten(100);
        if(port_->waitForReadyRead(100))
        {
            QByteArray response = port_->readAll();
            qDebug() << "Response from" << port_->portName() << " len:" << response.size();

            // create a Motor if receiving response
            if (response.size()==heartbeatLen_)
            {
                qDebug()<<"receive heart beat";
                QFile logFile(logFileName_);
                if (!logFile.exists() && logFile.open(QIODevice::Append | QIODevice::Text))
                {
                    QTextStream textStream(&logFile);
                    textStream << "time,id,pos,val,target,target_vel,err,err_next,err_int,err_diff,control\n";
                    logFile.close();
                } else
                {
                    qDebug() << "Error opening log file:" << logFile.errorString();
                }
                return true;
            }
        }
    }
    return false;
}

void Motor::execute(double pos, double vel, int real_id)
{
    if(vel<0)
    {
        vel = -vel;
        qDebug("vel is <0, vel=%f",vel);
    }

    int id_plus = vel>100?vel/100:0;
    vel = vel - id_plus*100;

    QByteArray cmd = real_id==-1? generateCommand(id_+id_plus,"continues",pos,vel,false,false):generateCommand(real_id+id_plus,"continues",pos,vel,false,false);
    QMutexLocker locker(&port_mtx_);
    port_->clear();
    if(port_->isOpen())
    {
        port_->write(cmd);
        port_->waitForBytesWritten(100);
    }
}

void Motor::executeCmd(QString cmd)
{
    QByteArray cmd_ascii = cmd.toUtf8();
    QMutexLocker locker(&port_mtx_);
    port_->clear();
    if(port_->isOpen())
    {
        port_->write(cmd_ascii);

    }
}


void Motor::stop()
{
    QByteArray cmd = generateCommand(id_,"continues",0,0,false,true);
    QMutexLocker locker(&port_mtx_);
    port_->clear();
    if(port_->isOpen())
    {
        port_->write(cmd);
        port_->waitForBytesWritten(100);
    }
}

void Motor::readSensor(int real_id)
{
    // send query
    QByteArray cmd = real_id==-1? generateCommand(id_,"continues",0,0,true,false):generateCommand(real_id,"continues",0,0,true,false);
    QMutexLocker locker(&port_mtx_);
    if(port_->isOpen())
    {
        port_->write(cmd);
        port_->waitForBytesWritten(100);
    }


    // parse income
    if(port_->waitForReadyRead(100))
    {
        // get time
        QDateTime currTime = QDateTime::currentDateTime();
        double elapsedTime = lastReadTime_.msecsTo(currTime)/1000.0; // in sec

        // parse reponse
        QByteArray response =port_->readAll();
        double angle = parseSensor(response);


        vel_=(angle-posVel_)/elapsedTime; // deg/sec

        lastVelTime_=currTime;
        posVel_=angle;


        // update pos
        lastReadTime_=currTime;
        pos_=angle;
    }
    else
    {
        qDebug()<<"no data";
    }
}

double Trajectory::getDesiredPosition(double currentTime)
{
    if (waypoints_.empty())
    {
        return 0.0;  // Default position if no waypoints are defined
    }

    // Use std::lower_bound for binary search
    auto it = std::lower_bound(waypoints_.begin(), waypoints_.end(), currentTime,
                               [](const Waypoint& wp, double time) {
                                   return wp.time < time;
                               });

    // Check if the iterator is at the beginning or end of the vector
    if (it == waypoints_.begin()) {
        return waypoints_.front().position;
    } else if (it == waypoints_.end()) {
        return waypoints_.back().position;
    }

    // Perform linear interpolation
    Waypoint wp1 = *(it - 1);
    Waypoint wp2 = *it;

    double t = (currentTime - wp1.time) / (wp2.time - wp1.time);
    double desiredPosition = wp1.position + t * (wp2.position - wp1.position);

    return desiredPosition;
}

double Trajectory::getNextPosition(double currentTime)
{
    if (waypoints_.empty())
    {
        return 0.0;
    }
    if(currentTime>getLastTime())
    {
        return getLastPos();
    }
    auto it = std::lower_bound(waypoints_.begin(), waypoints_.end(), currentTime,
                       [](const Waypoint& wp, double time) {
                           return wp.time < time;
                       });
    if (it == waypoints_.end()) {
        return (*(it - 1)).position;
    }
    return (*it).position;
}

double Trajectory::getDesiredVelocity(double currentTime)
{
    if (waypoints_.empty())
    {
        return 0.0;
    }

    // Use std::lower_bound for binary search
    auto it = std::lower_bound(waypoints_.begin(), waypoints_.end(), currentTime,
                               [](const Waypoint& wp, double time) {
                                   return wp.time < time;
                               });

    // Check if the iterator is at the beginning or end of the vector
    if (it == waypoints_.begin() || it == waypoints_.end())
    {
        return 0;
    }

    // Perform linear interpolation
    Waypoint wp1 = *(it - 1);
    Waypoint wp2 = *it;

    return (wp2.position-wp1.position)/(wp2.time-wp1.time);
}


void Motor::openLoopTrajectory(int id, const QList<double> &timePoints, const QList<double> &pathPoints, const QList<double> &velPoints, int real_id)
{
    if(id!=id_)
    {
        return;
    }
    // open log file
    QFile logFile(logFileName_);
    if (!logFile.open(QIODevice::Append | QIODevice::Text))
    {
        qDebug() << "Error opening log file:" << logFile.errorString();
        return;
    }

    // run the motor
    qDebug()<<"motor["<<id_<<"] start OpenLoopTrajectory";
    setStatus(MOTOR_RUNNING);

    QMutexLocker locker(&task_mtx_);
    int n=timePoints.size();
    for (int i = 0; i < n; ++i)
    {
        if(getStatue()==MOTOR_ABORT)
        {
            qDebug()<<QString("motor[%1] abort mission").arg(id_);
            break;
        }
        double elapsedTime = i<n-1 ? timePoints.at(i+1)-timePoints.at(i):0;
        double pos = pathPoints.at(i);
        double vel = velPoints.at(i);
        execute(pos,vel,real_id);
        readSensor(real_id);

        //log to file
        QTextStream textStream(&logFile);
        QString logStr=QString("%1,%2,%3,%4,%5,%6\n")
             .arg(QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss.zzz"))
             .arg(id_).arg(pos_).arg(vel_).arg(pos).arg(pos-pos_);
        textStream <<logStr;
        QThread::msleep(static_cast<unsigned>(elapsedTime * 1000));
    }
    setStatus(MOTOR_IDLE);
    if(timePoints.size()>1)
    {
        qDebug()<<"motor["<<id_<<"] finish OpenLoopTrajectory";
    }
    logFile.close();
}

void Motor::readSensorSlot(int id, int real_id)
{
    if(id!=id_)
    {
        return;
    }
    readSensor(real_id);
    if(status_==MOTOR_READING)
    {
        setStatus(MOTOR_IDLE);
    }

}

void Motor::stopSlot(int id)
{
    if(id!=id_)
    {
        return;
    }
    stop();
}

void Motor::colsedLoopTrajectory(int id, QList<double> timePoints,  QList<double> pathPoints,int real_id)
{
    if(id!=id_)
    {
        return;
    }
    readSensor(real_id);

    // create trajectory
    if(control_type_==CTLPIDONEPOINT)
    {
        if (std::abs(pos_ - pathPoints[1]) < 0.15)
        {
            return;
        }
        pathPoints[0] = pos_;
        timePoints[1] = std::max(1.0,timePoints[1]);
    }
    Trajectory traj;
    for(int i=0;i<timePoints.size();i++)
    {
        traj.addWaypoint(timePoints[i],pathPoints[i]);
    }

    // open log file
    QFile logFile(logFileName_);
    if (!logFile.open(QIODevice::Append | QIODevice::Text))
    {
        qDebug() << "Error opening log file:" << logFile.errorString();
        return;
    }
    QTextStream textStream(&logFile);

    // print time and path points
    textStream << "time points;";
    for (int i = 0; i < timePoints.size(); ++i)
    {
        textStream << timePoints[i];
        if (i < timePoints.size() - 1) textStream << ";";
    }
    textStream << "\n";
    textStream << "pos points;";
    for (int i = 0; i < pathPoints.size(); ++i)
    {
        textStream << pathPoints[i];
        if (i < pathPoints.size() - 1) textStream << ";";
    }
    textStream << "\n";

    // states
    double breakMargin=0.15;
    double errIntegral=0;
    double errDiff=0;
    double err=0;
    double err_next=0;
    double prevErr=0;
    double control=0;
    int enterLowSpeedCnt=0;
    bool enterLowSpeed=false;
    bool readyToLeave=false;
    QDateTime LowSpeedTimeStart;
    QDateTime ReadyToLeaveTime;
    QDateTime startTime = QDateTime::currentDateTime();
    QDateTime prevTime = QDateTime::currentDateTime();


    // run motor
    qDebug()<<"motor["<<id_<<"] start closedloop Trajectory";
    setStatus(MOTOR_RUNNING);


    QMutexLocker locker(&task_mtx_);
    while(true)
    {
        if(getStatue()==MOTOR_ABORT)
        {
            qDebug()<<QString("motor[%1] abort mission").arg(id_);
            break;
        }

        // time data
        QDateTime currentTime = QDateTime::currentDateTime();
        double deltaTime = prevTime.msecsTo(currentTime)/1000.0;
        if(deltaTime<0.01) continue;
        prevTime=currentTime;
        double relative_time=startTime.msecsTo(currentTime)/1000.0;

        // control state
        readSensor(real_id);
        QString logStr;
        double targetPos=traj.getDesiredPosition(relative_time);
        err= getPos() - targetPos;
        double nextPos = traj.getNextPosition(relative_time);
        double targetVel=traj.getDesiredVelocity(relative_time)/6.0; // rpm
        err_next = nextPos - getPos();
        if(type_==MTR_NHABS || type_==MTR_NHABSBIG)
        {
            targetPos = qBound(margin_-home_, targetPos, 360 - margin_-home_);
            nextPos = qBound(margin_-home_, nextPos, 360 - margin_-home_);
        }

        // break when reach
        if(relative_time>traj.getLastTime() && std::abs(err)<=breakMargin)
        {
            enterLowSpeedCnt = 0;
            if(!readyToLeave)
            {
                readyToLeave=true;
                ReadyToLeaveTime=QDateTime::currentDateTime();
                textStream << "entering readyToLeave err=" << QString::number(std::abs(err), 'f', 2) << "\n";
            }
            else
            {
                double waitTime = ReadyToLeaveTime.secsTo(currentTime);
                if(waitTime>2)
                {
                    double extratime = relative_time-traj.getLastTime();

                    break;
                }
            }
        }
        else
        {
            readyToLeave=false;
        }

        // enter low speed
        if(relative_time>traj.getLastTime() && std::abs(vel_)<3)
        {
            if(!enterLowSpeed)
            {
                enterLowSpeed=true;
                LowSpeedTimeStart = QDateTime::currentDateTime();
                enterLowSpeedCnt++;
                if(enterLowSpeed>5)
                {
                    break;
                }
                textStream << "entering low speed vel_=" << QString::number(std::abs(vel_), 'f', 2) << "\n";
            }
            else
            {
                double timeAfterLowSpeed = LowSpeedTimeStart.secsTo(currentTime);
                if(timeAfterLowSpeed>5)
                {
                    double extratime = relative_time-traj.getLastTime();
                    qDebug()<<"not reach, extra "<<extratime<<" s";
                    qDebug()<<"pos="<<pos_<<" vel="<<vel_;
                    break;
                }
            }
        }

        // don't break if run fast, not in low speed
        if(relative_time>traj.getLastTime() && std::abs(vel_)>3)
        {
            enterLowSpeed=false;
            enterLowSpeedCnt=0;
        }

        int direction=0;
        switch(control_type_)
        {
        case CTLPID:
            // update states
            errIntegral += deltaTime*err;
            errDiff=(err-prevErr)/deltaTime;
            prevErr = err;
            prevErr=err;
            control = controller_->Law(err,errIntegral,errDiff,targetVel);

            // reach next pos --> stop
            if(abs(err_next)<=breakMargin)
            {
                control = 0;
            }
            // reach target pos --> can't stop
            else if(targetVel!=0 && control==0)
            {
                control=1;
            }
            control = std::min(std::abs(control),100.0);

            execute(targetPos,std::abs(control),real_id);
            // do logging
            logStr=QString("%1,%2,%3,%4,%5,%6,%7,%8,%9,%10,%11\n")
                 .arg(QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss.zzz"))
                         .arg(id_).arg(pos_).arg(vel_).arg(targetPos).arg(targetVel)
                         .arg(err).arg(err_next).arg(errIntegral).arg(errDiff).arg(control);

            break;
        case CTLPIDONEPOINT:
            // update states
            direction = pathPoints[1]-pathPoints[0]>=0?1:-1;
            errIntegral += deltaTime*err;
            errDiff=(err-prevErr)/deltaTime;
            prevErr = err;
            prevErr=err;
            control = controller_->Law(err,errIntegral,errDiff,targetVel);

            if(enterLowSpeedCnt>=3)
            {
                control = std::abs(control) + 25;
            }
            control = std::min(std::abs(control),100.0);

            // adjustment
            // over reach
            if(direction*(err)>0 && control!=0)
            {
                control = 1;
            }
            // reach target pos --> can't stop
            else if(targetVel!=0 && control==0)
            {
                control=1;
            }
            // reach next pos --> stop
            if(abs(err_next)<=breakMargin)
            {
                control = 0;
            }
            if(control!=0)
            {
                execute(nextPos,control,real_id);

                // do logging
                logStr=QString("%1,%2,%3,%4,%5,%6,%7,%8,%9,%10,%11\n")
                             .arg(QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss.zzz"))
                             .arg(id_).arg(pos_).arg(vel_).arg(targetPos).arg(targetVel)
                             .arg(err).arg(err_next).arg(errIntegral).arg(errDiff).arg(control);
            }
            break;
        case CTLFIXEDINPUT:
        case CTLFIXEDONEPOINT:
            control = controller_->Law(err,errIntegral,errDiff,targetVel);
            control = std::min(std::abs(control),100.0);
            if(abs(err_next)<=breakMargin)
            {
                control = 0;
            }

            if(control!=0)
            {
                execute(nextPos,std::abs(control),real_id);
                logStr=QString("%1,%2,%3,%4,%5,%6,%7,%8,%9,%10,%11\n")
                             .arg(QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss.zzz"))
                             .arg(id_).arg(pos_).arg(vel_).arg(targetPos).arg(targetVel)
                             .arg(err).arg(err_next).arg(0).arg(0).arg(control);
            }
            break;
        case CTLTWOSPEEDONEPOINT:
            execute(nextPos,std::abs(10),real_id);
            execute(nextPos,std::abs(70),real_id);
            break;
        default:
            break;
        }
        if(control_type_==CTLFIXEDONEPOINT)
            break;
        if(logStr!="")
        {
            textStream <<logStr;
        }
    }
    if(control_type_!=CTLFIXEDONEPOINT)stop();
    setStatus(MOTOR_IDLE);
    qDebug()<<"motor["<<id_<<"] finish closedloop Trajectory, err="<<err_next;
    textStream<<"motor["<<id_<<"] finish closedloop Trajectory\n\n";
    logFile.close();
}

QString Motor::controlTypeToString(ControlType type)
{
    switch(type)
    {
    case CTLPID:
        return "PID";
    case CTLPIDONEPOINT:
        return "PID One Point";
    case CTLFIXEDINPUT:
        return "Fixed Input";
    case CTLFIXEDONEPOINT:
        return "Fixed Input one point";
    default:
        return "Unknown";
    }
}

QString Motor::motorStatusToString(MotorInternalStatus status)
{
    switch(status)
    {
    case MOTOR_RUNNING:
        return "Running";
    case MOTOR_IDLE:
        return "idle";
    case MOTOR_ABORT:
        return "Aborted";
    case MOTOR_READING:
        return "reading";
    default:
        return "Unknown";
    }
}

QString Motor::motorTypeToString(MotorType motor)
{
    switch (motor)
    {
        case MTR_NHINC: return "Nanhang Incremental";
        case MTR_NHABS: return "Nanhang Absolute";
        case MTR_PIEZO: return "Piezo";
        case MTR_NHABSBIG: return "Nanhang Absolute reduce";
        default: return "Unknown";
    }
}



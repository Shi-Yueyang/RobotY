#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QCoreApplication>
#include <QFile>
#include<QList>
#include <QDebug>
#include "motor.h"
#include "motorkernel.h"
#include "utils.h"

MotorKernel::MotorKernel(QString executableDir)
    :executableDir_(executableDir),creationTime_(QDateTime::currentDateTime())
{
    // log dir
    logDir_=(QCoreApplication::applicationDirPath() + QDir::separator() + "log"+QDir::separator()+QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss"));
    if (!logDir_.exists())
    {
        logDir_.mkpath(".");
    }
}

QList<int> MotorKernel::GetMotorList()
{
    return motorMap_.keys();
}

double MotorKernel::GetPos(int id, int real_id)
{
    if(!motorMap_.contains(id))
    {
        qDebug()<<"motor "<<id<<"GetPos: doesn't exist";
        return -1;
    }
    if(motorMap_.value(id)->getStatue()==MOTOR_RUNNING)
    {
        return motorMap_[id]->getPos();
    }
    if(motorMap_.value(id)->getStatue()==MOTOR_IDLE || motorMap_.value(id)->getStatue()==MOTOR_ABORT)
    {
        motorMap_.value(id)->setStatus(MOTOR_READING);
    }
    emit ReadSensorSignal(id,real_id);
    while(true)
    {
        if(motorMap_.value(id)->getStatue()!=MOTOR_READING)
        {
            break;
        }
        QThread::msleep(5);
    }

    return motorMap_[id]->getPos();
}

double MotorKernel::GetPosFast(int id, int real_id)
{
    if(!motorMap_.contains(id))
    {
        qDebug()<<"motor "<<id<<"GetPos: doesn't exist";
        return -1;
    }
    if(motorMap_.value(id)->getStatue()==MOTOR_RUNNING)
    {
        return motorMap_[id]->getPos();
    }
    else if(motorMap_.value(id)->getStatue()==MOTOR_IDLE || motorMap_.value(id)->getStatue()==MOTOR_ABORT)
    {
        motorMap_.value(id)->setStatus(MOTOR_READING);
        emit ReadSensorSignal(id,real_id);
    }
    return motorMap_[id]->getPos();
}

double MotorKernel::GetHome(int id)
{
    return motorMap_.value(id)->getHome();
}

void MotorKernel::SendHeartBeat(int motorId)
{
    if(!motorMap_.contains(motorId))
    {
        qDebug()<<"motor "<<motorId<<"GetPos: doesn't exist";
        return;
    }
    motorMap_[motorId]->heartbeat();
}

// emit motors
void MotorKernel::EmitDummy(QString message)
{
    emit DummySignal(message); // all motors will respond
}

void MotorKernel::EmitOpenLoop(int motorId, QList<double> timePoints, QList<double> posPoints, QList<double> velPoints, int real_id)
{
//    qDebug()<<"EmitOpenLoop motorid="<<motorId;
    emit OpenLoopSignal(motorId,timePoints,posPoints,velPoints,real_id);
}

void MotorKernel::EmitClosedLoop(int motorId, QList<double> timePoints, QList<double> posPoints, int real_id)
{
    emit ClosedLoopSignal(motorId,timePoints,posPoints,real_id);
}

void MotorKernel::RunMotorClosedLoop(int motorId, QList<double> timePoints, QList<double> posPoints)
{
    status_ = KERNEL_RUNNING;
    emit ClosedLoopSignal(motorId,timePoints,posPoints);
    while(!IsAllMotorFree() && status_!=KERNEL_ABORT)
    {
        QThread::msleep(50);
    }
    status_ = KERNEL_FREE;
}

void MotorKernel::EmitAbort(int motorId)
{
    emit AbortSingal(motorId);
}

void MotorKernel::RunMotorOnePos(int motorId, double pos, double vel)
{
    if(!motorMap_.contains(motorId))
    {
        return;
    }
    motorMap_[motorId]->execute(pos,vel);
}

void MotorKernel::RunMotorOneCmd(int motorId, QString cmd)
{
    if(!motorMap_.contains(motorId))
    {
        return;
    }
    motorMap_[motorId]->executeCmd(cmd);
}


void MotorKernel::RunMotorsHome()
{
    for(auto it=motorMap_.begin();it!=motorMap_.end();++it)
    {
        it.value()->execute(it.value()->getHome(),50);
    }
}

bool MotorKernel::IsAllMotorFree()
{
    for(auto it = motorMap_.begin();it!=motorMap_.constEnd();++it)
    {
        auto state = it.value()->getStatue();
        if(state!=MOTOR_IDLE && state!=MOTOR_ABORT)
        {
            return false;
        }
    }
    return true;

}

void MotorKernel::PrintStatus(int id)
{
    motorMap_.value(id)->print();
}

QString MotorKernel::PrintStatusShort(int id)
{
    return motorMap_.value(id)->printShort();
}


void MotorKernel::RefreshMotorList()
{


    // clean motors
    QMap<int,QString> CachedPortMap;
    for (auto it = motorMap_.begin(); it != motorMap_.end(); ++it)
    {
        delete it.value();
    }
    motorMap_.clear();
    QThread::sleep(1);
    QList<QSerialPortInfo> avaliablePorts = info.availablePorts();

    //load motorinfo
    QString filePath = executableDir_+ "/motorinfo.json";
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        qDebug() << "Failed to open the motorinfo file under "<<filePath;
        return ;
    }
    QByteArray jsonData = file.readAll();
    file.close();
    QJsonDocument jsonDoc = QJsonDocument::fromJson(jsonData);

    // parse jsonDoc into array
    QJsonObject rootObj = jsonDoc.object();
    QJsonArray motorArray = rootObj["motor_array"].toArray();
    QJsonObject axisMap = rootObj["axis_map"].toObject();

    // Iterate through each object in conf file
    QSet<QString> badports;
    for (const auto& json_motor : motorArray)
    {
        // motor information
        int id = json_motor.toObject()["id"].toInt();
        QString type = json_motor.toObject()["type"].toString();
        QString controller_type_str = json_motor.toObject()["control_type"].toString();

        // build controller
        QJsonObject controllerObject = json_motor.toObject()["control_params"].toObject();
        double p1,p2,p3,p4;
        p1 = controllerObject["p1"].toDouble();
        p2 = controllerObject["p2"].toDouble();
        p3 = controllerObject["p3"].toDouble();
        p4 = controllerObject["p4"].toDouble();

        qDebug()<<"trying to connect to motor"<<id;


        // scan all ports
        for(auto& portinfo:avaliablePorts)
        {
            QString name = portinfo.portName();
            // skip fake COM
            if(portinfo.description()=="蓝牙链接上的标准串行" || portinfo.description()=="" ||badports.contains(portinfo.portName())) continue;

            // skip used COM
            bool portExist=false;
            for(auto it=motorMap_.begin();it!=motorMap_.end();++it)
            {
                if(it.value()->getPortName()==portinfo.portName())
                {
                    portExist=true;
                    break;
                }
            }
            if(portExist) continue;

            // create controller and motor
            Controller* controller;
            ControlType control_type;
            if(controller_type_str=="pid"){controller = new ControllerPID(p1,p2,p3,p4);control_type = CTLPID;}
            else if(controller_type_str=="pidonepoint"){controller = new ControllerPID(p1,p2,p3,p4);control_type = CTLPIDONEPOINT;}
            else if(controller_type_str=="fixed"){controller = new ControllerFixed(p1,p2,p3,p4);control_type=CTLFIXEDINPUT;}
            else if(controller_type_str=="fixedonepoint"){controller = new ControllerFixed(p1,p2,p3,p4);control_type=CTLFIXEDONEPOINT;}
            else{controller = new ControllerFixed(p1,p2,p3,p4);control_type=CTLFIXEDINPUT;}

            Motor* motor;
            if(type=="nhinc") motor = new Motor(portinfo.portName(),id,logDir_,control_type,controller);
            else if(type == "nhabs") motor= new AbsMotor(portinfo.portName(),id,logDir_,control_type,controller);
            else if(type == "nhbigabs") motor= new AbsHeavyMotor(portinfo.portName(),id,logDir_,control_type,controller);
            else if(type == "piezo") motor= new PiezoMotor(portinfo.portName(),id,logDir_,control_type,controller);

            if(motor->isPortOpen() && motor->heartbeat())
            {
                QThread* motorThread=new QThread(this);
                motor ->moveToThread(motorThread);
                QObject::connect(this, &MotorKernel::DummySignal, motor, &Motor::dummy);
                QObject::connect(this,&MotorKernel::OpenLoopSignal,motor,&Motor::openLoopTrajectory);
                QObject::connect(this,&MotorKernel::ClosedLoopSignal,motor,&Motor::colsedLoopTrajectory);
                QObject::connect(this,&MotorKernel::ReadSensorSignal,motor,&Motor::readSensorSlot);
                QObject::connect(this,&MotorKernel::AbortSingal,motor,&Motor::stopSlot);
                motorThread->start();
                motorMap_.insert(id,motor);
                qDebug()<<"creating motor"<<id;
                CachedPortMap.insert(id,portinfo.portName());
                break;
            }
            else
            {
                if(!motor->isPortOpen())badports.insert(portinfo.portName());
                delete motor;
            }
        }
    }
    if(badports.size()>0)
    {
        qDebug() << "Bad ports:";
        for (const QString &port : badports)
            qDebug() << port;
    }

    // write cache file
    QString CacheFilePath = executableDir_ + "/cache.json";
    QFile CacheFile(CacheFilePath);
    QJsonObject jsonObj;
    for (auto it = CachedPortMap.constBegin(); it != CachedPortMap.constEnd(); ++it)
    {
        jsonObj[QString::number(it.key())] = it.value();
    }
    QJsonDocument jsonDocCachePort(jsonObj);
    if(CacheFile.open(QIODevice::WriteOnly))
    {
        CacheFile.write(jsonDocCachePort.toJson());
        CacheFile.close();
    }
    qDebug()<<"exit refresh motor list";
}

void MotorKernel::StopMotor(int id)
{
    // kill closedLoopTrajectory
    motorMap_[id]->setStatus(MOTOR_ABORT);
    // motor send stop command
    emit AbortSingal(id);
}

void MotorKernel::StopAll()
{
    for (auto it = motorMap_.begin(); it != motorMap_.end(); ++it)
    {
        StopMotor(it.key());
    }
}

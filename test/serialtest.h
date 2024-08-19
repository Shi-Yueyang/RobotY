#ifndef SERIALTEST_H
#define SERIALTEST_H

#include <QDebug>
#include<QThread>
#include <QDir>
#include<memory>
#include<QSerialPort>
#include<QSerialPortInfo>
#include <functional>
#include<QTimer>
#include<QThread>
#include<QObject>
#include<QCoreApplication>
#include"utils.h"

using SendingStrategyFunction = QList<QList<QDateTime>> (*)(QList<QSerialPort*>&, QList<int>&);

void create_serialports(QList<QSerialPort*>& ports,QList<int>& existedMotors);
QList<double> average_time_gap(QList<QList<QDateTime>>timepoints);
QList<QList<QDateTime>> sending_strategy1(QList<QSerialPort*>& ports,QList<int>& existedMotors);
QList<QList<QDateTime>> sending_strategy2(QList<QSerialPort*>& ports,QList<int>& existedMotors);
void test_serialport_speed(SendingStrategyFunction strategy);

class SerialPortWorker : public QObject {
    Q_OBJECT

public:
    explicit SerialPortWorker(QSerialPort* port, int id, QList<QDateTime>* timepoints, QObject* parent=nullptr)
        : QObject(parent), serialPort(port), m_id(id), timepoints_(timepoints), timeIsUp_(false)
    {
        serialPort->setParent(this);
    }

public slots:
    void sendData();

private:
    QSerialPort* serialPort;
    int m_id;
    QList<QDateTime>* timepoints_;
    bool timeIsUp_;
};

inline QByteArray generateCommand(int motorId, QString mode, double angle, double speed, bool query, bool stop)
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

inline double parseSensor(QByteArray dataBA)
{
    if(dataBA.size()<8)
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
    for (int i = 0; i < data.size(); ++i) {
        if (i > 0) {
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


#endif // SERIALTEST_H

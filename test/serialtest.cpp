#include "serialtest.h"
#include "utils.h"


void SerialPortWorker::sendData()
{
    QDateTime startTime = QDateTime::currentDateTime();
    qDebug() << "start at " << startTime.toString("yyyy-MM-dd hh:mm:ss.zzz");
    while (true) {
        QDateTime current = QDateTime::currentDateTime();
        if(startTime.secsTo(current)>1)
        {
            qDebug()<<"stop at "<<current.toString("yyyy-MM-dd hh:mm:ss.zzz");
            break;
        }
        if (serialPort->isOpen()) {
            QByteArray cmd = generateCommand(m_id," ",10,10,true,false);
            serialPort->write(cmd);
            serialPort->waitForBytesWritten(100);
            if (serialPort->waitForReadyRead(100)) {
                serialPort->readAll();
                timepoints_->push_back(current);
//                qDebug()<<"receiving "<<current.toString("yyyy-MM-dd hh:mm:ss.zzz");
            }
            else
            {
                qDebug()<<"not receiving "<<current.toString("yyyy-MM-dd hh:mm:ss.zzz");
            }
        }

    }
}

QList<QList<QDateTime>> sending_strategy2(QList<QSerialPort*>& ports,QList<int>& existedMotors)
{
    QList<QList<QDateTime>> timepoints(ports.size());
    QList<QThread*> threads;
    QList<SerialPortWorker*> workers;
    for (int i=0;i<ports.size();i++)
    {
        QThread* thread = new QThread;
        SerialPortWorker* worker = new SerialPortWorker(ports[i],existedMotors[i],&timepoints[i]);
        worker->moveToThread(thread);
        QObject::connect(thread, &QThread::finished, thread, &QThread::deleteLater);

        QObject::connect(thread, &QThread::started, worker, &SerialPortWorker::sendData);
        QObject::connect(thread, &QThread::finished, worker, &QObject::deleteLater);

        threads.append(thread);
        workers.append(worker);

        thread->start();
    }
    QThread::sleep(3);
    return timepoints;
}

QList<QList<QDateTime>> sending_strategy1(QList<QSerialPort*>& ports,QList<int>& existedMotors)
{
    QList<QList<QDateTime>> timepoints(ports.size());

    QDateTime startTime = QDateTime::currentDateTime();
    qDebug()<<"start at "<<startTime.toString("yyyy-MM-dd hh:mm:ss.zzz");
    while(true)
    {
        QDateTime current=QDateTime::currentDateTime();
        if(startTime.secsTo(current)>1)
        {
            qDebug()<<"stop at "<<current.toString("yyyy-MM-dd hh:mm:ss.zzz");
            break;
        }
        for(int i=0;i<ports.size();i++)
        {
            QByteArray cmd = generateCommand(existedMotors[i]," ",10,10,true,false);
            ports[i]->write(cmd);
            ports[i]->waitForBytesWritten(100);
            if(ports[i]->waitForReadyRead(100))
            {
                QByteArray response = ports[i]->readAll();
                timepoints[i].push_back(current);
            }
        }

    }
    return timepoints;
}

void test_serialport_speed(SendingStrategyFunction strategy)
{
    qDebug()<<ANSI_COLOR_BLUE <<"test_serialport_speed started"<< ANSI_COLOR_RESET;

    QList<QSerialPort*> ports;
    QList<int> existedMotors;
    create_serialports(ports,existedMotors);
    QList<QList<QDateTime>> timepoints=strategy(ports,existedMotors);

    QList<double> averagegap=average_time_gap(timepoints);
    for(int i=0;i<ports.size();i++)
    {
        qDebug()<<QString("motor[%1] port[%2] gap[%3]ms").arg(existedMotors[i]).arg(ports[i]->portName()).arg(averagegap[i]);
    }
    for(auto p:ports)
    {
        p->close();
        delete p;
    }
    qDebug()<<ANSI_COLOR_GREEN <<"test_serialport_speed pass"<< ANSI_COLOR_RESET<<"\n";

}

void create_serialports(QList<QSerialPort*>& ports,QList<int>& existedMotors)
{
    QSerialPortInfo info;
    QList<QSerialPortInfo> avaliablePorts = info.availablePorts();
    QList<int>motorIds{41,42,43,44,45,46};

    // open oprts
    for(auto& portinfo:avaliablePorts)
    {
        QSerialPort* port = new QSerialPort();

        // open port
        port->setPortName(portinfo.portName());
        port->setBaudRate(QSerialPort::Baud115200);
        port->setDataBits(QSerialPort::Data8);
        port->setParity(QSerialPort::NoParity);
        port->setStopBits(QSerialPort::OneStop);
        if (!port->open(QIODevice::ReadWrite)) {
            qDebug()<<"can't open port"<<port->portName();
            continue;
        }
        for(int id:motorIds)
        {
            QByteArray cmd = generateCommand(id," ",10,10,true,false);
            port->write(cmd);
            port->waitForBytesWritten(100);
            if(port->waitForReadyRead(100))
            {
                QByteArray response = port->readAll();
                // create a Motor if receiving response
                if (response.size()==8)
                {
                    ports.push_back(port);
                    existedMotors.push_back(id);
                    qDebug()<<"port["<<portinfo.portName()<<"] -- motor["<<id<<"]";
                    break;
                }
                else
                {
                    port->close();
                    delete port;
                }
            }
        }

    }
}

QList<double> average_time_gap(QList<QList<QDateTime>>timepoints)
{
    int n = timepoints.size();
    QList<double> avg_gap(n);
    for(int i=0;i<n;i++)
    {
        qint64 totalGap = 0;
        int num=timepoints[i].size();

        for (int j = 1; j < num; ++j) {
            qint64 gap = timepoints[i][j].msecsTo(timepoints[i][j-1]);
            totalGap += qAbs(gap);  // Use qAbs to handle negative gaps
        }
        avg_gap[i] = static_cast<double>(totalGap) / (num - 1);
    }
    return avg_gap;

}

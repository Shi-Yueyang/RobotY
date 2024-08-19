#include <QtGlobal>
#include "myserver.h"
#include "worker.h"

#define LOG(...) \
qDebug() << "[" << QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss:zzz") << "]" << __VA_ARGS__

MyServer::MyServer(QObject *parent)
    : QTcpServer(parent)
{
    QString exe_dir = QDir::currentPath();
    worker_ = new Worker(exe_dir);
    worker_->scan_motor_conf();
    QThread* workerThread=new QThread();
    worker_ ->moveToThread(workerThread);
    QObject::connect(worker_, &Worker::scan_motor_conf_request, worker_, &Worker::scan_motor_conf);
    QObject::connect(worker_, &Worker::run_motors_open_request, worker_, &Worker::run_motors_open);
    QObject::connect(worker_, &Worker::run_motors_close_request, worker_, &Worker::run_motors_close);
    QObject::connect(worker_, &Worker::run_one_motor_close_request, worker_, &Worker::run_one_motor_close);
    QObject::connect(worker_, &Worker::run_robot_request, worker_, &Worker::run_robot);
    QObject::connect(worker_, &Worker::run_motors_home_request, worker_, &Worker::run_motors_home);
    QObject::connect(worker_, &Worker::send_cmd_request, worker_, &Worker::send_cmd);
    QObject::connect(worker_, &Worker::run_one_motor_one_pos_open_request, worker_, &Worker::run_one_motor_one_pos_open);
    QObject::connect(worker_, &Worker::run_one_motor_one_pos_closed_request, worker_, &Worker::run_one_motor_one_pos_closed);


    QObject::connect(workerThread, &QThread::finished, workerThread, &QThread::deleteLater);
    workerThread->start();
    qDebug();

}

void MyServer::incomingConnection(qintptr socketDescriptor)
{
    // Create a new socket for the incoming connection
    QTcpSocket *socket = new QTcpSocket(this);
    if (!socket->setSocketDescriptor(socketDescriptor))
    {
        // Handle error
         LOG("Error setting socket descriptor:", socket->errorString());
        return;
    }

    // Connect signals/slots for socket
    connect(socket, &QTcpSocket::readyRead, this, &MyServer::readyRead);
    connect(socket, &QTcpSocket::disconnected, socket, &QTcpSocket::deleteLater);

    // Inform that a new client has connected
    LOG("New client connected");
}

void MyServer::readyRead()
{
    QTcpSocket *socket = qobject_cast<QTcpSocket *>(sender());
    if (!socket)
        return;

    // Read data from the socket
    QByteArray data = socket->readAll();
    QString input = QString::fromUtf8(data);
    bool doResponse=true;
    QStringList commands = input.split(';');
    QString cmd = commands[0];

//    LOG("data:"<<data);
//    LOG("input:"<<input);


    QString response = "[" + QDateTime::currentDateTime().toString("yyyy_MM_dd:hh_mm_ss_zzz") + "]\n";
    cmd = cmd.trimmed();
    QStringList parts = cmd.split(" ");

    // connect
    if (parts.at(0).toLower()== "c")
    {
        emit worker_->scan_motor_conf_request();
    }

    // print
    else if (parts.at(0).toLower()== "pr")
    {
        if(parts.size()==1)
        {
            response += worker_->printStatus();
        }
        else
        {
            int id, real_id=-1;
            QString id_str = parts.at(1);
            QList<QString> id_lst = parts.at(1).split("-");
            if(id_lst.size()==1) id = id_str.toInt();
            else
            {
                id = id_lst.at(0).toInt();
                real_id = id_lst.at(1).toInt();
            }

            response += worker_->printStatus(id,real_id);
        }
    }

    // print robot axis information
    else if(parts.at(0).toLower()=="yt")
    {
        response += worker_->printRobotAxisYt(false);
        doResponse=false;
    }

    // run closed loop
    else if( parts.at(0).toLower()== "cr" && parts.size()==4)
    {
        QString id_str = parts.at(1);
        QList<QString> id_lst = id_str.split("-");
        int id, real_id=-1;
        if(id_lst.size()==1) id = id_str.toInt();
        else
        {
            id = id_lst.at(0).toInt();
            real_id = id_lst.at(1).toInt();
        }

        double pos = parts.at(2).toDouble();
        double duration = parts.at(3).toDouble();
        emit worker_->run_one_motor_one_pos_closed_request(id,pos,duration,real_id);
        response += "run closedloop";
        response += QString(" id[%1] real_id[%2] pos[%3] duration[%4]").arg(id).arg(real_id).arg(pos, 0, 'f', 2).arg(duration, 0, 'f', 2);
    }

    // run openloop
    else if(parts.at(0).toLower()== "or2" && parts.size()==4)
    {
        QString id_str = parts.at(1);
        QList<QString> id_lst = id_str.split("-");
        int id, real_id=-1;
        if(id_lst.size()==1) id = id_str.toInt();
        else
        {
            id = id_lst.at(0).toInt();
            real_id = id_lst.at(1).toInt();
        }

        double pos = parts.at(2).toDouble();
        double vel = parts.at(3).toDouble();
        emit worker_->run_one_motor_one_pos_open_request(id,pos,vel,real_id);
        response += "run openloop";
        response += QString(" id[%1] real_id[%2] pos[%3] vel[%4]").arg(id).arg(real_id).arg(pos, 0, 'f', 2).arg(vel, 0, 'f', 2);
    }

    // inverse kinematics
    else if (parts.at(0).toLower()== "ik" && parts.size()==7)
    {
        QVector<double> points(6);
        for (int i = 1; i < parts.size(); i++)
        {
            bool conversionOk;
            double p = parts.at(i).toDouble(&conversionOk);
            if(conversionOk)
            {
                points[i-1] = p;
            }
        }

        QMap<QString,double> solution = worker_->robotIK2P(points[0],points[1],points[2],points[3],points[4],points[5]);
        for (auto it = solution.constBegin(); it != solution.constEnd(); ++it)
        {
            response += it.key() + ":" + QString::number(it.value()) + "\n";
        }
    }

    // run robots
    else if (parts.at(0).toLower()== "rr" && parts.size()==8)
    {
//        // input points
//        QVector<double> points(6);
//        QString order=parts.at(7);
//        for (int i = 1; i < parts.size(); i++)
//        {
//            bool conversionOk;
//            double p = parts.at(i).toDouble(&conversionOk);
//            if(conversionOk)
//            {
//                points[i-1] = p;
//            }
//        }
//        emit worker_->run_robot_request(points[0],points[1],points[2],points[3],points[4],points[5],order);
//        response += "running robot";
    }

    // stop
    else if (parts.at(0).toLower()== "s")
    {
        worker_->stop();
        response += "stop";
    }
    // home
    else if (parts.at(0).toLower()== "ho")
    {
        emit worker_->run_motors_home_request(5);
        response += "home";
    }

    else if(parts.at(0).toLower()== "help")
    {
        response += worker_->printHelp();
    }
    else
    {
        response += "wrong command";
    }

    // response
    if(doResponse)
    {
        LOG("cmd:"<<cmd);
    }
    socket->write(response.toUtf8());



}


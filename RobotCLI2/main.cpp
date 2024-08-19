#include <QCoreApplication>
#include "worker.h"
#include "utils.h"


int main(int argc, char *argv[])
{
    QCoreApplication app(argc, argv);
    // connecting
    qDebug()<<QString("[%1] refresh motor:").arg(QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss.zzz"));

    // create worker, worker thread
    QString exe_dir = QDir::currentPath();
    Worker* worker = new Worker(exe_dir);
    worker->scan_motor_conf();
    QThread* workerThread=new QThread();
    worker ->moveToThread(workerThread);
    QObject::connect(worker, &Worker::scan_motor_conf_request, worker, &Worker::scan_motor_conf);
    QObject::connect(worker, &Worker::run_motors_open_request, worker, &Worker::run_motors_open);
    QObject::connect(worker, &Worker::run_motors_close_request, worker, &Worker::run_motors_close);
    QObject::connect(worker, &Worker::run_one_motor_close_request, worker, &Worker::run_one_motor_close);
    QObject::connect(worker, &Worker::run_robot_request, worker, &Worker::run_robot);
    QObject::connect(worker, &Worker::run_motors_home_request, worker, &Worker::run_motors_home);
    QObject::connect(worker, &Worker::send_cmd_request, worker, &Worker::send_cmd);
    QObject::connect(worker, &Worker::run_one_motor_one_pos_open_request, worker, &Worker::run_one_motor_one_pos_open);
    QObject::connect(worker, &Worker::run_one_motor_one_pos_closed_request, worker, &Worker::run_one_motor_one_pos_closed);


    QObject::connect(workerThread, &QThread::finished, workerThread, &QThread::deleteLater);
    workerThread->start();
    qDebug();

    // the loop
    worker->printHelp();
    QTextStream inputStream(stdin);
    while(true)
    {
        qDebug()<<ANSI_COLOR_BLUE <<"input command:"<< ANSI_COLOR_RESET;
        app.processEvents();
        QString input = inputStream.readLine().trimmed(); // Trim whitespace
        QStringList commands = input.split(';');
        for (QString& cmd : commands)
        {
            cmd = cmd.trimmed();
            QStringList parts = cmd.split(" ");

            // connect
            if (parts.at(0).toLower()== "connect"|| parts.at(0).toLower()== "c")
            {
                emit worker->scan_motor_conf_request();
            }

            // print
            else if (parts.at(0).toLower()== "pr" )
            {
                if(parts.size()==1)
                    worker->printStatus();
                else if(parts.size()==2)
                {
                    if(parts.at(1).toLower()== "l")
                    {
                        worker->printStatusLong();
                    }
                    else if(parts.at(1).toLower()=="ax")
                    {
                        worker->printRobotAxis();
                    }
                    else
                    {
                        int id, real_id=-1;
                        QString id_str = parts.at(1);
                        QList<QString> id_lst = id_str.split("-");
                        if(id_lst.size()==1) id = id_str.toInt();
                        else
                        {
                            id = id_lst.at(0).toInt();
                            real_id = id_lst.at(1).toInt();
                        }

                        worker->printStatus(id,real_id);
                    }
                }
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

                QMap<QString,double> solution = worker->robotIK2P(points[0],points[1],points[2],points[3],points[4],points[5]);
                for (auto it = solution.constBegin(); it != solution.constEnd(); ++it)
                {
                    qDebug() << it.key() << ":" << it.value();
                }
            }

            // run robots
            else if (parts.at(0).toLower()== "rr" && parts.size()==8)
            {
                // input points
                QVector<double> points(6);
                QString order=parts.at(7);
                for (int i = 1; i < parts.size(); i++)
                {
                    bool conversionOk;
                    double p = parts.at(i).toDouble(&conversionOk);
                    if(conversionOk)
                    {
                        points[i-1] = p;
                    }
                }
                emit worker->run_robot_request(points[0],points[1],points[2],points[3],points[4],points[5],order);
            }

            // home
            else if (parts.at(0).toLower()== "home" || parts.at(0).toLower()== "ho")
            {
                emit worker->run_motors_home_request(5);
            }

            // stop
            else if (parts.at(0).toLower()== "stop" || parts.at(0).toLower()== "s")
            {
                worker->stop();
            }

            // set openloop vel
            else if((parts.at(0).toLower()=="vel" || parts.at(0).toLower()=="v") && parts.size()>=3)
            {
                for (int i = 1; i < parts.size(); i += 2)
                {
                    bool conversionOk;
                    int id = parts.at(i).toInt(&conversionOk);
                    if(!conversionOk || !worker->hasMotor(id))
                    {
                        qDebug() << "Invalid id:" << id;
                        continue;
                    }
                    double vel = parts.at(i + 1).toDouble(&conversionOk);
                    if (!conversionOk)
                    {
                        qDebug() << "Invalid vel:"<<vel<<" for id:" << id;
                        continue;
                    }
                    worker->set_open_vel(id,vel);
                }
            }

            // set pos
            else if((parts.at(0).toLower()=="pos"||parts.at(0).toLower()=="p") && parts.size()>=3)
            {
                for (int i = 1; i < parts.size(); i += 2)
                {
                    bool conversionOk;
                    int id = parts.at(i).toInt(&conversionOk);
                    if(!conversionOk || !worker->hasMotor(id))
                    {
                        qDebug() << "Invalid id:" << id;
                        continue;
                    }

                    double pos = parts.at(i + 1).toDouble(&conversionOk);
                    if (!conversionOk)
                    {
                        qDebug() << "Invalid pos:"<<pos<<" for id:" << id;
                        continue;
                    }
                    worker->set_pos(id,pos);
                }
            }

            // run openloop
            else if(parts.at(0).toLower()== "openrun" || parts.at(0).toLower()== "or")
            {
                emit worker->run_motors_open_request();
            }

            // run openloop 2
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
                emit worker->run_one_motor_one_pos_open_request(id,pos,vel,real_id);
            }

            // run closedloop
            else if((parts.at(0).toLower()== "closerun" || parts.at(0).toLower()== "cr") && parts.size()==2)
            {
                bool conversionOk;
                double duration = parts.at(1).toDouble(&conversionOk);
                if(!conversionOk)
                {
                    qDebug() << "Invalid duration";
                    continue;
                }
                emit worker->run_motors_close_request(duration);
            }

            else if(parts.at(0).toLower()== "cr2" && parts.size()==4)
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
                emit worker->run_one_motor_one_pos_closed_request(id,pos,duration,real_id);
            }


            // send command
            else if((parts.at(0).toLower()== "sendcmd" || parts.at(0).toLower()== "sc") && parts.size()==3)
            {
                bool conversionOk;
                int id = parts.at(1).toInt(&conversionOk);
                QString cmd = parts.at(2);
                emit worker->send_cmd_request(id,cmd);
            }

            // help
            else if(parts.at(0).toLower()== "help" || parts.at(0).toLower()== "h")
            {
                worker->printHelp();
            }
        }

    }
    return app.exec();
}

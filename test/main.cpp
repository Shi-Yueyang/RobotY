#include <QDebug>
#include<QThread>
#include <QDir>
#include <cmath>
#include<memory>
#include<QSerialPort>
#include <cmath>
#include <tuple>
#include "utils.h"
#include "serialtest.h"
#include "motorkernel.h"
#include "robotengine.h"
#include "motor.h"


void test_trajectory_getDesiredPosition()
{
    qDebug()<<ANSI_COLOR_BLUE <<"Trajectory::trajectory_getDesiredPosition started"<< ANSI_COLOR_RESET;
    Trajectory traj;
    traj.addWaypoint(0,0);
    traj.addWaypoint(1,1);
    traj.addWaypoint(2,3);
    double t = 1.75;
    double p = traj.getDesiredPosition(t);
    qDebug()<<t<<"-"<<p;

    t = 2;
    p = traj.getDesiredPosition(t);
    qDebug()<<t<<"-"<<p;

    t = 2.2;
    p = traj.getDesiredPosition(t);
    qDebug()<<t<<"-"<<p;

}


void test_trajectory_getNextPosition()
{
    qDebug()<<ANSI_COLOR_BLUE <<"Trajectory::trajectory_getDesiredPosition started"<< ANSI_COLOR_RESET;
    Trajectory traj;
    traj.addWaypoint(0,0);
    traj.addWaypoint(1,1);
    traj.addWaypoint(2,3);

    double t = 0;
    double p = traj.getNextPosition(t);
    qDebug()<<t<<"-"<<p;

    t = 0.5;
    p = traj.getNextPosition(t);
    qDebug()<<t<<"-"<<p;

    t = 1.75;
    p = traj.getNextPosition(t);
    qDebug()<<t<<"-"<<p;

    t = 2;
    p = traj.getNextPosition(t);
    qDebug()<<t<<"-"<<p;

    t = 2.5;
    p = traj.getNextPosition(t);
    qDebug()<<t<<"-"<<p;
}

void test_trajectory_getDesiredVelocity()
{
    Trajectory traj;
    traj.addWaypoint(0,0);
    traj.addWaypoint(1,1);
    traj.addWaypoint(2,3.5);
    double v1 = traj.getDesiredVelocity(0);
    double v2 = traj.getDesiredVelocity(0.5);
    double v3 = traj.getDesiredVelocity(1);
    double v4 = traj.getDesiredVelocity(1.5);
    double v5 = traj.getDesiredVelocity(2);
    double v6 = traj.getDesiredVelocity(3);
    qDebug()<<QString("v1=%1, v2=%2, v3=%3, v4=%4, v5=%5, v6=%6").arg(v1).arg(v2).arg(v3).arg(v4).arg(v5).arg(v6);
}

void test_motorKernel_RefreshMotorLIst()
{
    qDebug()<<ANSI_COLOR_BLUE <<"MotorKernel::RefreshMotorLIst started"<< ANSI_COLOR_RESET;
    std::unique_ptr<MotorKernel> kernel = std::make_unique<MotorKernel>(QDir::currentPath());
    kernel->RefreshMotorList();
    QThread::sleep(1);
    qDebug()<<ANSI_COLOR_GREEN <<"MotorKernel::RefreshMotorLIst pass"<< ANSI_COLOR_RESET<<"\n";

}


void test_motorKernel_EmitDummy()
{
    MotorKernel* kernel = new MotorKernel(QDir::currentPath());
    kernel->RefreshMotorList();
    for(int motorId:kernel->GetMotorList())
    {
        kernel->EmitDummy("hello");
        break;
    }

}

void test_EmitDummy_in_Mission()
{
    MotorKernel* kernel = new MotorKernel(QDir::currentPath());
    kernel->RefreshMotorList();
    QList<double> timepoints{0,3,6};
    QList<double> posPoints{0,70,0};

    for(int motorId:kernel->GetMotorList())
    {
        kernel->EmitClosedLoop(motorId,timepoints,posPoints);
        kernel->EmitDummy("hello");
        break;
    }
}

void test_motorKernel_EmitOpenLoop()
{
    MotorKernel* kernel = new MotorKernel(QDir::currentPath());
    kernel->RefreshMotorList();
    QList<double> timepoints{0,1,2};
    QList<double> posPoints{0,60,0};
    QList<double> velPoints{0,60,30};
    for(int motorId:kernel->GetMotorList())
    {
        kernel->EmitOpenLoop(motorId,timepoints,posPoints,velPoints);
    }


}

void test_motorKernel_EmitClosedLoop()
{
    MotorKernel* kernel = new MotorKernel(QDir::currentPath());
    kernel->RefreshMotorList();
    QList<double> timepoints{0,3,6};
    QList<double> posPoints{0,50,0};
//    for(int motorId:kernel->GetMotorList())
//    {
//        kernel->EmitClosedLoop(motorId,timepoints,posPoints);
//    }
    kernel->EmitClosedLoop(1,timepoints,posPoints);

//    QList<double> timepoints{0,5,10};
//    QList<double> posPoints{0,500,0};
//    kernel->EmitClosedLoop(47,timepoints,posPoints);

}
void test_repeat_open()
{
    int id=47;
    MotorKernel* kernel = new MotorKernel(QDir::currentPath());
    kernel->RefreshMotorList();
    QList<double> targets={-30,-15,15,30};
    QList<double> vels={75,75,80,80};
    int repeat=5;
    for(int i=0;i<targets.size();i++)
    {
        for(int j=0;j<repeat;j++)
        {
            qDebug()<<i<<" "<<j;
            double target = targets[i]*50;
            double vel = vels[i];
            kernel->EmitOpenLoop(id,{0},{target},{vel});
            QThread::sleep(15);
            kernel->EmitOpenLoop(id,{0},{0},{vel});
            QThread::sleep(12);
        }
    }
    qDebug()<<"finsh";
}

void test_repeat_close()
{
    MotorKernel* kernel = new MotorKernel(QDir::currentPath());
    kernel->RefreshMotorList();
    QList<double> timepoints={0};
    QList<double> posPoints={0};
    int repeat=5;
    for(int i=1;i<=1;i++)
    {
        int n = repeat*2;
        double dt=4;
        double target=60*i;

        for(int i=1;i<=2*n;i++)
        {
            timepoints.append(timepoints.last()+dt);
            if(((i+1)/2)%2!=0)
            {
                posPoints.append(target);
            }
            else
            {
                posPoints.append(0);
            }
        }
    }

    qDebug() << "Time \t Position";
    qDebug() << "-------------------";
    for (int i = 0; i < qMin(timepoints.size(), posPoints.size()); ++i)
    {
        qDebug() << timepoints.at(i) << "\t" << posPoints.at(i);
    }

    kernel->EmitClosedLoop(43,timepoints,posPoints);
}

void test_abortmission()
{
    MotorKernel* kernel = new MotorKernel(QDir::currentPath());
    kernel->RefreshMotorList();
    QList<double> timepoints{0,3,6};
    QList<double> posPoints{0,70,0};

    for(int motorId:kernel->GetMotorList())
    {
        kernel->EmitClosedLoop(motorId,timepoints,posPoints);
        QThread::msleep(200);
        kernel->StopMotor(motorId);
        break;
    }
}

void test_Run_one_command()
{
    MotorKernel* kernel = new MotorKernel(QDir::currentPath());
    kernel->RefreshMotorList();
    for(int motorId:kernel->GetMotorList())
    {
        kernel->SendHeartBeat(motorId);
        kernel->RunMotorOnePos(motorId,0,30);
        QThread::sleep(3);
        kernel->RunMotorOnePos(motorId,90,30);
        QThread::sleep(3);
        kernel->RunMotorOnePos(motorId,180,30);
        QThread::sleep(3);
        kernel->RunMotorOnePos(motorId,270,30);
        QThread::sleep(3);
        kernel->RunMotorOnePos(motorId,360,30);
    }
}

void test_abs_motor_parse_sensor()
{
    MotorKernel* kernel = new MotorKernel(QDir::currentPath());
    kernel->RefreshMotorList();
    int sleeptime=4;
    for(int motorId:kernel->GetMotorList())
    {
        kernel->RunMotorOnePos(motorId,0,30);
        QThread::sleep(sleeptime);
        kernel->GetPos(motorId);

        kernel->RunMotorOnePos(motorId,90,30);
        QThread::sleep(sleeptime);
        kernel->GetPos(motorId);

        kernel->RunMotorOnePos(motorId,180,30);
        QThread::sleep(sleeptime);
        kernel->GetPos(motorId);

        kernel->RunMotorOnePos(motorId,270,30);
        QThread::sleep(sleeptime);
        kernel->GetPos(motorId);

        kernel->RunMotorOnePos(motorId,360,30);
        QThread::sleep(sleeptime);
        kernel->GetPos(motorId);
    }
}

void test_piezo_command()
{
    ControllerPID c(0,0,0,0);
    QDir dir =(QCoreApplication::applicationDirPath() + QDir::separator() + "log"+QDir::separator()+QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss"));

    Motor* mtr = new PiezoMotor("none",1,dir,CTLPID,&c);
    QByteArray cmd =  mtr->generateCommand(1,"c",10,0,false,false);
    QByteArray cmdstop =  mtr->generateCommand(1,"c",10,0,false,true);
    QByteArray cmdquery =  mtr->generateCommand(1,"c",10,0,true,false);

    qDebug()<<"move command"<<cmd;
    qDebug()<<"stop command"<<cmdstop;
    qDebug()<<"query command"<<cmdquery;
}

void test_acbsc()
{
    double a = 3.0;
    double b = 8.0;
    double c = 5.0;

    QPair<double, double> result = acbsc(a, b, c);
    qDebug() << "Theta1:" << result.first;
    qDebug() << "Theta2:" << result.second;
}


void test_format_list()
{
    QList<double> myList = {1.111, 2.222, 3.333, 4.444, 5.555, 6.666};
    QString formattedString = formatList(myList);
    qDebug()<<formattedString;

}

void test_kinematic_Long()
{
    RobotEngineLong engine(QDir::currentPath());
    double x1=-203.16;
    double y1=56.7;
    double z1=160.61;
    double x2=x1,  y2= y1, z2= z1-10;

    QMap<QString,double> result = engine.robotIK2P(x1,y1,z1,x2,y2,z2);
    qDebug() << "Result:";
    for (auto it = result.constBegin(); it != result.constEnd(); ++it) {
        qDebug() << it.key() << ": " << it.value();
    }
}

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    qDebug()<<ANSI_COLOR_CYAN<<"test started"<<ANSI_COLOR_RESET;

//    test_trajectory_getNextPosition();
//    test_trajectory_getDesiredPosition();
//    test_motorKernel_RefreshMotorLIst();
//    test_serialport_speed(&sending_strategy2);
//    test_trajectory_getDesiredVelocity();
//    test_motorKernel_EmitDummy();
//    test_motorKernel_EmitOpenLoop();
//    test_abs_angle_to_hex();
//    test_Run_one_command();
//    test_abs_motor_parse_sensor();
//    test_motorKernel_EmitClosedLoop();
//    test_piezo_command();
//    test_acbsc();
//    test_rcm_ik();
//    test_format_list();
//    test_EmitDummy_in_Mission();
//    test_abortmission();
//    test_repeat_close();
//    test_repeat_open();
    test_kinematic_Long();

    return a.exec();
}

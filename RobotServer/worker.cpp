#include "Worker.h"
#include "utils.h"
#include <QVector>
#include <limits>
#include <cmath>

Worker::Worker(QString exe_dir)
    : exe_dir_(exe_dir)
{
    kernel_ = new MotorKernel(exe_dir);
    robot_ = new RobotEngineLong(exe_dir);
}

Worker::~Worker()
{
    delete kernel_;
    delete robot_;
}

void Worker::scan_motor_conf()
{
    qDebug() << "scan motor  conf";
    targets_.clear();
    openVel_.clear();
    kernel_->RefreshMotorList();
    for(int id:kernel_->GetMotorList())
    {
        targets_.insert(id,0);
        openVel_.insert(id,0);
    }
}

void Worker::run_motors_open()
{
    if (hasJob_.empty())
    {
        qDebug() << "no new job ";
    }
    QList<int> activeMotors = kernel_->GetMotorList();

    for (int motorId : hasJob_)
    {
        if (!activeMotors.contains(motorId))
        {
            continue;
        }
        double pos = targets_.value(motorId);
        double vel = openVel_.value(motorId);
        kernel_->EmitOpenLoop(motorId, {0}, {pos}, {vel});
    }
    QThread::msleep(50);
    while (!kernel_->IsAllMotorFree())
    {
        QThread::msleep(50);
    }
    hasJob_.clear();
    qDebug() << "finish";
    hasJob_.clear();
}

void Worker::run_one_motor_one_pos_open(int id, double pos, double vel, int real_id)
{
    kernel_->EmitOpenLoop(id,{0},{pos},{vel}, real_id);
}

void Worker::run_one_motor_one_pos_closed(int id, double pos, double duration, int real_id)
{
    double startPos;
    double lastPos;
    do
    {
        startPos = kernel_->GetPos(id);
        if (std::abs(startPos - lastPos) < 1)
        {
            break;
        }
        lastPos = startPos;
    } while (true);

    QList<double> timepoints{0, duration};
    QList<double> posPoints{startPos, pos};
    kernel_->EmitClosedLoop(id, timepoints, posPoints,real_id);
}


void Worker::run_motors_close(double duration)
{
    if (hasJob_.empty())
    {
        qDebug() << "no new job ";
    }
    QList<int> activeMotors = kernel_->GetMotorList();

    for (int motorId : hasJob_)
    {
        double pos = targets_.value(motorId);
        if (!activeMotors.contains(motorId))
        {
            continue;
        }
        double startPos;
        double lastPos;
        do
        {
            startPos = kernel_->GetPos(motorId);
            if (std::abs(startPos - lastPos) < 1)
            {
                break;
            }
            lastPos = startPos;
        } while (true);

        QList<double> timepoints{0, duration};
        QList<double> posPoints{startPos, pos};
        kernel_->EmitClosedLoop(motorId, timepoints, posPoints);
    }
    hasJob_.clear();
}

void Worker::run_one_motor_close(int id, double target, double duration)
{
    double startPos = kernel_->GetPos(id);
    QList<double> timepoints{0, duration};
    QList<double> posPoints{startPos, target};
    kernel_->EmitClosedLoop(id, timepoints, posPoints);
}

void Worker::run_robot(double x1, double y1, double z1, double x2, double y2, double z2,QString order)
{
    stopFlag_=false;
    QMap<QString,double> solution = robot_->robotIK2P(x1,y1,z1,x2,y2,z2);
    QVector<QString> axes_all;
    for (int i = 0; i < order.length(); ++i)
    {
        QChar ch = order.at(i);
        if (ch == 'x') axes_all.append("rx");
        else if (ch == 'y') axes_all.append("ry");
        else if (ch == 'z') axes_all.append("rz");
        else if (ch == 'a') axes_all.append("r1");
        else if (ch == 'b') axes_all.append("r2");
        else if (ch == 'c') axes_all.append("rn");

    }
    for(int i = 0;i<axes_all.size() && !stopFlag_;i++)
    {
        QString axis_name = axes_all.at(i);
        int id = robot_->axisToMotor(axis_name);
        double target = solution.value(axis_name);
        double duration = 3;
        if(kernel_->GetMotorList().contains(id))
        {
            run_one_motor_close(id,target,duration);
        }
        qDebug()<<"step "<<i;
        do
        {
            QThread::msleep(50);
        }while(!kernel_->IsAllMotorFree() && !stopFlag_);
    }
    if(stopFlag_)
    {
        kernel_->StopAll();
        stopFlag_=false;
    }
    qDebug()<<"finish run robot";
}

void Worker::run_motors_home(double duration)
{
    QList<int> activeMotors = kernel_->GetMotorList();
    for(int id:activeMotors)
    {
        double target = 0;
        run_one_motor_close(id,target,duration);
    }
}

void Worker::send_cmd(int id, QString cmd)
{
    kernel_->RunMotorOneCmd(id,cmd);
}

QString Worker::printStatus()
{

    QString status;
    auto activeMotors = kernel_->GetMotorList();
    for (int motorId : activeMotors)
    {
        kernel_->GetPos(motorId);
        QString shortStatus = kernel_->PrintStatusShort(motorId);
        status += shortStatus + "\n";
    }
    return status;
}

QString Worker::printStatus(int id,int real_id)
{
    QList<int> activeMotors = kernel_->GetMotorList();
    QString status;
    if(activeMotors.contains(id))
    {
        kernel_->GetPos(id,real_id);
        status = kernel_->PrintStatusShort(id);
    }
    return status;
}

void Worker::printStatusLong()
{
    QList<int> activeMotors = kernel_->GetMotorList();
    for (int motorId : activeMotors)
    {
        kernel_->PrintStatus(motorId);
    }
}

QString Worker::printRobotAxisYt(bool doPrint)
{
    QList<int> activeMotors = kernel_->GetMotorList();
    QString response="yt:";
    QVector<double> all_poses(6, std::numeric_limits<double>::quiet_NaN());
    for (int id : activeMotors)
    {
        QString axis = robot_->motorToAxis(id);
        double pos = kernel_->GetPosFast(id);
        if(axis=="rx") all_poses[0]=pos/120;
        else if(axis=="ry") all_poses[1]=pos/120;
        else if(axis=="rz") all_poses[2]=pos/120;
        else if(axis=="r1") all_poses[3]=pos/50;
        else if(axis=="r2") all_poses[4]=pos;
        else if(axis=="rn") all_poses[5]=pos/36;
    }
    for(int i=0;i<all_poses.size();i++)
    {
        double pos = all_poses[i];
        if(std::isnan(pos)) response += "NaN";
        else response += QString::number(pos, 'f', 3);

        if(i==all_poses.size()-1) response += ";";
        else response += ",";
    }
    if(doPrint)
    {
        qDebug()<<response;
    }
    return response;
}

QString Worker::printHelp()
{
    QString message;
    message += QString(ANSI_COLOR_BLUE) + "-------------------help----------------------" + ANSI_COLOR_RESET + "\n";
    message += QString(ANSI_COLOR_RED) + "he" + ANSI_COLOR_RESET + " print help\n";
    message += QString(ANSI_COLOR_RED) + "c" + ANSI_COLOR_RESET + " reconnect to motor drivers\n";
    message += QString(ANSI_COLOR_RED) + "pr (id) (long/l)" + ANSI_COLOR_RESET + " print current motor position or other info\n";
    message += QString(ANSI_COLOR_RED) + "ik [x1] [y1] [z1] [x2] [y2] [z2]" + ANSI_COLOR_RESET + " solve inverse kinematics\n";
    message += QString(ANSI_COLOR_RED) + "or [id-(real_id)] [pos] [vel]" + ANSI_COLOR_RESET + " move in openloop mode\n";
    message += QString(ANSI_COLOR_RED) + "cr [id-(real_id)] [pos] [duration]" + ANSI_COLOR_RESET + " move in closedloop mode\n";
    message += QString(ANSI_COLOR_RED) + "rr [x1] [y1] [z1] [x2] [y2] [z2] [order]" + ANSI_COLOR_RESET + " run robot with inversekinematics\n";
    message += QString(ANSI_COLOR_RED) + "s" + ANSI_COLOR_RESET + " stop all motors\n";
    message += QString(ANSI_COLOR_RED) + "ho" + ANSI_COLOR_RESET + " return all motors to zero\n";
    message += QString(ANSI_COLOR_BLUE) + "--------------------------------------------" + ANSI_COLOR_RESET + "\n";
    return message;
}

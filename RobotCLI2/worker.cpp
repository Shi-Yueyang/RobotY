#include "Worker.h"
#include "utils.h"

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
    qDebug() << "scan motor conf";
    targets_.clear();
    openVel_.clear();
    kernel_->RefreshMotorList();
    robot_->refreshAxisMap();
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

void Worker::printStatus()
{
    printMap(targets_,"target");
    printMap(openVel_,"openvel");
    QList<int> activeMotors = kernel_->GetMotorList();
    qDebug()<<"----------------";
    for (int motorId : activeMotors)
    {
        kernel_->GetPos(motorId);
        kernel_->PrintStatusShort(motorId);
    }
}

void Worker::printRobotAxis()
{
    robot_->printAxisMap();
}

void Worker::printStatus(int id,int real_id)
{
    QList<int> activeMotors = kernel_->GetMotorList();
    if(activeMotors.contains(id))
    {
        kernel_->GetPos(id,real_id);
        kernel_->PrintStatusShort(id);
    }
}

void Worker::printStatusLong()
{
    QList<int> activeMotors = kernel_->GetMotorList();
    for (int motorId : activeMotors)
    {
        kernel_->PrintStatus(motorId);
    }
}

void Worker::printMap(const QMap<int, double>& map, QString mapname)
{
    if(hasJob_.empty())
    {
        return;
    }
    qDebug().noquote() << "----------------";
    qDebug().noquote() << mapname;
    for (auto it = map.constBegin(); it != map.constEnd(); ++it)
    {
        if(hasJob_.contains(it.key()))
        {
            qDebug().noquote() << QString("Motor %1: %2").arg(it.key(), 2).arg(it.value(), 10, 'f', 2);
        }
    }
}

void Worker::printHelp()
{
    qDebug() << ANSI_COLOR_BLUE << "-------------------help----------------------" << ANSI_COLOR_RESET;
    qDebug() << ANSI_COLOR_RED << "h" << ANSI_COLOR_RESET << " print help";
    qDebug() << ANSI_COLOR_RED << "c" << ANSI_COLOR_RESET << " reconnect to motor drivers";
    qDebug() << ANSI_COLOR_RED << "pr (id) (long/l)" << ANSI_COLOR_RESET << " print current motor position or other info";
    qDebug() << ANSI_COLOR_RED << "ik [x1] [y1] [z1] [x2] [y2] [z2]" << ANSI_COLOR_RESET << "solve inverse kinematics";
    qDebug() << ANSI_COLOR_RED << "or2 [id-(real_id)] [pos] [vel]" << ANSI_COLOR_RESET << " move in openloop mode";
    qDebug() << ANSI_COLOR_RED << "cr2 [id-(real_id)] [pos] [duration]" << ANSI_COLOR_RESET << " move in closedloop mode";

    qDebug() << ANSI_COLOR_RED << "rr [x1] [y1] [z1] [x2] [y2] [z2] [sequence]" << ANSI_COLOR_RESET << " run robot with inversekinematics";
    qDebug() << ANSI_COLOR_RED << "sc [id] [cmd str]" << ANSI_COLOR_RESET << "send [cmd str] to [id]";
    qDebug() << ANSI_COLOR_RED << "s" << ANSI_COLOR_RESET << " stop all motors";
    qDebug() << ANSI_COLOR_RED << "home" << ANSI_COLOR_RESET << " return all motors to zero";
    qDebug() << ANSI_COLOR_BLUE << "--------------------------------------------" << ANSI_COLOR_RESET;
}

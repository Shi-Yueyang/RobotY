#include "robotengine.h"
#include <iostream>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>

RobotEngine::RobotEngine(QString executableDir):
    executableDir_(executableDir)
{
    refreshAxisMap();
}

void RobotEngine::refreshAxisMap()
{

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
    QJsonObject axisMapObj = rootObj["axis_map"].toObject();
    for (auto it = axisMapObj.constBegin(); it != axisMapObj.constEnd(); ++it)
    {
        axis_map_.insert(it.key(),it.value().toInt());
        axis_map_inv_.insert(it.value().toInt(),it.key());
    }
}

QPair<double, double> RobotEngine::acbsc(double a, double b, double c)
{
    double theta1, theta2;

    if (a*a + b*b - c*c > -1e-8) {
        double Delta = qMax(0.0, a*a + b*b - c*c);
        double t1 = (b + qSqrt(Delta)) / (a + c);
        double t2 = (b - qSqrt(Delta)) / (a + c);
        theta1 = 2 * qAtan(t1);
        theta2 = 2 * qAtan(t2);
    } else {
        theta1 = qQNaN();
        theta2 = qQNaN();
    }

    return qMakePair(theta1, theta2);
}

double RobotEngine::dotProduct(const QVector<double>& v1, const QVector<double>& v2)
{
    if (v1.size() != v2.size())
    {
        // Handle error: vectors must have the same size
        return 0.0;
    }

    double result = 0.0;
    for (int i = 0; i < v1.size(); ++i)
    {
        result += v1[i] * v2[i];
    }
    return result;
}

void RobotEngine::printAxisMap()
{
    qDebug() << "Axis Map:";
    for (auto it = axis_map_.constBegin(); it != axis_map_.constEnd(); ++it) qDebug() << it.key() << ":" << it.value();
}

QMap<QString,double> RobotEngine::robotIK2P(double x1, double y1, double z1, double x2, double y2, double z2)
{
    double theta2_1=0,theta2_2=0,theta1_1=0,theta1_2=0,rx,ry,rz;
    double theta_1_ratio=50;
    double deg = M_PI / 180;
    double a = qCos(45.78 * deg);
    double b = -qSin(45.78 * deg);
    double insert_ration = 36;

    QVector<double> insert_dir_robot = {(x2 - x1), (y2 - y1), (z2 - z1)};

    // rcm
    QVector<double> insert_dir_rcm={insert_dir_robot[1]*(-1),insert_dir_robot[2]*(-1), insert_dir_robot[0]}; // robot frame --> rcm frame
    double insert_len = 0.0;
    for (double val : insert_dir_rcm)
    {
        insert_len += val * val;
    }
    insert_len = std::sqrt(insert_len);
    if(insert_len>0.1)
    {
        for(auto &d:insert_dir_rcm)
        {
            d /= insert_len;
        }

        QPair<double, double> theta2;
        theta2 = acbsc(b, a, insert_dir_rcm[2]);
        theta2_1 = theta2.first;
        theta2_2 = theta2.second;
        theta1_1 = qAtan2(insert_dir_rcm[0] / (b * qSin(theta2_1) - a * qCos(theta2_1)), insert_dir_rcm[1] / (a * qCos(theta2_1) - b * qSin(theta2_1)));
        theta1_2 = qAtan2(insert_dir_rcm[0] / (b * qSin(theta2_2) - a * qCos(theta2_2)), insert_dir_rcm[1] / (a * qCos(theta2_2) - b * qSin(theta2_2)));
        theta1_1 = theta1_1/deg;
        theta1_2 = theta1_2/deg;
        theta2_1 = theta2_1/deg;
        theta2_2 = theta2_2/deg;
    }

    // use calibrate axis
    QVector<double> translate_dir={x1-x0_,y1-y0_,z1-z0_};
    double dx = dotProduct(translate_dir,x_base);
    double dy = dotProduct(translate_dir,y_base);
    double dz = dotProduct(translate_dir,z_base);

    double k = 147;
    dz = qSqrt(28900.0-(k-dz)*(k-dz)) - qSqrt(28900.0-k*k);

    // robot frame --> xyz frame
    rx = -1*dx*120;
    ry = -1*dy*120;
    rz = dz*120;
    QMap<QString,double> s1 = {{"rx",rx}, {"ry",ry}, {"rz",rz}, {"r1",theta1_1*theta_1_ratio*(-1)}, {"r2",theta2_1},{"rn", insert_len*insert_ration}};
    QMap<QString,double> s2 = {{"rx",rx}, {"ry",ry}, {"rz",rz}, {"r1",theta1_2*theta_1_ratio*(-1)}, {"r2",theta2_2},{"rn", insert_len*insert_ration}};
    QMap<QString, double> s3 = {{"rx", 0}, {"ry", 0}, {"rz", 0}, {"r1", 0}, {"r2", 0}, {"rn", 0}};

    if (theta2_1 >= 0 && theta2_1 <= 70) {
        if (theta2_2 >= 0 && theta2_2 <= 70) {
            if (std::abs(theta1_1) < std::abs(theta1_2)) {
                return s1;
            } else {
                return s2;
            }
        } else {
            return s1;
        }
    } else if (theta2_2 >= 0 && theta2_2 <= 70) {
        return s2;
    } else {
        return s3;
    }
}

QMap<QString,double> RobotEngineLong::robotIK2P(double x1, double y1, double z1, double x2, double y2, double z2)
{
    const double deg = M_PI / 180.0;
    QMap<QString, double> result;

    result["rx"] = (-x1 + x0_) * 120.0; // theta1
    result["ry"] = (-y1 + y0_) * 120.0;
    result["rz"] = (135.5 - sqrt(28900.0 - pow(z1 - z0_, 2.0))) * 120.0; // theta3

    double d6 = sqrt(pow(x2 - x1, 2.0) + pow(y2 - y1, 2.0) + pow(z2 - z1, 2.0));
    result["r1"] =(atan2(-(z2 - z1) / d6, (y2 - y1) / d6) / deg - 90.0)*50.0;    // theta4 换蜗轮蜗杆记得乘以减速比50
    result["r2"] = acos(-(x2 - x1) / d6) / deg - 45.87; // theta5
    result["rn"] = sqrt(pow(x2 - x1, 2.0) + pow(y2 - y1, 2.0) + pow(z2 - z1, 2.0)) * 36.0; // theta6

    return result;
}

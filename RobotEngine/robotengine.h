#ifndef ROBOTENGINE_H
#define ROBOTENGINE_H
#include<iostream>
#include<QMap>

class  RobotEngine
{
public:
    RobotEngine(QString executableDir);

    virtual QMap<QString,double> robotIK2P(double x1, double y1, double z1, double x2, double y2, double z2);

    int axisToMotor(QString axis_name){return axis_map_[axis_name];}
    QString motorToAxis(int id){return axis_map_inv_[id];}
    void printAxisMap();
    void refreshAxisMap();
    void setHome(double x0, double y0, double z0){x0_=x0; y0_=y0; z0_=z0;}

private:
    QPair<double, double> acbsc(double a, double b, double c);
    double dotProduct(const QVector<double>& v1, const QVector<double>& v2);

private:
    QString executableDir_;
    QMap<QString,int> axis_map_;
    QMap<int,QString> axis_map_inv_;
protected:
    double x0_= -203.54;
    double y0_= 54.51;
    double z0_= 74.43;

    QVector<double> x_base={1,0,0};
    QVector<double> y_base={0,1,0};
    QVector<double> z_base={0,0,1};
};

class  RobotEngineLong : public RobotEngine
{
public:
    RobotEngineLong(QString executableDir):RobotEngine(executableDir) {}
    virtual QMap<QString,double> robotIK2P(double x1, double y1, double z1, double x2, double y2, double z2) override;

};

#endif // ROBOTENGINE_H

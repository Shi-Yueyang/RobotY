#ifndef UTILS_H
#define UTILS_H
#include <QByteArray>
#include <QString>
#include <QDebug>
#include <cmath>
#include <QtMath>
#include <QPair>

#define ANSI_COLOR_RED     "\033[1;31m"
#define ANSI_COLOR_GREEN     "\033[1;32m"
#define ANSI_COLOR_YELLOW  "\033[1;33m"
#define ANSI_COLOR_BLUE    "\033[1;34m"
#define ANSI_COLOR_MAGENTA "\033[1;35m"
#define ANSI_COLOR_CYAN    "\033[1;36m"
#define ANSI_COLOR_RESET   "\033[0m"

inline QByteArray toByteArray(QString Command)
{
    QByteArray byteArray;
    QStringList hexList = Command.split(" ", Qt::SkipEmptyParts);

    for (const QString& hex : hexList)
    {
        bool ok;
        byteArray.append(static_cast<char>(hex.toInt(&ok, 16)));

        if (!ok)
        {
            // Handle conversion error (invalid hex number)
            qDebug() << "Error: Invalid hex number -" << hex;
            break;
        }
    }
    return byteArray;
}

// for nanhang abs motor
//inline QString angleToHexStr(double value)
//{

//    // process value
//    if(value<0) value = -value;
//    if(value>350) value = 350;
//    if(value<10) value = 10;
//    value = fmod(value, 360.0);

//    // Convert value to integer in the range [0, 03FFFF]
//    int intValue = static_cast<int>((value / 360.0) * 0x3FFFF);
//    if(intValue==0)
//    {
//        intValue+=1;
//    }
//    // Convert integer to hexadecimal string with leading zeros
//    QString hexString = QString("%1").arg(intValue, 6, 16, QChar('0')).toUpper();

//    return hexString;
//}

inline QPair<double, double> acbsc(double a, double b, double c) {
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


inline QString formatList(const QList<double>& list) {
    QString result;
    QTextStream stream(&result);
    for (int i = 0; i < list.size(); ++i) {
        stream << QString::number(list[i]);
        if (i < list.size() - 1) {
            stream << ",";
        }
    }
    stream << ";";
    return result;
}

#endif // UTILS_H

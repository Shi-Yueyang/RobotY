#include <QCoreApplication>
#include<QDebug>
#include"myserver.h"

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    MyServer server;
    if (!server.listen(QHostAddress::Any, 1234))
    {
        // Handle error
        qDebug() << "Error starting server:" << server.errorString();
        return 1;
    }

    qDebug() << "Server started, listening on port" << server.serverPort();    return a.exec();
}

#ifndef MYSERVER_H
#define MYSERVER_H

#include "utils.h"
#include "motorkernel.h"
#include"robotengine.h"
#include <QTcpServer>
#include <QTcpSocket>


class Worker;

class MyServer : public QTcpServer {
    Q_OBJECT
public:
    MyServer(QObject *parent = nullptr);

protected:
    void incomingConnection(qintptr socketDescriptor) override;

private slots:
    void readyRead();
private:
    Worker* worker_;
};

#endif // MYSERVER_H

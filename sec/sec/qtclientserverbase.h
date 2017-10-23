#ifndef QTCLIENTSERVERBASE_H
#define QTCLIENTSERVERBASE_H

#include <QObject>
#include <QtNetwork/QLocalSocket>
#include <QDataStream>


namespace sec {


class QtClientBase : public QObject {

    Q_OBJECT

public:
    explicit QtClientBase(QObject *parent = nullptr);

    virtual ~QtClientBase();

protected:
    bool connectToServer(QString socketname);
    bool connectToServerAndWait(QString socketname);

    void disconnectFromServer();

    void write(const QString& str);

    QString readResponse();

    bool waitForConnected();


private:
    QLocalSocket* socket;

};


}

#endif // QTCLIENTSERVERBASE_H

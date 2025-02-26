#ifndef TPDOOBJECT_H
#define TPDOOBJECT_H

#include <QObject>

class TPDOObject : public QObject
{
    Q_OBJECT

    bool m_running;
    int count;

public:
    explicit TPDOObject(QObject *parent = nullptr);
    bool running() const;

signals:
    void changedData();
    void finished();
    void runningChanged(bool running);

public slots:
    void run();
    void setRunning(bool running);
};

#endif // TPDOOBJECT_H

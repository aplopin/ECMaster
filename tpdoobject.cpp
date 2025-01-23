#include "tpdoobject.h"
#include <QThread>

TPDOObject::TPDOObject(QObject *parent) : QObject(parent)
{

}

bool TPDOObject::running() const
{
    return m_running;
}

// Основной метод, в котором будет выолняться полезная рыбота объекта
void TPDOObject::run()
{
    count = 0;

    while(m_running)
    {
        count ++;

        emit changedData();
        QThread::msleep(250);
    }

    emit finished();
}

void TPDOObject::setRunning(bool running)
{
    if(m_running == running)
        return;

    m_running = running;
    emit runningChanged(running);
}


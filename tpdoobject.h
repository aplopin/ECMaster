#ifndef TPDOOBJECT_H
#define TPDOOBJECT_H

#include <QObject>

class TPDOObject : public QObject
{
    Q_OBJECT

    bool m_running; // Переменная управления полезного цикла метода run()
    int count; // Счетчик выполнения рыботы потока для контроля

public:
    explicit TPDOObject(QObject *parent = nullptr);
    bool running() const;

signals:
    void changedData(); // Сигнал для изменения данных
    void finished(); // Сигнал, по которму будет завершаться поток, после завершения метода run()
    void runningChanged(bool running);

public slots:
    void run(); // Метод с полезной нагрузкой, который может выполняться в цикле
    void setRunning(bool running);
};

#endif // TPDOOBJECT_H

#ifndef MESSAGE_H
#define MESSAGE_H
#include <QObject>
#include <QByteArray>

class Message
{
public:
    int sender;
    int task;
    qint64 bid_time;
    double winning_bid;
    int winning_agent;


    Message(int sender, int task, qint64 bid_time, double winning_bid,int winning_agent);
    QByteArray serialize();
    static Message* bytes_to_message(QByteArray& data);
};

#endif // MESSAGE_H

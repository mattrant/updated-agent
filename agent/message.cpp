#include "message.h"
#include <QByteArray>
Message::Message(int sender, int task, qint64 bid_time, double winning_bid, int winning_agent)
{
    this->sender = sender;
    this->task = task;
    this->bid_time = bid_time;
    this->winning_bid = winning_bid;
    this->winning_agent = winning_agent;
}

QByteArray Message::serialize(){
    //TODO: packetize the info
    QByteArray sender = QByteArray::number(this->sender);
    sender = sender.rightJustified(25,'0',true);
    QByteArray task = QByteArray::number(this->task);
    task = task.rightJustified(25,'0',false);
    QByteArray winning_bid = QByteArray::number(this->winning_bid,'g',10);
    winning_bid = winning_bid.rightJustified(25,'0',false);
    QByteArray bid_time = QByteArray::number(this->bid_time);
    bid_time = bid_time.rightJustified(25,'0',false);
    QByteArray winning_agent = QByteArray::number(this->winning_agent);
    winning_agent = winning_agent.rightJustified(25,'0',false);

    return sender+task+bid_time+winning_bid+winning_agent;
}

Message* Message::bytes_to_message(QByteArray &data){

    int sender = data.mid(0,25).toInt();
    int task = data.mid(25,25).toInt();
    qint64 bid_time = data.mid(50,25).toLongLong();
    double winning_bid = data.mid(75,25).toDouble();
    int winning_agent = data.mid(100,25).toInt();

    return new Message(sender,task,bid_time,winning_bid,winning_agent);

}

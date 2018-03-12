#ifndef BUILDER_H
#define BUILDER_H

#include <QThread>
#include <QVector>
#include <QHash>

#include "location.h"
#include "message.h"
#include "UBAgent.h"

class UBAgent;
class Builder: public QThread
{
    Q_OBJECT
public:
    void run();
private:


    UBAgent* agent;
    QVector<int> bundle;


    QHash<int,Message*> outgoing;

    QVector<int> winning_agents;
    QVector<double> winning_bids;
    QVector<qint64> bid_times;

    QVector<double> personal_bids;

    double score(QVector<int> path);

    void reset_task(int task);
    void update(Message* m);
    void forward_message(Message *m);
    void broadcast_task(int task);


public:

    int ID;

     QVector<int> path;

    void build_bundle();
    void resolve_conflicts();
    void broadcast_info();


    Builder(UBAgent* agent);

    int get_bundle_size(){return bundle.size();}
    QVector<int> get_path(){return path;}
    void print_bundle();
    void print_winning_agents();

    static qint64 get_nano_time();
signals:
    void newPath(QVector<int> path);
};

#endif // BUILDER_H

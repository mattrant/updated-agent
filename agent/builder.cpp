
#include "builder.h"

#include "location.h"
#include "UBAgent.h"
#include <QtMath>
#include <QtGlobal>
#include <QDateTime>
#include <QWaitCondition>

#include <iostream>
#include <fstream>

#include "UBConfig.h"

#include <QTimer>
#include <QCommandLineParser>

#include "Vehicle.h"
#include "TCPLink.h"
#include "MissionManager.h"
#include "QGCApplication.h"

/**
 * @brief Builder::Builder
 * @param agent agent that instantiated the thread
 *
 */
Builder::Builder(UBAgent* agent)
{
    this->agent = agent;

    agent->read_in_task_locations();

    winning_agents.resize(agent->task_locations.size());
    winning_bids.resize(agent->task_locations.size());
    bid_times.resize(agent->task_locations.size());
    personal_bids.resize(agent->task_locations.size());
    //initialize all info vectors
    for(int i=0;i<agent->task_locations.size();++i){
        winning_agents[i] = -1;
        winning_bids[i] = 0;
        bid_times[i] = 0;
        personal_bids[i] = 0;
    }


}

void write_locations_to_wp_file(QVector<Location*> locations){

    std::ofstream wp_file;
    wp_file.open("wp_file.txt");
    wp_file<<"QGC WPL 110"<<std::endl;

    for(int i=0;i<locations.size();++i){
        //waypoint file format: http://qgroundcontrol.org/mavlink/waypoint_protocol
        wp_file<<i<<"\t0\t3\t16\t0\t5\t0\t0\t"<<locations[i]->lat<<"\t"<<locations[i]->lon<<"\t10\t1"<<std::endl;
    }
    wp_file.close();
}

/**
 * @brief Builder::run
 * Performs the bundle building and conflict resolution rules as specified in the ACBBA documentation
 */
void Builder::run(){
    std::cout<<"ID: "<<ID<<" | Initializing ACBBA modules"<<std::endl;

    this->ID = agent->m_mav->id();
    //agent->init_task_locations(100);

    std::ofstream start_pos;
    start_pos.open("start_pos.txt");
    start_pos<<agent->m_mav->coordinate().latitude()<<" "<<agent->m_mav->coordinate().longitude();
    start_pos.close();

    agent->start_time = QDateTime::currentMSecsSinceEpoch();

    while(true){
        build_bundle();
        broadcast_info();
        resolve_conflicts();

    }
}
/**
 * @brief Builder::score
 * Calculates the score that would result from performing the tasks in the path in the
 * order that is present in the path. Assumes a complete task location graph. Uses the
 * scoring algorithm present in the ACBBA documentation
 *
 * @param path the path that should have it's score calculated
 * @return the score that would be accrued by performing the tasks along this path
 */
double Builder::score(QVector<int> path){


    double score = 0;
    double currentTime = 0;
    Location current_location(agent->m_mav->coordinate().latitude(),agent->m_mav->coordinate().latitude());
    for(int i =0;i<path.size();++i){
        Location next_location = *(agent->task_locations[path[i]]);

        double timeToNextTask = Location::distance(current_location,next_location)/MAX_SPEED;
        score+= qPow(TIME_DISCOUNT_FACTOR,currentTime+timeToNextTask)*REWARD;

        current_location = next_location;
        currentTime+=timeToNextTask;

    }


    return score;

}
/**
 * @brief Builder::build_bundle
 *
 * Constructs a bundle of tasks in order to maximize the score gained from performing the tasks. Places as
 * many tasks as allowed into the bundle as possible.
 */
void Builder::build_bundle(){

    QVector<int> altered;

    while(bundle.size() < MAX_ASSIGNMENTS){

        double current_path_score = score(path);

        for(int j =0 ;j<agent->task_locations.size();++j){
            //skip tasks that are already in the bundle
            if(bundle.contains(j)){
                continue;
            }

            double best_j_bid = 0;
            //insert task at best possible location
            for(int n =0;n<=path.size();++n){

                QVector<int> possible_path = path;
                if(n==path.size()){
                    possible_path.append(j);
                }
                else{
                    possible_path.insert(n,j);
                }

                double bid = score(possible_path) - current_path_score;
                best_j_bid = bid > best_j_bid ? bid : best_j_bid;
            }
            //save best bid for each task
            personal_bids[j] = best_j_bid;

        }

        //check to see if our bids are higher than the best known bids
        QVector<int> has_higher_bid;
        for(int j=0;j<agent->task_locations.size();++j){
            if(personal_bids[j] > winning_bids[j] && !bundle.contains(j)){
                has_higher_bid.append(j);
            }
            else if(personal_bids[j] == winning_bids[j] && ID <= winning_agents[j] && !bundle.contains(j)){
                has_higher_bid.append(j);
            }
        }

        //leave if all of our bids are lower than the best known bids
        if(has_higher_bid.size() == 0){
            break;
        }
        int selected_task = -1;
        double highest =0;
        //find task with best bid
        foreach (int task, has_higher_bid) {
            if(personal_bids[task] > highest){
                selected_task = task;
                highest = personal_bids[task];
            }
        }
        //determine best spot to insert the task in the path
        int best_n_pos = -1;
        double best_selected_bid = 0;

        for(int n = 0;n<=path.size();++n){

            QVector<int> possible_path = path;

            if(n == path.size()){
                possible_path.append(selected_task);
            }
            else{
                possible_path.insert(n,selected_task);
            }

            double bid = score(possible_path) - current_path_score;
            if(bid > best_selected_bid){
                best_selected_bid = bid;
                best_n_pos = n;
            }
        }

        if(best_n_pos == path.size()){
            path.append(selected_task);
        }
        else{
            path.insert(best_n_pos,selected_task);
        }
        altered.append(selected_task);
        bundle.append(selected_task);
        winning_bids[selected_task] = best_selected_bid;
        winning_agents[selected_task] = ID;
        bid_times[selected_task] = QDateTime::currentMSecsSinceEpoch();


    }
    //add changes to outgoing buffers
    foreach (int task, altered) {
        outgoing[task]=new Message(ID,task,bid_times[task],winning_bids[task],winning_agents[task]);
    }
    if(altered.size()!=0){
        print_bundle();
        emit newPath(path);
    }

}
/**
 * @brief Builder::reset_task
 * @param task
 */
void Builder::reset_task(int task){
    winning_agents[task] = -1;
    winning_bids[task] = 0;
}
/**
 * @brief Builder::forward_message
 * Places a message received from another agent into the outgoing buffer.
 *
 * @param m
 */
void Builder::forward_message(Message* m){
   //TODO: check if this is right
   outgoing[m->task] = m;



}
/**
 * @brief Builder::broadcast_task
 * Places a message containing local task info into the outgoing buffer
 *
 * @param task the task that should be broadcasted
 */
void Builder::broadcast_task(int task){
    //TODO: check if this is correct
    Message* m = new Message(ID,task,bid_times[task],winning_bids[task],winning_agents[task]);
    outgoing[task] = m;
}
/**
 * @brief Builder::update
 * Updates local winning bid and winning agent to reflect the new information that was
 * received from the network.
 *
 * @param m message containg the new information
 */
void Builder::update(Message *m){
    winning_agents[m->task] = m->winning_agent;
    winning_bids[m->task] = m->winning_bid;
    bid_times[m->task] = m->bid_time;
}

qint64 Builder::get_nano_time(){
    timespec ts;
    clock_gettime(CLOCK_REALTIME,&ts);
    qint64 t = ts.tv_sec * std::pow(10,9) + ts.tv_nsec;
    return t;

}
bool sort_messages(Message* a, Message*b){
    return a->bid_time < b->bid_time;
}

/**
 * @brief Builder::resolve_conflicts
 * Resolves conflicts that arise between agents regarding task allocation. This consensus phase
 * attempts to adjust the bundle to account for other agents and their bids for similar tasks. No two
 * agents should be assigned the same task.
 */
void Builder::resolve_conflicts(){


    //check for new messages
    agent->incoming_lock.lock();

    QVector<Message*> msg_buff = *(agent->incoming[agent->curr_buff]);

    agent->curr_buff = (agent->curr_buff+1)%2;
    agent->incoming[agent->curr_buff]->clear();

    agent->incoming_lock.unlock();


    QVector<Message*> messages;

    //prevent agents from having multiple messages from the same sender regarding the same task
    foreach(Message* msg,msg_buff){

        bool swapped = false;
        bool dont_insert = false;

        for(int i =0;i<messages.size();++i){
            if(messages[i]->sender == msg->sender && messages[i]->task == msg->task){
                //only keep the newest info
                if(messages[i]->bid_time < msg->bid_time){
                    messages[i] = msg;
                    swapped = true;
                    break;
                }
                else{
                    dont_insert = true;
                    break;
                }
            }
        }
        if(!swapped && !dont_insert){
            messages.append(msg);
        }

        //messages.append(msg);

    }
    //sort from lowest timestamp to highest
    std::sort(messages.begin(),messages.end(),sort_messages);
    foreach(Message* msg, messages){

        int task = msg->task;
        int my_winner = winning_agents[task];
        int sender_winner = msg->winning_agent;

        if(sender_winner == msg->sender && msg->sender!= ID){

            if(my_winner == ID){
                if(msg->winning_bid > winning_bids[task]){
                    update(msg);
                    forward_message(msg);
                }
                else if(msg->winning_bid < winning_bids[task]){
                    bid_times[task] = Builder::get_nano_time();
                    broadcast_task(task);
                }
                else if(msg->winning_bid == winning_bids[task] && msg->sender < my_winner){
                   update(msg);
                   forward_message(msg);
                }
                else if(msg->winning_bid == winning_bids[task] && msg->sender > my_winner){
                    bid_times[task] = Builder::get_nano_time();
                    broadcast_task(task);
                }
            }
            else if(my_winner == msg->sender){
                if(msg->bid_time > bid_times[task]){
                    update(msg);
                    forward_message(msg);
                }
                else if(msg->bid_time < bid_times[task]){
                    broadcast_task(task);
                }
                else if(msg->bid_time == bid_times[task]){
                    //forward_message(msg);
                }
                //forward_message(msg);
            }
            else if(my_winner != -1){

                if(msg->bid_time <= bid_times[task] && msg->winning_bid < winning_bids[task]){
                    broadcast_task(task);
                }
                else if(msg->bid_time >=bid_times[task] && msg->winning_bid > winning_bids[task]){
                    update(msg);
                    forward_message(msg);
                }
                else if(msg->winning_bid == winning_bids[task] && sender_winner < my_winner){
                    update(msg);
                    forward_message(msg);
                }
                else if(msg->winning_bid == winning_bids[task] && sender_winner > my_winner){
                    broadcast_task(task);
                }
                else if(msg->bid_time > bid_times[task] && msg->winning_bid < winning_bids[task]){

                    //allows agents to use higher bid times to change info to allow for correct winners to propagate
                    //update(msg);
                    //update(msg);
                    forward_message(msg);
                    //broadcast_task(task);
                }
                else if(msg->bid_time < bid_times[task] && msg->winning_bid > winning_bids[task]){
                    //update(msg);
                    forward_message(msg);
                    //broadcast_task(task);
                }
                else forward_message(msg);

            }
            else if(my_winner == -1){
                update(msg);
                forward_message(msg);
            }

        }
        else if(sender_winner == ID){
            if(my_winner == ID){


                if(msg->bid_time < bid_times[task]){
                    broadcast_task(task);
                }
                else if(msg->bid_time > bid_times[task]){
                    broadcast_task(task);
                }
            }
            else if(my_winner == msg->sender){
                reset_task(task);
                broadcast_task(task);
                //forward_message(msg);
            }
            else if(my_winner != -1){
//                Message* m = new Message(ID,task,Builder::get_nano_time(),0.0,ID);
//                forward_message(m);
                broadcast_task(task);
//                reset_task(task);
//                forward_message(msg);

                //broadcast_task(task);
                //reset_task(task);
                //forward_message(msg);
                //broadcast_task(task);
            }
            else if(my_winner == -1){
                bid_times[task] = Builder::get_nano_time();
                broadcast_task(task);
            }
        }
        else if(sender_winner != -1){
            if(my_winner == sender_winner){
                if(msg->bid_time > bid_times[task]){
                    update(msg);
                    forward_message(msg);
                }
                else if(msg->bid_time < bid_times[task]){
                    broadcast_task(task);
                    //forward_message(msg);
                }

            }
            else if(my_winner == msg->sender){
//                if(msg->bid_time > bid_times[task]){
//                    update(msg);
//                    forward_message(msg);
//                }
//                else if(msg->bid_time < bid_times[task]){
//                    broadcast_task(task);
//                }
                update(msg);
                forward_message(msg);


            }
            else if(my_winner == ID){
                if(msg->winning_bid > winning_bids[task]){
                    update(msg);
                    forward_message(msg);
                }
                else if(msg->winning_bid < winning_bids[task]){
                    bid_times[task] = Builder::get_nano_time();
                    broadcast_task(task);
                }
                else if(msg->winning_bid == winning_bids[task] && sender_winner < ID){
                    update(msg);
                    forward_message(msg);
                }
                else if(msg->winning_bid == winning_bids[task] && sender_winner > ID){
                    bid_times[task] = Builder::get_nano_time();
                    broadcast_task(task);
                }
            }
            else if(my_winner != -1 && my_winner != sender_winner){
                  //update(msg);
                  forward_message(msg);
            }
            else if(my_winner == -1){
                update(msg);
                forward_message(msg);
            }
        }
        else if (sender_winner == -1){
            if(my_winner == ID){
                bid_times[task] = Builder::get_nano_time();
                broadcast_task(task);
            }
            else if(my_winner == msg->sender){
                if(msg->bid_time > bid_times[task]){
                    update(msg);
                    forward_message(msg);
                }
                else if(msg->bid_time < bid_times[task]){
                    broadcast_task(task);
                    //forward_message(msg);
                }

            }

            else if(my_winner != -1){
                //broadcast_task(task);


                update(msg);
                forward_message(msg);

            }
            else if(my_winner == -1){
                forward_message(msg);
                //broadcast_task(task);
            }

        }

    }

    foreach (int task, bundle) {
        if(winning_agents[task] != ID){
            for(int j = bundle.indexOf(task)+1;j<bundle.size();++j){
                int removed_item = bundle[j];
                winning_bids[removed_item] = 0;
                winning_agents[removed_item] = -1;
                path.remove(path.indexOf(removed_item));
                qint64 new_bid_time =Builder::get_nano_time();
                outgoing[removed_item] = new Message(ID,removed_item,new_bid_time,winning_bids[removed_item],winning_agents[removed_item]);
            }
            path.remove(path.indexOf(task));
            //qint64 new_bid_time = Builder::get_nano_time();
            //outgoing[task] = new Message(ID,task,new_bid_time,winning_bids[task],winning_agents[task]);
            bundle = bundle.mid(0,bundle.indexOf(task));
            emit newPath(path);
            break;
        }
    }
}

/**
 * @brief Builder::broadcast_info
 * Sends out the contents of the outgoing message buffer to the network
 */
void Builder::broadcast_info(){
    //send all messages to BROADCAST
    if(outgoing.size()){
        //std::cout<<"ID: "<<ID<<" | Sending "<<outgoing.size()<<" messages..."<<std::endl;

    }
    for(QHash<int,Message*>::iterator i = outgoing.begin();i!=outgoing.end();++i){
        //needed for inter-thread use of a socket
        emit agent->sendData(BROADCAST_ID,i.value()->serialize());
    }
    outgoing.clear();

}
void Builder::print_bundle(){
    std::cout<<"ID: "<<ID<<" | ";
    for(int i =0;i<bundle.size();++i){
        std::cout<<bundle[i]<<" ";
    }
    std::cout<<std::endl;
}

void Builder::print_winning_agents(){
    std::cout<<"ID: "<<ID<<" | Winners: ";
    for(int i =0;i<winning_agents.size();++i){
        std::cout<<winning_agents[i]<<" ";
    }
    std::cout<<std::endl;
}

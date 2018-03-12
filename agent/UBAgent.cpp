#include "UBAgent.h"
#include "location.h"
#include "UBConfig.h"

#include "UBNetwork.h"
#include "LinkManager.h"

#include <QtMath>
#include <QWaitCondition>
#include <QSharedMemory>

#include <sys/file.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <fstream>

#include <execinfo.h>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>

#include <iostream>

#include "UBConfig.h"

#include <QTimer>
#include <QCommandLineParser>
#include <QtDebug>
#include "Vehicle.h"
#include "TCPLink.h"
#include "MissionManager.h"
#include "QGCApplication.h"

/**
 * @brief UBAgent::init_task_locations
 * Generates random locations for a set number of tasks
 * @param seed number used to get consistent randomness between agents
 */
void UBAgent::init_task_locations(unsigned seed){
      seed+=1;
      return;
//    qsrand(seed);
//    for(int i =0;i<TASKS;++i){
//        double base_lat = 43.000755; //field near UB North
//        double base_lon = -78.776023;

//        int rand_dist = qrand()%200;
//        double rand_angle = qrand()%361;
//        rand_angle = qDegreesToRadians(rand_angle);

//        projections::MercatorProjection proj;
//        core::Point pix1 = proj.FromLatLngToPixel(base_lat, base_lon, GND_RES);

//        Vector3d v1(pix1.X(), pix1.Y(), 0);
//        Vector3d v2(rand_dist*qCos(rand_angle),rand_dist*qSin(rand_angle),0);

//        Vector3d v = v1+v2;

//        internals::PointLatLng pll = proj.FromPixelToLatLng(v.x(), v.y(), GND_RES);

//        double lat = pll.Lat();
//        double lon = pll.Lng();
//        task_locations.append(new Location(lat,lon));
//    }
}

/**
 * @brief UBAgent::qInfo_task_locations
 *
 * Reads in the specified waypoint file and generates the list of task locations
 * based on the file's contents.
 */
void UBAgent::read_in_task_locations(){

    std::ifstream in_file;
    in_file.open("wp_file.txt");


    int task_num;
    int curr_wp,coord_frame,command,p1,p2,p3,p4;
    double lat,lon;
    int alt,other;

    //version info at beginning of file
    std::string x,y,z;
    in_file>>x>>y>>z;

    //waypoint file format: http://qgroundcontrol.org/mavlink/waypoint_protocol#waypoint_file_format
    while(in_file>>task_num>>curr_wp>>coord_frame>>command>>p1>>p2>>p3>>p4>>lat>>lon>>alt>>other){

        task_locations.append(new Location(lat,lon));

    }

    in_file.close();

}

/**
 * @brief UBAgent::UBAgent
 * @param parent
 */
UBAgent::UBAgent(QObject *parent) : QObject(parent),
    m_mav(nullptr)
{
    std::cout<<"Hello World\n";
    m_net = new UBNetwork();
    connect(m_net, SIGNAL(dataReady(quint32, QByteArray)), this, SLOT(dataReadyEvent(quint32, QByteArray)));
    connect(this,SIGNAL(sendData(quint32,QByteArray)),m_net,SLOT(sendData(quint32,QByteArray)));
    m_timer = new QTimer(this);
    m_timer->setInterval(MISSION_TRACK_RATE);
    connect(m_timer, SIGNAL(timeout()), this, SLOT(missionTracker()));

    incoming.resize(2);
    incoming[0] = new QVector<Message*>();
    incoming[1] = new QVector<Message*>();
    curr_buff = 0;

    builder = new Builder(this);
    connect(builder,SIGNAL(newPath(QVector<int>)),this,SLOT(pathChanged(QVector<int>)));

    current_task = -1;
    is_working= false;

    m_mission_stage = STAGE_IDLE;

    //needed in order to pass a QVector of ints through a signal/slot combo
    qRegisterMetaType<QVector<int> >("QVector<int>");

    startAgent();
}

/**
 * @brief UBAgent::startAgent
 */
 void UBAgent::startAgent() {
     QCommandLineParser parser;
     parser.setSingleDashWordOptionMode(QCommandLineParser::ParseAsLongOptions);

     parser.addOptions({
         {{"I", "instance"}, "Set instance (ID) of the agent", "id"},
     });

 //    parser.process(*QCoreApplication::instance());
     parser.parse(QCoreApplication::arguments());

     quint8 id = parser.value("I").toUInt();
     LinkConfiguration* link = nullptr;
     if (id) {
         quint32 port = 10 * id + STL_PORT + 3;
         TCPConfiguration* tcp = new TCPConfiguration(tr("TCP Port %1").arg(port));
         tcp->setAddress(QHostAddress::LocalHost);
         tcp->setPort(port);

         link = tcp;
     } else {
         SerialConfiguration* serial = new SerialConfiguration("Serial Port");
         serial->setBaud(BAUD_RATE);
         serial->setPortName(SERIAL_PORT);

         link = serial;
     }

     link->setDynamic();
     link->setAutoConnect();

     LinkManager* linkManager = qgcApp()->toolbox()->linkManager();
     linkManager->addConfiguration(link);
     linkManager->linkConfigurationsChanged();

     connect(qgcApp()->toolbox()->multiVehicleManager(), SIGNAL(vehicleAdded(Vehicle*)), this, SLOT(vehicleAddedEvent(Vehicle*)));
     connect(qgcApp()->toolbox()->multiVehicleManager(), SIGNAL(vehicleRemoved(Vehicle*)), this, SLOT(vehicleRemovedEvent(Vehicle*)));

     m_net->connectToHost(QHostAddress::LocalHost, 10 * id + NET_PORT);
     m_timer->start(MISSION_TRACK_RATE);
 }


void UBAgent::setMAV(Vehicle* mav) {
    if (m_mav) {
        disconnect(m_mav, SIGNAL(armedChanged(bool)), this, SLOT(armedChangedEvent(bool)));
        disconnect(m_mav, SIGNAL(flightModeChanged(QString)), this, SLOT(flightModeChangedEvent(QString)));
    }

    m_mav = mav;

    if (m_mav) {
        connect(m_mav, SIGNAL(armedChanged(bool)), this, SLOT(armedChangedEvent(bool)));
        connect(m_mav, SIGNAL(flightModeChanged(QString)), this, SLOT(flightModeChangedEvent(QString)));
    }
}

void UBAgent::vehicleAddedEvent(Vehicle* mav) {
    if (!mav || m_mav == mav) {
        return;
    }

    setMAV(mav);
    m_net->setID(mav->id());

    qInfo() << " connected with ID: " << m_mav->id();
}

void UBAgent::vehicleRemovedEvent(Vehicle* mav) {
    if (!mav || m_mav != mav) {
        return;
    }

    setMAV(nullptr);
    m_net->setID(0);

    qInfo() << "MAV disconnected with ID: " << mav->id();
}

void UBAgent::armedChangedEvent(bool armed) {
    Q_UNUSED(armed);
}

void UBAgent::flightModeChangedEvent(QString mode) {
    qInfo() << mode;
}
/**
 * @brief UBAgent::set_waypoint_to_location
 * Setts the current waypoint of the UAV to the specified location
 * @param l the location object that should be turned into a waypoint
 */
void UBAgent::set_waypoint_to_location(Location &l){

    QGeoCoordinate qp = QGeoCoordinate(l.lat,l.lon, 10);

    m_mav->missionManager()->writeArduPilotGuidedMissionItem(qp,false);
}


/**
 * @brief UBAgent::pathChanged
 *
 * Updates the list of current tasks to reflect the changes in the path
 *
 */
void UBAgent::pathChanged(QVector<int> path){


    work_lock.lock();
    QString  str = "ID: "+QString::number(m_mav->id())+" | New Path: ";
    std::ofstream ofs;
    ofs.open("update.txt");
    QString s = "";
    foreach(int task,path){
        s+=QString::number(task)+" ";
    }
    qInfo()<<str+s;
    ofs<<s.toStdString()<<std::endl;
    ofs.close();

    //write_changes(str);
    std::ofstream time_file;
    time_file.open("time.txt");
    time_file<<QDateTime::currentMSecsSinceEpoch() - start_time<<std::endl;
    time_file.close();





    if(!task_list.empty()){
        task_list.clear();
    }

    for(int i =0;i<path.size();++i){
        task_list.enqueue(path[i]);
    }

    if(path.isEmpty()){
        //Do not go to tasks. Stay at current location
        set_waypoint_to_location(*(new Location(m_mav->coordinate().latitude(),m_mav->coordinate().longitude())));
        work_lock.unlock();
        return;
    }
    if(!is_working){
        while(completed_tasks.contains(task_list.head())){
            task_list.dequeue();
        }
        current_task = task_list.dequeue();
        if(current_task!= -1){
            set_waypoint_to_location(*task_locations[current_task]);
        }
    }

    work_lock.unlock();






}

/**
 * @brief UBAgent::dataReadyEvent
 * Places information from the network into the incoming buffer for the builder thread
 * @param srcID
 * @param data
 */
void UBAgent::dataReadyEvent(quint32 srcID, QByteArray data) {

    //std::cout<<"Received Info from: "<<srcID<<std::endl;
    incoming_lock.lock();

    incoming[curr_buff]->append(Message::bytes_to_message(data));

    incoming_lock.unlock();


}



/**
 * @brief work
 * Performs hashcash type work to simulate some work being completed
 */
void UBAgent::work(){
    qInfo()<<"ID: "<<m_mav->id()<<" | Starting Task: "<<current_task;

    //work done
    work_lock.lock();
    completed_tasks.insert(current_task);
    qInfo()<<"ID: "<<m_mav->id()<<" | Finishing Task: "<<current_task;
    current_task =-1;
    is_working = false;
    work_lock.unlock();
    finished_task.wakeAll();

}

/**
 * @brief UBAgent::distance
 * Calculates the distance between two locations specified by lat,lon,alt coordinates
 * @param lat1
 * @param lon1
 * @param alt1
 * @param lat2
 * @param lon2
 * @param alt2
 * @return
 */
double UBAgent::distance(double lat1, double lon1, double alt1, double lat2, double lon2, double alt2) {

  QGeoCoordinate q1 = QGeoCoordinate(lat1,lon1,alt1);
  QGeoCoordinate q2 = QGeoCoordinate(lat2,lon2,alt2);

   return q1.distanceTo(q2);
}
/**
 * @brief UBAgent::inPointZone
 * Checks if the agent is within a certain distance of the desired location. All arguments are the coordinates
 * the location that the agent is approaching
 * @param lat
 * @param lon
 * @param alt
 * @return true if the agent is within POINT_ZONE meters of the specified location
 */
bool UBAgent::inPointZone(double lat, double lon, double alt) {
    double dist = distance(m_mav->coordinate().latitude(), m_mav->coordinate().longitude(), m_mav->altitudeRelative()->rawValue().toDouble(), lat, lon, alt);

    return dist < POINT_ZONE;
}
/**
 * @brief UBAgent::stage_mission
 * Carries out the actions required to complete missions after a list of tasks has been assembled
 */
void UBAgent::stage_mission(){
    //set waypoint to next task
    work_lock.lock();
    while(is_working){
        finished_task.wait(&work_lock);
    }
    //qInfo()<<"ID: "<<m_mav->id()<<" | Current Task: "<<current_task;
     if(current_task ==-1){
         //assign new task
         if(task_list.isEmpty()){
             //no more tasks
             work_lock.unlock();
             return;
         }
         while(completed_tasks.contains(task_list.head())){
             task_list.dequeue();
         }

         if(!task_list.isEmpty()){
             current_task = task_list.dequeue();
             if(current_task!=-1){
                 set_waypoint_to_location(*task_locations[current_task]);

             }
         }

     }
     else{
         //check if we have arrived at the work location
         if( current_task != -1){

}
         Location* curr_task_location = task_locations[current_task];
         if(inPointZone(curr_task_location->lat,curr_task_location->lon,10)){
             is_working = true;
             work_lock.unlock();
             //use work thread for immediate return?
             work();
             return;
         }
     }

     work_lock.unlock();
}
/**
 * @brief UBAgent::setup
 * Starts the ACBBA modules
 */
void UBAgent::setup(){
    builder->start();
    m_mission_stage = STAGE_MISSION;
    qInfo()<<"ID: "<<m_mav->id()<<" | Current Stage: MISSION";
}

/**
 * @brief UBAgent::stage_idle
 * Enusres that UAS has taken off before the mission begins
 */
void UBAgent::stage_idle(){
    if(m_mav->flightMode() !="Guided"){
          m_mav->setFlightMode("Guided");
    }
    else{
        m_mav->setArmed(true);
        m_mav->guidedModeTakeoff();
    }

}

/**
 * @brief UBAgent::missionTracker
 * Goes to the task location and performs the work at the location
 *
 */
void UBAgent::missionTracker() {

    if(!m_mav){
        return;
    }

    switch(m_mission_stage){
        case STAGE_IDLE:
//            if(m_uav->isArmed()){
//                m_mission_stage = STAGE_BEGIN;
//                qInfo()<<"ID: "<<m_mav->id()<<" | Current Stage: BEGIN";

//            }
//            else{
//                stage_idle();
//            }
            if(!m_mav->armed()){
                m_mav->setFlightMode("Guided");
                m_mav->setArmed(true);
            }
            else{

                m_mav->guidedModeTakeoff();
                m_mission_stage = STAGE_BEGIN;
            }
            break;
        default:
            break;
        case STAGE_BEGIN:
            setup();
        case STAGE_MISSION:
            stage_mission();
            break;
    }


}

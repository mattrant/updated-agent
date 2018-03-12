#include "location.h"
#include "QGCGeo.h"
#include <QtMath>
#include <QGeoCoordinate>

Location::Location(double lat, double lon)
{
    this->lat = lat;
    this->lon = lon;
}
double Location::distance(Location &a,Location &b){


    QGeoCoordinate q1 = QGeoCoordinate(a.lat,a.lon);
    QGeoCoordinate q2 = QGeoCoordinate(b.lat,b.lon);

    return q1.distanceTo(q2);
}

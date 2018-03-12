#ifndef LOCATION_H
#define LOCATION_H


class Location
{
public:
    double lat;
    double lon;
    Location(double lat, double lon);
    static double distance(Location &a,Location &b);
};

#endif // LOCATION_H

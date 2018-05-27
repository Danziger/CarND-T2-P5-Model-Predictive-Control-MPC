#ifndef HELPERS_H_
#define HELPERS_H_


#include <string>


// For portability of M_PI (Vis Studio, MinGW, etc.):

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


// To convert back and forth between radians and degrees:

#ifndef DEG_2_RAD
#define DEG_2_RAD(X) ( X * M_PI / 180 )
#endif

#ifndef RAD_2_DEG
#define RAD_2_DEG(X) ( X * 180 / M_PI )
#endif


using namespace std;


namespace helpers {

    /*
    * Checks if the SocketIO event has JSON data.
    * If there is data the JSON object in string format will be returned,
    * else the empty string "" will be returned.
    */
    string hasData(const string s);

    /**
    * Helper method to normalize angles that will hopefully be inlined by the compiler.
    */
    double normalizeAngle(double angle);

};

#endif /* HELPERS_H_ */

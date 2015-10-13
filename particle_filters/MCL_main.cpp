#include <iostream>
#include "MCL_particle_filter.hpp"

using namespace std;

const int NumParticle = 100*50;
const int SizeParticle = 3;
const int SizeMeasurement = 12;
const int SizeControl = 3;
typedef MCLParticleFilter<NumParticle, SizeParticle, SizeMeasurement, SizeControl> PF;
double random( double length ){ return (rand()*1.0/RAND_MAX)*length - length*0.5; }
double getdistance( double x , double y , double bx , double by ){ return sqrt( (x - bx)*(x - bx) + (y - by)*(y - by) );}
double gettheta( double x , double y , double ori , double bx , double by )
{
    double delta_x = bx - x;
    double delta_y = by - y;
    double beacon_theta = 0 ;

    if(      delta_x > 0 && delta_y >= 0) beacon_theta = atan(delta_y/delta_x);
    else if( delta_x < 0 && delta_y >= 0) beacon_theta = PI + atan(delta_y/delta_x);
    else if( delta_x < 0 && delta_y <  0) beacon_theta = PI + atan(delta_y/delta_x);
    else if( delta_x > 0 && delta_y <  0) beacon_theta = 2*PI + atan(delta_y/delta_x);
    else if( delta_x ==0 && delta_y >  0) beacon_theta = 0.5*PI;
    else if( delta_x ==0 && delta_y <  0) beacon_theta = 1.5*PI;

    double theta = beacon_theta - ori;
    if( delta_x > 0 && delta_y > 0 && ori > PI ) theta += 2*PI;
    else if( delta_x > 0 && delta_y < 0 && ori < PI) theta -= 2*PI;

    return theta;
}


int main()
{
    srand(time(NULL));

    PF::StateTransitionMatrix A;
    PF::ControlMatrix B;
    PF::MeasurementCovarianceMatrix Q;
    PF::WhiteCovarianceVector N;

    PF::ParticleVector p;


    A << 1 , 0 , 0 ,
         0 , 1 , 0 ,
         0 , 0 , 1 ;

    B << 1 , 0 , 0 , //Should be tuned by the real speed;
         0 , 1 , 0 ,
         0 , 0 , 1 ;

    Q << 400, 0     , 0     , 0     , 0     , 0     , 0     , 0     , 0     , 0     , 0     , 0     ,
         0  , 0.002 , 0     , 0     , 0     , 0     , 0     , 0     , 0     , 0     , 0     , 0     ,
         0  , 0     , 400   , 0     , 0     , 0     , 0     , 0     , 0     , 0     , 0     , 0     ,
         0  , 0     , 0     , 0.002 , 0     , 0     , 0     , 0     , 0     , 0     , 0     , 0     ,
         0  , 0     , 0     , 0     , 400   , 0     , 0     , 0     , 0     , 0     , 0     , 0     ,
         0  , 0     , 0     , 0     , 0     , 0.002 , 0     , 0     , 0     , 0     , 0     , 0     ,
         0  , 0     , 0     , 0     , 0     , 0     , 400   , 0     , 0     , 0     , 0     , 0     ,
         0  , 0     , 0     , 0     , 0     , 0     , 0     , 0.002 , 0     , 0     , 0     , 0     ,
         0  , 0     , 0     , 0     , 0     , 0     , 0     , 0     , 400   , 0     , 0     , 0     ,
         0  , 0     , 0     , 0     , 0     , 0     , 0     , 0     , 0     , 0.002 , 0     , 0     ,
         0  , 0     , 0     , 0     , 0     , 0     , 0     , 0     , 0     , 0     , 400   , 0     ,
         0  , 0     , 0     , 0     , 0     , 0     , 0     , 0     , 0     , 0     , 0     , 0.002 ;

    N << 50  , 0   , 0 ,
         0   , 50  , 0 ,
         0   , 0   , 0.1 ;

    PF  location_filter(A,B,Q,N);
    PF::MeasurementVector z;
    PF::ControlVector c;

    for( int i = 0 ; i < 8 ; ++i)
    {
        c << 50 , 0 , 0;
        p(0) = p(0) + c(0)*cos(c(2));

        double distance = getdistance(p(0),p(1),1500,1000);
        double angle = gettheta(p(0),p(1),p(2),1500,1000);
        if( i%2 == 1 )
            z << -1 , 0 ,
                 -1 , 0 ,
                 -1 , 0 ,
                 -1 , 0 ,
                 -1 , 0 ,
                 distance+random(40) , -angle+random(0.1) ;
        else
            z << distance+random(40) ,  angle+random(0.1) ,
                 -1 , 0 ,
                 -1 , 0 ,
                 -1 , 0 ,
                 -1 , 0 ,
                 -1 , 0 ;
        location_filter.process( z , c);
    }
//
    //double y_counter = 0;
    //double x_counter = 400;
    for( int i = 0 ; i < 4 ; ++i)
    {
        //action << 0 , 50 , 0,2
        c << -85*sin(p(2)+0.196) , 85*cos(p(2)+0.196) , 0.196;

        p(0) = p(0) + c(0);
        p(1) = p(1) + c(1);
        p(2) = p(2) + c(2);

        cout << "x = " << p(0) << "\ty = " << p(1) << "\ttheat = " << p(2) << endl;

        double distance = getdistance(p(0),p(1),1500,1000);
        double angle = gettheta(p(0),p(1),p(2),1500,1000);
        z << distance+random(40) , angle+random(0.1) ,
             -1 , 0 ,
             -1 , 0 ,
             -1 , 0 ,
             -1 , 0 ,
             -1 , 0 ;
        location_filter.process( z , c);
    }

    for( int i = 0 ; i < 4 ; ++i)
    {
        //action << 0 , 50 , 0,2
        c << -85*sin(p(2)+0.196) , 85*cos(p(2)+0.196) , 0.196;

        p(0) = p(0) + c(0);
        p(1) = p(1) + c(1);
        p(2) = p(2) + c(2);

        cout << "x = " << p(0) << "\ty = " << p(1) << "\ttheat = " << p(2) << endl;

        double distance = getdistance(p(0),p(1),0,1000);
        double angle = gettheta(p(0),p(1),p(2),0,1000);
        z << -1 , 0 ,
             distance+random(40) , angle+random(0.1) ,
             -1 , 0 ,
             -1 , 0 ,
             -1 , 0 ,
             -1 , 0 ;
        location_filter.process( z , c);
    }





//    z <<     -1 , 0 ,
//             -1 , 0 ,
//             -1 , 0 ,
//             -1 , 0 ,
//             -1 , 0 ,
//         1720 ,-0.620 ;
//    location_filter.process( z , c);


//    Eigen::Matrix< double, 3 , 1 > b ;
//    Eigen::Matrix< double, 1 , 2 > c ;
//    Eigen::Matrix< double, 2 , 2 > d ;
//    ParticleStateSet a;
//    a << 1,2,3,
//         0,1,0;
//    b << 5,6,7;
//    c << 5.0,6.0;
//    d << 1.0,0.0,
//         0.0,2.0;
//
//    d = d.Constant(0);
//
//    cout << d << endl;
//    cout << a/2.0 << endl;
//    cout << a.row(1)*b;
      return 0;
}

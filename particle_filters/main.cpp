
#include <iostream>
#include "RMCL_particle_filter.hpp"

using namespace std;

const int NumParticle = 50*25;
const int SizeParticle = 3;
const int SizeMeasurement = 12;
const int SizeControl = 3;
typedef RMCLParticleFilter<NumParticle, SizeParticle, SizeMeasurement, SizeControl> RPF;
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

    RPF::StateTransitionMatrix A;
    RPF::ControlMatrix B;
    RPF::MeasurementCovarianceMatrix Q;
    RPF::WhiteCovarianceVector N;

    RPF::ParticleVector p;


    A << 1 , 0 , 0 ,
         0 , 1 , 0 ,
         0 , 0 , 1 ;

    B << 1 , 0 , 0 , //Should be tuned by the real speed;
         0 , 1 , 0 ,
         0 , 0 , 1 ;

    Q << 600, 0     , 0     , 0     , 0     , 0     , 0     , 0     , 0     , 0     , 0     , 0     ,
         0  , 0.003 , 0     , 0     , 0     , 0     , 0     , 0     , 0     , 0     , 0     , 0     ,
         0  , 0     , 600   , 0     , 0     , 0     , 0     , 0     , 0     , 0     , 0     , 0     ,
         0  , 0     , 0     , 0.003 , 0     , 0     , 0     , 0     , 0     , 0     , 0     , 0     ,
         0  , 0     , 0     , 0     , 600   , 0     , 0     , 0     , 0     , 0     , 0     , 0     ,
         0  , 0     , 0     , 0     , 0     , 0.003 , 0     , 0     , 0     , 0     , 0     , 0     ,
         0  , 0     , 0     , 0     , 0     , 0     , 600   , 0     , 0     , 0     , 0     , 0     ,
         0  , 0     , 0     , 0     , 0     , 0     , 0     , 0.003 , 0     , 0     , 0     , 0     ,
         0  , 0     , 0     , 0     , 0     , 0     , 0     , 0     , 600   , 0     , 0     , 0     ,
         0  , 0     , 0     , 0     , 0     , 0     , 0     , 0     , 0     , 0.003 , 0     , 0     ,
         0  , 0     , 0     , 0     , 0     , 0     , 0     , 0     , 0     , 0     , 600   , 0     ,
         0  , 0     , 0     , 0     , 0     , 0     , 0     , 0     , 0     , 0     , 0     , 0.003 ;

    N << 40  , 0   , 0 ,
         0   , 40  , 0 ,
         0   , 0   , 0.08 ;

    RPF  location_filter(A,B,Q,N);
    RPF::MeasurementVector z;
    RPF::ControlVector c;
    cout << "stage 1 ==========================================\n";
    for( int i = 0 ; i < 8 ; ++i)
    {
        c << 50 , 0 , 0;
        p(0) = p(0) + c(0)*cos(c(2));

        ofstream fout("location.txt");
        fout << p(0) << ' ' << p(1) << ' ' << p(2) << endl;

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
    cout << "stage 2 ==========================================\n";
    //double y_counter = 0;
    //double x_counter = 400;
    for( int i = 0 ; i < 4 ; ++i)
    {
        //action << 0 , 50 , 0,2
        c << -85*sin(p(2)+0.196) , 85*cos(p(2)+0.196) , 0.196;

        p(0) = p(0) + c(0);
        p(1) = p(1) + c(1);
        p(2) = p(2) + c(2);

        //cout << "x = " << p(0) << "\ty = " << p(1) << "\ttheat = " << p(2) << endl;
        ofstream fout("location.txt");
        fout << p(0) << ' ' << p(1) << ' ' << p(2) << endl;

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
    cout << "stage 3 ==========================================\n";
    for( int i = 0 ; i < 4 ; ++i)
    {
        //action << 0 , 50 , 0,2
        c << -85*sin(p(2)+0.196) , 85*cos(p(2)+0.196) , 0.196;

        p(0) = p(0) + c(0);
        p(1) = p(1) + c(1);
        p(2) = p(2) + c(2);

        //cout << "x = " << p(0) << "\ty = " << p(1) << "\ttheat = " << p(2) << endl;

        ofstream fout("location.txt");
        fout << p(0) << ' ' << p(1) << ' ' << p(2) << endl;

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

//kidnapped
    p(0) = 300;
    p(1) = -550;
    p(2) = 1.5*PI-0.1;
    cout << "stage 4 ==========================================\n";
    for( int i = 0 ; i < 31 ; ++i)
    {
        //action << 0 , 50 , 0,2
        c << -20*sin(p(2)+0.05) , 20*cos(p(2)+0.05) , 0.05;

        p(0) = p(0) + c(0);
        p(1) = p(1) + c(1);
        p(2) = p(2) + c(2);
        ofstream fout("location.txt");
        fout << p(0) << ' ' << p(1) << ' ' << p(2) << endl;

        //cout << "x = " << p(0) << "\ty = " << p(1) << "\ttheat = " << p(2) << endl;
        //cout << "distance = " << distance << "\tangle = " << angle << endl;
        if( i%2 == 0)
        {
            double distance = getdistance(p(0),p(1),0.0,-1000.0);
            double angle = gettheta(p(0),p(1),p(2),0.0,-1000.0);
            z << -1 , 0 ,
                 -1 , 0 ,
                 -1 , 0 ,
                 -1 , 0 ,
                 distance+random(40) , angle+random(0.1) ,
                 -1 , 0 ;
        }
        else
        {
            double distance = getdistance(p(0),p(1),1500,-1000);
            double angle = gettheta(p(0),p(1),p(2),1500,-1000);
            //cout << "distance = " << distance << "\tangle = " << angle << endl;
            z << -1 , 0 ,
                 -1 , 0 ,
                 -1 , 0 ,
                 -1 , 0 ,
                 -1 , 0 ,
                 distance+random(40) , angle+random(0.1) ;
        }
        location_filter.process( z , c);
    }

//    int k = 5;
//    Eigen::ArrayXXf a2 = Eigen::ArrayXXf::Zero(3,k);
//    cout << a2 << endl;
//    cout << "stage 5 ==========================================\n";
//    for( int i = 0 ; i < 8 ; ++i)
//    {
//        //action << 0 , 50 , 0,2
//        c << -40*sin(p(2)+0.09) , 40*cos(p(2)+0.09) , 0.09;
//
//        p(0) = p(0) + c(0);
//        p(1) = p(1) + c(1);
//        p(2) = p(2) + c(2);
//        ofstream fout("location.txt");
//        fout << p(0) << ' ' << p(1) << ' ' << p(2) << endl;
//
//        //cout << "x = " << p(0) << "\ty = " << p(1) << "\ttheat = " << p(2) << endl;
//        double distance = getdistance(p(0),p(1),1500,-1000);
//        double angle = gettheta(p(0),p(1),p(2),1500,-1000);
//        //cout << "distance = " << distance << "\tangle = " << angle << endl;
//        z << -1 , 0 ,
//             -1 , 0 ,
//             -1 , 0 ,
//             -1 , 0 ,
//             -1 , 0 ,
//             distance+random(40) , angle+random(0.1) ;
//        location_filter.process( z , c);
//    }

//    cout << d << endl;
//    cout << a/2.0 << endl;
//    cout << a.row(1)*b;
      return 0;
}

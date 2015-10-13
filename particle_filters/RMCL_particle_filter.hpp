#ifndef RMCL_PARTICLE_FILTER_H
#define RMCL_PARTICLE_FILTER_H

//#include <Eigen/core>
#include "eigen/Eigen/dense"
#include "eigen/Eigen/core"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <fstream>
#include <iomanip>
#define PI 3.14159265358979323846

using namespace std;

template<int NumParticle, int SizeParticle, int SizeMeasurement, int SizeControl>
class RMCLParticleFilter
{
public:
    typedef Eigen::Matrix< double, SizeParticle , NumParticle> ParticleStateSet;
    typedef Eigen::Matrix< double, 1 , NumParticle> ParticleWeightSet;

    typedef Eigen::Matrix< double, SizeParticle , 1> ParticleVector;
    typedef Eigen::Matrix< double, SizeParticle , 1> WhiteNoiseVector;
    typedef Eigen::Matrix< double, SizeMeasurement , 1> MeasurementVector;
    typedef Eigen::Matrix< double, SizeControl , 1> ControlVector;

    typedef Eigen::Matrix< double, SizeParticle , SizeParticle> WhiteCovarianceVector;
    typedef Eigen::Matrix< double, SizeMeasurement , SizeMeasurement> MeasurementCovarianceMatrix;
    typedef Eigen::Matrix< double, SizeParticle , SizeParticle> StateTransitionMatrix;
    typedef Eigen::Matrix< double, SizeParticle , SizeControl> ControlMatrix;

    RMCLParticleFilter(StateTransitionMatrix A_matrix, ControlMatrix B_matrix, MeasurementCovarianceMatrix Q_matrix, WhiteCovarianceVector N_matrix )
    {
        X = ParticleStateSet :: Zero();
        W = ParticleWeightSet :: Constant(100);
        X_bar = ParticleStateSet :: Zero();

        A = A_matrix;
        B = B_matrix;
        N = N_matrix;
        Q = Q_matrix;

        field_length = 5000;
        field_width  = 2500;
        Wslow = 0.02;
        Wfast = 0.02;
        Waverage = 0;
        Alphaslow = 0.05;
        Alphafast = 0.65;

        for(int i = 0 ; i < NumParticle ; i++)
        {
            X(0,i) = random(field_length);
            X(1,i) = random(field_width);
            X(2,i) = random(2*PI)+PI;
        }
    }

  //Probabilistic Robotics, pg. 200, Table 8.2
    void process(MeasurementVector z, ControlVector u)
    {
        Eigen::Matrix< double , 2 , 2 > cov;
        Eigen::Matrix< double , 2 , 1 > beacon_location, z_tmp, x_tmp;
        WhiteNoiseVector noise;

        //Step1 sampling and moving
        for( int i = 0 ; i < NumParticle ; i++)
        {
            noise << random(N(0,0)) , random(N(1,1)) , random(N(2,2));
            X_bar.col(i) = A*X.col(i) + B*u + noise;

            if(X_bar(2,i) >= 2*PI) X_bar(2,i) -= 2*PI;
            else if(X_bar(2,i) < 0) X_bar(2,i) += 2*PI;
        }

        //Step2 calculating weight
        for( int i = 0 ; i < z.size() ; i+=2)
        {
            if(z(i) == -1) continue;
            switch(i){
                case 0 : beacon_location << 1500.0,  1000.0;break;
                case 2 : beacon_location << 0.0   ,  1000.0;break;
                case 4 : beacon_location <<-1500.0,  1000.0;break;
                case 6 : beacon_location <<-1500.0, -1000.0;break;
                case 8 : beacon_location << 0.0   , -1000.0;break;
                case 10: beacon_location << 1500.0, -1000.0;break;
            }
            for ( int j = 0 ; j < NumParticle ; j++)
            {
                double distance = getdistance( X_bar(0,j) , X_bar(1,j) , beacon_location(0) , beacon_location(1));
                double theta = gettheta(X_bar(0,j) , X_bar(1,j) , X_bar(2,j) , beacon_location(0) , beacon_location(1));
                z_tmp << z(i)       , z(i+1) ;
                x_tmp << distance   , theta  ;
                cov   << Q(i  ,i)   , Q(i  ,i+1) ,
                         Q(i+1,i)   , Q(i+1,i+1) ;
                W(j) *= gaussian2d(z_tmp,x_tmp,cov);

//                if(W(j)>1){
//                    cout << "============================================================" << endl;
//                    cout << "distance = " << distance << "\ttheta = " << theta << "\tX_bar(j) = " << X_bar(2,j) << "\tW(" << j << ")= "<< W(j)<< endl;
//                }
            }
        }

        Waverage = W.sum()/NumParticle;
        Wslow = (1-Alphaslow)*Wslow + Alphaslow*Waverage;
        Wfast = (1-Alphafast)*Wslow + Alphafast*Waverage;

        W /= W.sum(); //Normalize
        low_variance_sampler(); //Resampler

//      k_mean(&NAO_LOCATION, X);

    }

private:
    ParticleStateSet X;
    ParticleStateSet X_bar;
    ParticleWeightSet W;
    ParticleVector NAO_LOCATION;

    StateTransitionMatrix A;
    ControlMatrix B;
    MeasurementCovarianceMatrix Q; //measurement error
    WhiteCovarianceVector N;

    double Wslow;
    double Wfast;
    double Waverage;
    double Alphaslow;
    double Alphafast;

    int field_length;
    int field_width;

    double random( double length ){ return (rand()*1.0/RAND_MAX)*length - length*0.5; }

    double gaussian2d(Eigen::Matrix<double,2,1> z , Eigen::Matrix<double,2,1> x , Eigen::Matrix<double,2,2> cov)
        { return (1.0/sqrt(2*PI*cov.determinant()))* exp(-0.5 * (z-x).transpose() * cov.inverse() * (z-x)) ;}

    double getdistance( double x , double y , double bx , double by )
        { return sqrt( (x - bx)*(x - bx) + (y - by)*(y - by) );}

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

    void low_variance_sampler()
    {
        int i , j = 0;
        double max_p = 1.0 - (Wfast/Wslow);
        if(max_p < 0) max_p = 0;
        int num_random = NumParticle * max_p;
        int num_resample = NumParticle - num_random;

        cout << "max_p = " << max_p << "\tWaverage = " << Waverage <<endl;

        ParticleWeightSet c; c(0) = W(0);
        for( i = 1 ; i < NumParticle ; i++){ c(i) = c(i-1) + W(i);}

        double thres =  (rand()*1.0/RAND_MAX)/num_resample;
        for( i = 0 ; i < num_resample ; i++)
        {
            while( thres > c(j)) ++j;
            X.col(i) = X_bar.col(j);
            thres += 1.0/num_resample;
        }
        for( i = num_resample ; i < NumParticle ; i++)
        {//The distribution should be modified
            X(0,i) = random(field_length);
            X(1,i) = random(field_width);
            X(2,i) = random(2*PI)+PI;
        }

        W << W.Constant(100);

        //ofstream fout0("next_dots.txt");
        //for(int i = 0 ; i < NumParticle ; i++) fout0 << X(0,i) << '\t' << X(1,i) << '\t' << X(2,i) << '\n' ;
        //for( i = 0 ; i < NumParticle ; i++){cout << "c(" << i <<") = " << setprecision(15) << c(i) << endl ;}
    }

  //void k_mean(&ParticleVector L, ParticleStateSet X)
};

#endif //RMCL_PARTICLE_FILTER_H

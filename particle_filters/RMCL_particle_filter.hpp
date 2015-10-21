#ifndef RMCL_PARTICLE_FILTER_H
#define RMCL_PARTICLE_FILTER_H


#include <Eigen/Dense>
#include <math/Pose2D.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <iomanip>
#include "../core/localization/Particle.h"

#define PI 3.14159265358979323846
#define ThetaRatio 2500/PI

using namespace std;

template<int NumParticle, int SizeParticle, int SizeMeasurement, int SizeControl>
class RMCLParticleFilter
{
public:
    typedef Eigen::Matrix< double, SizeParticle , NumParticle> ParticleStateSet;
    typedef Eigen::Matrix< double, 1 , NumParticle> ParticleWeightSet;
    typedef Eigen::Matrix< int   , 1 , NumParticle> ParticleLabelSet;

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
        L = ParticleLabelSet :: Zero();
        W = ParticleWeightSet :: Constant(100);
        X_bar = ParticleStateSet :: Zero();

        A = A_matrix;
        B = B_matrix;
        N = N_matrix;
        Q = Q_matrix;

        field_length = 5000;
        field_width  = 2500;
        Wslow = 0.16;
        Wfast = 0.16;
        Waverage = 0;
        Alphaslow = 0.01;
        Alphafast = 0.05;



        for(int i = 0 ; i < NumParticle ; i++)
        {
            X(0,i) = random(field_length);
            X(1,i) = random(field_width);
            X(2,i) = random(2*PI)+PI;
        }

        NAO_LOCATION << 0 , 0 , 0 ;
    }

    void init(Point2D loc, float orientation)
    {
        for(int i = 0 ; i < NumParticle ; i++)
        {
            X(0,i) = random(field_length);
            X(1,i) = random(field_width);
            X(2,i) = random(2*PI)+PI;
        }

        NAO_LOCATION << loc.x , loc.y , orientation;
    }

  //Probabilistic Robotics, pg. 200, Table 8.2
    void process(MeasurementVector z, ControlVector u)
    {
        Eigen::Matrix< double , 2 , 2 > cov;
        Eigen::Matrix< double , 2 , 1 > beacon_location, z_tmp, x_tmp;

        bool flag = false;
        ControlVector c;
        //Step1 sampling and moving
        for( int i = 0 ; i < NumParticle ; i++)
        {
            double d_x = u(0)*cos(X(2,i)) - u(1)*sin(X(2,i));
            double d_y = u(0)*sin(X(2,i)) + u(1)*cos(X(2,i));
            double d_o = u(2);
            c << d_x , d_y , d_o;
            X_bar.col(i) = A*X.col(i) + B*c;

            if(X_bar(2,i) >= 2*PI) X_bar(2,i) -= 2*PI;
            else if(X_bar(2,i) < 0) X_bar(2,i) += 2*PI;
        }

        //Step2 calculating weight
        for( int i = 0 ; i < z.size() ; i+=2)
        {
            if(z(i) == -1) continue;
            flag = true;
            switch(i){
                case 0 : beacon_location << 1500.0,  1000.0;break;
                case 4 : beacon_location << 0.0   ,  1000.0;break;
                case 8 : beacon_location <<-1500.0,  1000.0;break;
                case 10: beacon_location <<-1500.0, -1000.0;break;
                case 6 : beacon_location << 0.0   , -1000.0;break;
                case 2 : beacon_location << 1500.0, -1000.0;break;
            }

            z_tmp << z(i)       , z(i+1) ;
            cov   << Q(i  ,i)   , Q(i  ,i+1) ,
                     Q(i+1,i)   , Q(i+1,i+1) ;

            for ( int j = 0 ; j < NumParticle ; j++)
            {
                double distance = getdistance( X_bar(0,j) , X_bar(1,j) , beacon_location(0) , beacon_location(1));
                double theta = gettheta(X_bar(0,j) , X_bar(1,j) , X_bar(2,j) , beacon_location(0) , beacon_location(1));

                x_tmp << distance   , theta  ;
                W(j) *= gaussian2d(z_tmp,x_tmp,cov);

            }
        }

        if(!flag)
        {
            X = X_bar;
            NAO_LOCATION = NAO_LOCATION + B*c;

            if(NAO_LOCATION(2) >= 2*PI) NAO_LOCATION(2) -= 2*PI;
            else if(NAO_LOCATION(2) < 0) NAO_LOCATION(2) += 2*PI;

            return;
        } 
        
        Waverage = W.sum()/NumParticle;
        Wslow = (1-Alphaslow)*Wslow + Alphaslow*Waverage;
        Wfast = (1-Alphafast)*Wslow + Alphafast*Waverage;

        W /= W.sum(); //Normalize
        low_variance_sampler(); //Resampler

        L = L.Zero();
        // cout << "Randomratio = "<< Randomratio << endl;
        NAO_LOCATION = kmeans( Randomratio ); // Get the best location of NAO
        //NAO_LOCATION = getAverage( Randomratio );
    }

    std::vector<Particle> getParticles()
    {
        std::vector<Particle> particles;

        for(int i = 0 ; i < NumParticle ; i++)
        {
            Particle p;
            p.x = X(0,i);
            p.y = X(1,i);
            p.t = X(2,i);
            p.w = 1;
            particles.push_back(p);
            // printf("p(x,y,t,w) = (%f,%f,%f,%f)\t i = %d\n", p.x , p.y , p.t , p.w , i);
        }

        return particles;
    }

ParticleVector getNAO_LOCATION(){return NAO_LOCATION;}

private:
    ParticleStateSet X;
    ParticleStateSet X_bar;
    ParticleWeightSet W;
    ParticleLabelSet L;
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
    double Randomratio;

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

        default_random_engine generator;

        normal_distribution<double> distributionx(0.0,N(0,0));
        normal_distribution<double> distributiony(0.0,N(1,1));
        normal_distribution<double> distributiono(0.0,N(2,2));

        Randomratio = 1.0 - (Wfast/Wslow);
        if(Randomratio < 0) Randomratio = 0;
        int num_random = NumParticle * Randomratio;
        int num_resample = NumParticle - num_random;

        ParticleWeightSet c; c(0) = W(0);
        for( i = 1 ; i < NumParticle ; i++){ c(i) = c(i-1) + W(i);}

        //for( i = 0 ; i < NumParticle ; i++){cout << "c(" << i <<") = " << setprecision(15) << c(i) << endl ;}

        double thres =  (rand()*1.0/RAND_MAX)/num_resample;
        WhiteNoiseVector noise;
        for( i = 0 ; i < num_resample ; i++)
        {
            while( thres > c(j)) ++j;

            //noise << random(N(0,0)) , random(N(1,1)) , random(N(2,2));
            noise << distributionx(generator),distributiony(generator),distributiono(generator);
            X.col(i) = X_bar.col(j) + noise;
            if(X(2,i) >= 2*PI) X(2,i) -= 2*PI;
            else if(X(2,i) < 0) X(2,i) += 2*PI;

            thres += 1.0/num_resample;
        }
        for( i = num_resample ; i < NumParticle ; i++)
            { X.col(i) << random(field_length) , random(field_width) , random(2*PI)+PI ; }

        W << W.Constant(100);

    }


    ParticleVector kmeans( double ratio )
    {
        int i , j , l = 0;
        ParticleVector nao_location;

        //1. decide a proper p;
        int num_random = NumParticle * ratio;
        int num_resample = NumParticle - num_random;
        int k = (int)(ratio*100);
        if( !k ) k = 1;
        if( k == 1)
        {
            double** p = getMeans(num_resample,1);
            nao_location << p[0][0] , p[0][1] , p[0][2];
            delete []p[0];
            delete []p;

            return nao_location;
        }
        //2. Initial first K center

        int *DistXYT = new int[num_resample];
        double **means  = new double*[k];
        for( i = 0 ; i < k ; ++i) means[i] = new double[SizeParticle];


        int first_means_index = rand()%num_resample ;
        for( i = 0 ; i < SizeParticle ; ++i) means[0][i] = X(i,first_means_index);

        double sum_DistXYT = 0;
        double random_bound = 0;
        for( i = 0 ; i < num_resample ; ++i)
        {
            DistXYT[i] = getDistXYT(X.col(i) , means[0]);
            sum_DistXYT += DistXYT[i];
        }
        random_bound = rand()*sum_DistXYT/RAND_MAX ;
        while( random_bound > 0) random_bound -= DistXYT[l++];
        for( i = 0 ; i < SizeParticle ; ++i) means[1][i] = X(i,l-1);

        for( j = 1 ; j < k ; ++j)
        {
            l = 0;
            sum_DistXYT = 0;
            for( i = 0 ; i < num_resample ; ++i)
            {
                double tmp_dist = getDistXYT( X.col(i) , means[j] );
                //cout << "tmp_dist = " << tmp_dist << endl;
                if(DistXYT[i] > tmp_dist)
                {
                    DistXYT[i] = tmp_dist;
                    L(i) = j;
                }
                sum_DistXYT += DistXYT[i];
            }
            random_bound = rand()*sum_DistXYT/RAND_MAX ;
            while( random_bound > 0) random_bound -= DistXYT[l++];
            for( i = 0 ; i < SizeParticle&&(j!=k-1) ; ++i) means[j+1][i] = X(i,l-1);
        }


        //3. Kmeans **center , X and L initialized
        double oldvar = -1;
        double newvar = getVar( num_resample , k , means);



        while( abs(newvar - oldvar) > 100 )
        {
            //cout << "newvar - oldvar =  " << newvar - oldvar << endl;
            for( i = 0 ; i < k ; ++i) delete []means[i];
            delete []means;

            means = getMeans( num_resample , k ) ;
            //for(j = 0 ; j < k ; ++j) printf("means process (x,y,z) = (%f,%f,%f) \n",means[j][0],means[j][1],means[j][2]);

            for( i = 0 ; i < num_resample ; ++i) L(i) = judgeCluster( i , k , means);
            oldvar = newvar;
            newvar = getVar( num_resample , k , means );
        }

        //4. Weighted average
        int *counter = new int[k];
        
        for( i = 0 ; i < k ; ++i) counter[i] = 0;
        for( i = 0 ; i < num_resample ; ++i ) ++counter[L(i)];


        int max_cluster_label = 0;
        int max_cluster_number= counter[0];
        for( i = 1 ; i < k ; ++i)
        {
            if( counter[i] > counter[i-1] )
            {
                max_cluster_label = i;
                max_cluster_number = counter[i];
            }
        }
        for( i = 0 ; i < SizeParticle ; ++i )
            nao_location(i) = means[max_cluster_label][i];

        // for( int i = 0 ; i < SizeParticle ; ++i )
        // {
        //     double tmp = 0;
        //     for( int j = 0 ; j < k ; ++j) tmp = tmp + counter[j]*means[j][i];
        //     nao_location(i) = tmp/num_resample;
        //     //cout << "!!!!!!!!!!!!!!  nao_location(" << i << ") = " << tmp/num_resample << endl;
        // }

        for( i = 0 ; i < k ; ++i) delete []means[i];
        delete []means;
        delete []counter;
        delete []DistXYT;
        return nao_location;
    }

    double getDistXYT( ParticleVector a , double *b)
        {    return sqrt( (a(0)-b[0])*(a(0)-b[0]) + (a(1)-b[1])*(a(1)-b[1]) + ThetaRatio*ThetaRatio*(a(2)-b[2])*(a(2)-b[2])) ;}

    double getVar( int length , int k , double **means )
    {
        double var = 0;
        for(int i = 0 ; i < length ; ++i)
        {
           //cout << "Get Var  == >   L(" << i << ") = " << L(i) << endl;
           double tmp = getDistXYT( X.col(i) , means[L(i)]) ;
           var +=  tmp*tmp;
        }
        return var;
    }

    double** getMeans( int length , int k)
    {
        int i , j;
        int *counter = new int[k];
        double **means = new double*[k];
        for( i = 0 ; i < k ; ++i) means[i] = new double[SizeParticle];
        for( i = 0 ; i < k ; ++i) counter[i] = 0;

        for( i = 0 ; i < k ; ++i )
            for( j = 0 ; j < SizeParticle ; ++j)
                means[i][j] = 0;

        for( i = 0 ; i < length ; ++i )
        {
            //cout << "Get Means  == >   L(" << i << ") = " << L(i) << endl;
            for( j = 0 ; j < SizeParticle ; ++j) means[L(i)][j] += X(j,i);
            ++counter[L(i)];
        }

        for( i = 0 ; i < k ; ++i) {
            for( j = 0 ; j < SizeParticle ; ++j) means[i][j] /= counter[i];
        }

        delete []counter;
        return means;
    }

    int judgeCluster( int index , int k , double **means)
    {
        double min_dist = getDistXYT( X.col(index) , means[0]);
        int label = 0;

        for( int i = 1 ; i < k ; ++i)
        {
            double tmp_dist = getDistXYT( X.col(index) , means[i]);
            if( min_dist > tmp_dist )
            {
                min_dist = tmp_dist;
                label = i ;
            }
        }
        return label;
    }

    ParticleVector getAverage( double ratio )
    {
        int i , j , l = 0;
        ParticleVector nao_location;

        //1. decide a proper p;
        int num_random = NumParticle * ratio;
        int num_resample = NumParticle - num_random;

        double counter[3] = {0};

        for( i = 0 ; i < num_resample ; ++i)
            for( j = 0 ; j < SizeParticle ; ++j)
                counter[j] += X(j,i);
        counter[0] /= num_resample;
        counter[1] /= num_resample;
        counter[2] /= num_resample;

        nao_location << counter[0] , counter[1] , counter[2] ;
        return nao_location;
    }

};

#endif //RMCL_PARTICLE_FILTER_H

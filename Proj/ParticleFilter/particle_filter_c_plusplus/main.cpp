/// main.cpp
/// this example shows how to use the C++ particle filter template

# include <iostream>
# include <fstream>
# include <cmath>
# include <chrono>
# include <random>
# include "setting.h"
# include "pfilter.h"  // include the template

// initialize the random seed
unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
std::default_random_engine generator (seed);

const precision_type PI = 3.14159265359;
const precision_type alpha = 0.91;
const precision_type beta = 1.0;

typedef double statetype;
typedef double obsvtype;

std::normal_distribution <statetype> distribution(0.0,1.0);

precision_type f(statetype x1, statetype x2){
    return exp(-0.5*pow((x1-alpha*x2),2));
}

precision_type g(statetype x, obsvtype y){
    return 1/exp(x/2)*exp(-0.5*pow(y/beta/exp(x/2),2));
}

precision_type q(statetype x1, statetype x2, obsvtype y){
    return exp(-0.5*pow((x1-alpha*x2),2));
}

statetype q_sam(statetype x, obsvtype y){
    return distribution(generator)+alpha*x;
}


int main(){
    using namespace std::chrono;
    ParticleFilter <statetype,obsvtype> A(f,g,q,q_sam);
    std::ifstream in("data_y");     // data input
    std::ofstream on("data_xhat");  // data output
    on.precision(15);

    A.initialize(2000);  // initialize with
                        // the number of particles we want to use
    high_resolution_clock::time_point t1 = high_resolution_clock::now();
    obsvtype y;
    statetype x;
    int iter_num = 0;
    while(in>>y)
    {
        x = A.update(y);
        on<<iter_num<<"\t"<<x<<std::endl;
        ++iter_num;
    }
    high_resolution_clock::time_point t2 = high_resolution_clock::now();

    duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
    std::cout << "It took " << time_span.count() << " seconds.";
    return 0;       //  we are done
}

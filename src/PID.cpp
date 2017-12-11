#include "PID.h"
#include <algorithm>
#include <iostream>
#include <math.h>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID( double Kp, double Ki, double Kd, double cap) : Kp( Kp ), Ki( Ki ), Kd( Kd ), cap(cap)
{
    p_error = 0.0;
    i_error = 0.0;
    d_error = 0.0;

    twiddle_error = 0;
    counter = 0;
}

PID::~PID() {}


void PID::UpdateError(double cte) {

    d_error = cte - p_error;
    i_error += cte;
    p_error = cte;
    
    twiddle_error += fabs(cte);
}

double PID::TotalError() {
	return std::max( -cap, std::min( -Kp * p_error - Kd * d_error - Ki * i_error, cap ) );
}

void PID::doRecalc(int interval, double threshold) {

    counter++;

    if (counter < interval) {                            
        return;
    }

    const double error = twiddle_error / interval;

    if (error < threshold){
        std::cout << "Found IT: Kp: " << Kp << " Ki: " << Ki << " Kd: " << Kd << std::endl;
        return;
    }

    counter = 0;

    const double multi =  1.2 * std::max(1.0, (twiddle_error / interval));
    Kp *= multi;
    Ki *= multi;
    Kd *= multi;

    std::cout << "Kp: " << Kp << " Ki: " << Ki << " Kd: " << Kd << std::endl;
              
    twiddle_error = 0;
}

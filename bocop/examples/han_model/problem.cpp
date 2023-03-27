#include <OCP.h>

template <typename Variable>
inline void OCP::finalCost(double initial_time, double final_time, const Variable *initial_state, const Variable *final_state, const Variable *parameters, const double *constants, Variable &final_cost)
{
    final_cost = -final_state[2];
};

template <typename Variable>
inline void OCP::dynamics(double time, const Variable *state, const Variable *control, const Variable *parameters, const double *constants, Variable *state_dynamics)
{

    // Constants
    double K       = constants[0];
    double sig     = constants[1];
    double a       = constants[2];
    double tau     = constants[3];
    double kd      = constants[4];
    double kr      = constants[5];
    double R       = constants[6];
    // Control variables
    Variable D      = control[0];
    Variable I      = control[1];
    // staes
    Variable x = state[0];
    Variable c = state[1];

    // State dynamics
    state_dynamics[0]    =     ( (K*sig*I*(1-c)*exp(-a*x) )/(1 + tau*sig*I*exp(-a*x)) )*x -(D+R)*x;           // dot{x}
    state_dynamics[1]    =     ( (kd*tau*pow(sig*I*exp(-a*x),2) )/(1 + tau*sig*I*exp(-a*x) ) )*(1-c) - kr*c;      // dot{c}
    state_dynamics[2]    =     x*D;   

};

template <typename Variable>
inline void OCP::boundaryConditions(double initial_time, double final_time, const Variable *initial_state, const Variable *final_state, const Variable *parameters, const double *constants, Variable *boundary_conditions)
{
    boundary_conditions[0] = initial_state[0];    //  x(0)
    boundary_conditions[1] = initial_state[1];    //  c(0)
    boundary_conditions[2] = initial_state[2];    //  Cr1(0) = 0
    //boundary_conditions[3] = final_state[0];      //  x(T)
    //boundary_conditions[4] = final_state[1];      //  c(T)
    //boundary_conditions[3] = final_state[2];      //  A MAXIMISER

};

template <typename Variable>
inline void OCP::pathConstraints(double time, const Variable *state, const Variable *control, const Variable *parameters, const double *constants, Variable *path_constraints)
{
};

void OCP::preProcessing()
{}


//
//
template void OCP::finalCost<double>(double initial_time, double final_time, const double *initial_state, const double *final_state, const double *parameters, const double *constants, double &final_cost);
template void OCP::dynamics<double>(double time, const double *state, const double *control, const double *parameters, const double *constants, double *state_dynamics);
template void OCP::boundaryConditions<double>(double initial_time, double final_time, const double *initial_state, const double *final_state, const double *parameters, const double *constants, double *boundary_conditions);
template void OCP::pathConstraints<double>(double time, const double *state, const double *control, const double *parameters, const double *constants, double *path_constraints);

template void OCP::finalCost<double_ad>(double initial_time, double final_time, const double_ad *initial_state, const double_ad *final_state, const double_ad *parameters, const double *constants, double_ad &final_cost);
template void OCP::dynamics<double_ad>(double time, const double_ad *state, const double_ad *control, const double_ad *parameters, const double *constants, double_ad *state_dynamics);
template void OCP::boundaryConditions<double_ad>(double initial_time, double final_time, const double_ad *initial_state, const double_ad *final_state, const double_ad *parameters, const double *constants, double_ad *boundary_conditions);
template void OCP::pathConstraints<double_ad>(double time, const double_ad *state, const double_ad *control, const double_ad *parameters, const double *constants, double_ad *path_constraints);

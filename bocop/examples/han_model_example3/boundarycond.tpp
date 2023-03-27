//  Project W. Djema, Joel Fierro, Olivier Bernard (Inria) - 10/2022
//  boundary Conditions
////////////////////////////////////////////////////////////////////////////////
#include "header_boundarycond"
{
    boundary_conditions[0] = initial_state[0];    //  x(0)
    boundary_conditions[1] = initial_state[1];    //  c(0)
    boundary_conditions[2] = initial_state[2];    //  Cr1(0) = 0
////////////////////////////////////////////////////////////////////////////////
    boundary_conditions[3] = final_state[0];      //  x(T)
    boundary_conditions[4] = final_state[1];      //  c(T)
    boundary_conditions[5] = final_state[2];      //  A MAXIMISER
}
////////////////////////////////////////////////////////////////////////////////

//  Project W. Djema, Joel Fierro, Olivier Bernard (Inria) - 10/2022
//  DYNAMICS

#include "header_dynamics"
{
// Constants
double K       = constants[0];
double sig     = constants[1];
double a       = constants[2];
double tau     = constants[3];
double kd      = constants[4];
double kr      = constants[5];
double R       = constants[6];
////////////////////////////////////////////////////////////////////////////////
// Control variables
Tdouble D      = control[0];
Tdouble I      = control[1];
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
  Tdouble x    = state[0];   // Biomass
  Tdouble c    = state[1];   // C state han model
  Tdouble Cr1  = state[2];   // criterion to integrate
  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////
  // State dynamics
  state_dynamics[0]    =     ( (K*sig*I*(1-c)*exp(-a*x) )/(1 + tau*sig*I*exp(-a*x)) )*x -(D+R)*x;           // dot{x}
  state_dynamics[1]    =     ( (kd*tau*pow(sig*I*exp(-a*x),2) )/(1 + tau*sig*I*exp(-a*x) ) )*(1-c) - kr*c;      // dot{c}
  state_dynamics[2]    =     x*D;                                                                           // Cr
  ///////////////////////////////
  ///////////////////////////////
}

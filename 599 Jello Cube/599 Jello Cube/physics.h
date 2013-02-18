/*

  CS 599, Physically Based Modeling for Interactive Simulation and Games
  USC/Viterbi/Computer Science
  Assignment 1 starter code

*/

#ifndef _PHYSICS_H_
#define _PHYSICS_H_

// Computes the hooks law in 3D and returns the force due to the spring of type springType connecting A and B 
point computeHooksForce(point A, point B, struct world * jello, int springType);

// Computes the damping force in 3D and returns the damping force due to the spring connecting A and B 
point computeDampingForce(point A, point B, struct world * jello);

// Checks wheather typeP point is inside jello cube
// Checks if the spring connecting curP and typeP is already parsed 
// Computes the hooks and damp force for the sprig connecting them
// Updates the force array with the force computed
void updateForce(point curP, point typeP, struct world * jello, int springType);

int checkIfAlreadyParsed(point typeP, point curP);

int checkIfInsideCube(point A);

void computeAcceleration(struct world * jello, struct point a[8][8][8]);

// perform one step of Euler and Runge-Kutta-4th-order integrators
// updates the jello structure accordingly
void Euler(struct world * jello);
void RK4(struct world * jello);

#endif


/*

  CS 599, Physically Based Modeling for Interactive Simulation and Games
  USC/Viterbi/Computer Science
  Assignment 1 starter code

*/

#include "jello.h"
#include "physics.h"
#include <cstring>

#define STRUCTURAL 1
#define SHEARSIDE 2
#define SHEARDIAGONAL 3
#define BEND 4
#define COLLISION 5
#define KELASTIC 6
#define KCOLLISION 7
#define DELASTIC 8
#define DCOLLISION 9
#define XMIN -2
#define XMAX 2
#define YMIN -2
#define YMAX 2
#define ZMIN -2
#define ZMAX 2
#define WALL 1
#define SPHERE 2

point force[8][8][8];	// keep updating the force for the nodes as and when 
						// forces due to different springs are calculated

node FFCube[8];			// stores the index of 8 nodes of the current force 
						// field cube being considered in the loop

point FFCubePos[8];		// stores the positions of the 8 nodes of the current FFCube

double forceFieldCubeSize = 0;	// length of each face of the cube in the force field
int noOfCubesInRow = 0;			// count of the number of FF cubes along one side of the bounding box

// Hook's Law in 3D
//				F = -k hook ( |L| - R) L / |L|	
// springType	1 = Structural
//				2 = Shear side
//				3 = Shear diagonal
//				4 = Bend
//				5 = Collision
point computeHooksForce(point A, point B, struct world * jello, int springType, int coEffType)
{
	
	point L, unitV;
	double mag = 0;
	double restLength = 0;
	point hooksForce;
	
	memset( (void*)&L, 0, sizeof(L));
	memset( (void*)&unitV, 0, sizeof(unitV));
	memset( (void*)&hooksForce, 0, sizeof(hooksForce));

	if(coEffType == KELASTIC)
	{
		L.x = jello->p[(int)A.x][(int)A.y][(int)A.z].x - jello->p[(int)B.x][(int)B.y][(int)B.z].x;
		L.y = jello->p[(int)A.x][(int)A.y][(int)A.z].y - jello->p[(int)B.x][(int)B.y][(int)B.z].y;
		L.z = jello->p[(int)A.x][(int)A.y][(int)A.z].z - jello->p[(int)B.x][(int)B.y][(int)B.z].z;

		mag = sqrt((L.x * L.x) + (L.y * L.y) + (L.z * L.z));

		unitV.x = L.x / mag;
		unitV.y = L.y / mag;
		unitV.z = L.z / mag;
		
		// Find the rest length for the current type of spring
		switch(springType)
		{
			case 1:	//Structural spring
				restLength = SRLength;
				break;
			case 2: //Shear side spring
				restLength = SHSideRLength;
				break;
			case 3: //Shear diagonal spring
				restLength = SHDiagonalRLength;
				break;
			case 4: //Bend spring
				restLength = BRLength;
				break;
			case 5: //Collision spring
				restLength = 0;
				break;
		}

		hooksForce.x = -(jello->kElastic) * (mag - restLength) * (unitV.x);
		hooksForce.y = -(jello->kElastic) * (mag - restLength) * (unitV.y);
		hooksForce.z = -(jello->kElastic) * (mag - restLength) * (unitV.z);
	}
	else if(coEffType == KCOLLISION)
	{
		L.x = jello->p[(int)A.x][(int)A.y][(int)A.z].x - B.x;
		L.y = jello->p[(int)A.x][(int)A.y][(int)A.z].y - B.y;
		L.z = jello->p[(int)A.x][(int)A.y][(int)A.z].z - B.z;

		mag = sqrt((L.x * L.x) + (L.y * L.y) + (L.z * L.z));

		unitV.x = L.x / mag;
		unitV.y = L.y / mag;
		unitV.z = L.z / mag;

		hooksForce.x = -(jello->kCollision) * (mag) * (unitV.x);
		hooksForce.y = -(jello->kCollision) * (mag) * (unitV.y);
		hooksForce.z = -(jello->kCollision) * (mag) * (unitV.z);
	}
	return hooksForce;
}


// Damping in 3D
//                F = -kd * ( va - vb ) . L / | L | * L / | L |
point computeDampingForce(point A, point B, struct world *jello, int coEffType)
{
	
	point L, unitV, vDiff;
	double mag = 0;
	double dotProd = 0;
	point dampingForce;
	point v1, v2;	// to store the velocities of both the points connected to the spring
	
	memset( (void*)&L, 0, sizeof(L));
	memset( (void*)&unitV, 0, sizeof(unitV));
	memset( (void*)&vDiff, 0, sizeof(vDiff));
	memset( (void*)&dampingForce, 0, sizeof(dampingForce));
	memset( (void*)&v1, 0, sizeof(v1));
	memset( (void*)&v2, 0, sizeof(v2));
	//printf("A.x = %lf, A.y = %lf, A.z = %lf, B.x = %lf ,B.y = %lf ,B.z = %lf\n", A.x,A.y,A.z,B.x,B.y,B.z);

	if(coEffType == DELASTIC)
	{

		L.x = jello->p[(int)A.x][(int)A.y][(int)A.z].x - jello->p[(int)B.x][(int)B.y][(int)B.z].x;
		L.y = jello->p[(int)A.x][(int)A.y][(int)A.z].y - jello->p[(int)B.x][(int)B.y][(int)B.z].y;
		L.z = jello->p[(int)A.x][(int)A.y][(int)A.z].z - jello->p[(int)B.x][(int)B.y][(int)B.z].z;
		//printf("L.x = %lf, L.y = %lf, L.z = %lf\n", L.x, L.y, L.z);
		
		// Magnitude
		mag = sqrt((L.x * L.x) + (L.y * L.y) + (L.z * L.z));
		//printf("mag = %lf\n", mag);
		
		// Normalise the vector
		unitV.x = L.x / mag;
		unitV.y = L.y / mag;
		unitV.z = L.z / mag;
		// printf("x = %lf  y = %lf  z = %lf\n", unitV.x, unitV.y, unitV.z); 

		// Difference in velocities
		v1.x = jello->v[(int)A.x][(int)A.y][(int)A.z].x;
		v1.y = jello->v[(int)A.x][(int)A.y][(int)A.z].y;
		v1.z = jello->v[(int)A.x][(int)A.y][(int)A.z].z;

		v2.x = jello->v[(int)B.x][(int)B.y][(int)B.z].x;
		v2.y = jello->v[(int)B.x][(int)B.y][(int)B.z].y;
		v2.z = jello->v[(int)B.x][(int)B.y][(int)B.z].z;

		vDiff.x = v1.x - v2.x;
		vDiff.y = v1.y - v2.y;
		vDiff.z = v1.z - v2.z;
		
		// vDiff Dot L
		dotProd = (vDiff.x * L.x) + (vDiff.y * L.y) + (vDiff.z * L.z);
		
		dampingForce.x = (-jello->dElastic) * ( dotProd / mag ) * (unitV.x);
		dampingForce.y = (-jello->dElastic) * ( dotProd / mag ) * (unitV.y);
		dampingForce.z = (-jello->dElastic) * ( dotProd / mag ) * (unitV.z);
	}
	else if(coEffType == DCOLLISION)
	{
		
		L.x = jello->p[(int)A.x][(int)A.y][(int)A.z].x - B.x;
		L.y = jello->p[(int)A.x][(int)A.y][(int)A.z].y - B.y;
		L.z = jello->p[(int)A.x][(int)A.y][(int)A.z].z - B.z;
		//printf("L.x = %lf, L.y = %lf, L.z = %lf\n", L.x, L.y, L.z);
		
		// Magnitude
		mag = sqrt((L.x * L.x) + (L.y * L.y) + (L.z * L.z));
		//printf("mag = %lf\n", mag);
		
		// Normalise the vector
		unitV.x = L.x / mag;
		unitV.y = L.y / mag;
		unitV.z = L.z / mag;
		// printf("x = %lf  y = %lf  z = %lf\n", unitV.x, unitV.y, unitV.z); 

		// Difference in velocities
		v1.x = jello->v[(int)A.x][(int)A.y][(int)A.z].x;
		v1.y = jello->v[(int)A.x][(int)A.y][(int)A.z].y;
		v1.z = jello->v[(int)A.x][(int)A.y][(int)A.z].z;

		// vDiff Dot L
		dotProd = (v1.x * L.x) + (v1.y * L.y) + (v1.z * L.z);

		dampingForce.x = (-jello->dCollision) * ( dotProd / mag ) * (unitV.x);
		dampingForce.y = (-jello->dCollision) * ( dotProd / mag ) * (unitV.y);
		dampingForce.z = (-jello->dCollision) * ( dotProd / mag ) * (unitV.z);
	}
	
	return dampingForce;
}	


int checkIfInsideCube(point A)
{

	if(A.x < 0 || A.x > 7 || A.y < 0 || A.y > 7 || A.z < 0 || A.z > 7)
		return 0;
	else
		return 1;

}


int checkIfAlreadyParsed(point typeP, point curP)
{

	if((typeP.x < curP.x) || (typeP.y < curP.y) || (typeP.z < curP.z))
		return 0;
	else 
		return 1;
}

// Checks wheather typeP point is inside jello cube
// Checks if the spring connecting curP and typeP is already parsed 
// Computes the hooks and damp force for the spring connecting them
// Updates the force array with the force computed
void updateForce(point curP, point typeP, struct world * jello, int springType)
{
	
	int inOrOut = 0;			// 1 = inside , 0 = outside
	int parsedOrNot = 0;		// 1 = unparsed, 0 = already parsed
	point hooksF, dampF;		// to store the force computed in the above defined functions
	
	memset( (void*)&hooksF, 0, sizeof(hooksF));
	memset( (void*)&dampF, 0, sizeof(dampF));

	inOrOut = checkIfInsideCube(typeP);

	if(inOrOut)		
	{
		// type# point is inside the jello cube
		parsedOrNot = checkIfAlreadyParsed(typeP, curP);

		if(parsedOrNot)
		{
			// type# point has not been parsed yet

			hooksF = computeHooksForce(curP, typeP, jello, springType, KELASTIC);
			dampF = computeDampingForce(curP, typeP, jello, DELASTIC);
			
			// Add the forces to the current point in the loop
			force[(int)curP.x][(int)curP.y][(int)curP.z].x += hooksF.x + dampF.x;
			force[(int)curP.x][(int)curP.y][(int)curP.z].y += hooksF.y + dampF.y;
			force[(int)curP.x][(int)curP.y][(int)curP.z].z += hooksF.z + dampF.z;
			
			// Add the forces to the current type# point
			force[(int)typeP.x][(int)typeP.y][(int)typeP.z].x += -hooksF.x + (-dampF.x);
			force[(int)typeP.x][(int)typeP.y][(int)typeP.z].y += -hooksF.y + (-dampF.y);
			force[(int)typeP.x][(int)typeP.y][(int)typeP.z].z += -hooksF.z + (-dampF.z);
		}
	}
}


void callWhenCollide(point curP, point wallP, struct world * jello, int springType, int prop)
{
	
	point hooksF, dampF;		
	
	memset( (void*)&hooksF, 0, sizeof(hooksF));
	memset( (void*)&dampF, 0, sizeof(dampF));

	hooksF = computeHooksForce(curP, wallP, jello, springType, KCOLLISION);
	dampF = computeDampingForce(curP, wallP, jello, DCOLLISION);
	
	if(prop == WALL)
	{
		// Add the forces to the current point in the loop
		force[(int)curP.x][(int)curP.y][(int)curP.z].x += (hooksF.x) + dampF.x + 0.9;
		force[(int)curP.x][(int)curP.y][(int)curP.z].y += (hooksF.y) + dampF.y + 0.9;
		force[(int)curP.x][(int)curP.y][(int)curP.z].z += (hooksF.z) + dampF.z + 0.9;
	}
	else if(prop == SPHERE)
	{
		// Add the forces to the current point in the loop
		force[(int)curP.x][(int)curP.y][(int)curP.z].x += (hooksF.x)/4 + dampF.x/2;
		force[(int)curP.x][(int)curP.y][(int)curP.z].y += (hooksF.y)/4 + dampF.y/2;
		force[(int)curP.x][(int)curP.y][(int)curP.z].z += (hooksF.z)/4 + dampF.z/2;

	}
}

void CheckForObjectCollision(point curP, struct world *jello)
{
	double dist = 0, slope = 0, c = 0, ratio = 0;
	point pos;
	point collisionP;

	memset( (void*)&pos, 0, sizeof(pos));
	pos.x = jello->p[(int)curP.x][(int)curP.y][(int)curP.z].x;
	pos.y = jello->p[(int)curP.x][(int)curP.y][(int)curP.z].y;
	pos.z = jello->p[(int)curP.x][(int)curP.y][(int)curP.z].z;

	// Compute the distance between the curP and the center of the sphere
	dist = sqrt( ((SPHEREx - pos.x)*(SPHEREx - pos.x)) + ((SPHEREy - pos.y)*(SPHEREy - pos.y)) + ((SPHEREz - pos.z)*(SPHEREz - pos.z)) );

	if(dist <= SPHEREr)
	{
		// jello colliding with the sphere object present in the bounding box
		
		ratio = SPHEREr / dist;

		collisionP.x = ratio * (SPHEREx - pos.x);
		collisionP.y = ratio * (SPHEREy - pos.y);
		collisionP.z = ratio * (SPHEREz - pos.z); 

		//------
/*
		dist = sqrt( ((SPHEREx - jello->p[4][4][4].x)*(SPHEREx - jello->p[4][4][4].x)) + ((SPHEREy - jello->p[4][4][4].y)*(SPHEREy - jello->p[4][4][4].y)) + ((SPHEREz - jello->p[4][4][4].z)*(SPHEREz - jello->p[4][4][4].z)) );
			
		ratio = SPHEREr / dist;

		collisionP.x = ratio * (SPHEREx - jello->p[4][4][4].x);
		collisionP.y = ratio * (SPHEREy - jello->p[4][4][4].y);
		collisionP.z = ratio * (SPHEREz - jello->p[4][4][4].z); 
*/
		//------
		callWhenCollide(curP, collisionP, jello, COLLISION, SPHERE);
		memset( (void*)&collisionP, 0, sizeof(collisionP));
	}
}

void checkForCollision(point curP, struct world *jello)
{

	point wallP;

	int x = (int)curP.x;
	int y = (int)curP.y;
	int z = (int)curP.z;

	memset( (void*)&wallP, 0, sizeof(wallP));
    
	if(sphereExists)
	{
		// Check if the jello collides with the object placed in the scene
		CheckForObjectCollision(curP, jello);
	}

	if(jello->p[x][y][z].x > 2)
	{	// collision with right face
		// RIGHT -> (2, y, z) structural spring
		wallP.x = 2;
		wallP.y = jello->p[x][y][z].y;
		wallP.z = jello->p[x][y][z].z;
		
		callWhenCollide(curP, wallP, jello, COLLISION, WALL);
		memset( (void*)&wallP, 0, sizeof(wallP));
	
	}

	if(jello->p[x][y][z].x < -2)
	{	// collision with left face
		// LEFT -> (-2, y, z) structural spring
		wallP.x = -2;
		wallP.y = jello->p[x][y][z].y;
		wallP.z = jello->p[x][y][z].z;
		
		callWhenCollide(curP, wallP, jello, COLLISION, WALL);
		memset( (void*)&wallP, 0, sizeof(wallP));
	
	}

	if(jello->p[x][y][z].y > 2)
	{	// collision with top face
		// TOP -> (x, 2, z) structural spring
		wallP.x = jello->p[x][y][z].x;
		wallP.y = 2;
		wallP.z = jello->p[x][y][z].z;
		
		callWhenCollide(curP, wallP, jello, COLLISION, WALL);
		memset( (void*)&wallP, 0, sizeof(wallP));
	
	}

	if(jello->p[x][y][z].y < -2)
	{	// collision with bottom face
		// BOTTOM -> (x, -2, z) structural spring
		wallP.x = jello->p[x][y][z].x;
		wallP.y = -2;
		wallP.z = jello->p[x][y][z].z;
		
		callWhenCollide(curP, wallP, jello, COLLISION, WALL);
		memset( (void*)&wallP, 0, sizeof(wallP));
	
	}

	if(jello->p[x][y][z].z > 2)
	{	// collision with back face
		// BACK -> (x, y, 2) structural spring
		wallP.x = jello->p[x][y][z].x;
		wallP.y = jello->p[x][y][z].y;
		wallP.z = 2;
		
		callWhenCollide(curP, wallP, jello, COLLISION, WALL);
		memset( (void*)&wallP, 0, sizeof(wallP));
	
	}

	if(jello->p[x][y][z].z < -2)
	{	// collision with front face
		// FRONT -> (x, y, -2) structural spring
		wallP.x = jello->p[x][y][z].x;
		wallP.y = jello->p[x][y][z].y;
		wallP.z = -2;
		
		callWhenCollide(curP, wallP, jello, COLLISION, WALL);
		memset( (void*)&wallP, 0, sizeof(wallP));
		
	}
}



void Interpolate(point curP, node curN, struct world *jello)
{

	// Interpolate all the 8 FFCube nodes to the position of the curJ node inside the FFCube Node
	point eightForces[8];
	int i = 0;
	double alpha = 0, beta = 0, gama = 0;
	double oneMAlpha = 0, oneMBeta = 0, oneMGama = 0;

	memset( (void*)&eightForces, 0, sizeof(eightForces));
	// can fetch the force from the force field array
	// compute alpha beta gama delta and their 1-alpha beta gama delta avlues
	// multiply and add them according to formula
	// add the final force to the J point force structure.
	
	for(i = 0; i < 8; i++)
	{
		eightForces[i].x = jello->forceField[FFCube[i].x * jello->resolution * jello->resolution 
			+ FFCube[i].y * jello->resolution + FFCube[i].z].x = 2; 
        eightForces[i].y = jello->forceField[i * jello->resolution * jello->resolution 
			+ FFCube[i].y * jello->resolution + FFCube[i].z].y = 1;
        eightForces[i].z = jello->forceField[i * jello->resolution * jello->resolution 
          + FFCube[i].y * jello->resolution + FFCube[i].z].z = 4;
	}

	// prefetch alpha, beta, gama and their compliments
	alpha = curP.x - FFCubePos[0].x;	//distance of the jello node from FFCube along x axis 
	beta = curP.y - FFCubePos[0].y;		//distance of the jello node from FFCube along y axis
	gama = curP.z - FFCubePos[0].z;		//distance of the jello node from FFCube along z axis

	oneMAlpha = forceFieldCubeSize - alpha;		// compliment of alpha
	oneMBeta = forceFieldCubeSize - beta;		// compliment of beta
	oneMGama = forceFieldCubeSize - gama;		// compliment of gama

	// F1
	eightForces[0].x *= oneMAlpha * oneMBeta * oneMGama;
	eightForces[0].y *= oneMAlpha * oneMBeta * oneMGama;
	eightForces[0].z *= oneMAlpha * oneMBeta * oneMGama;

	// F2
	eightForces[1].x *= alpha * oneMBeta * oneMGama;
	eightForces[1].y *= alpha * oneMBeta * oneMGama;
	eightForces[1].z *= alpha * oneMBeta * oneMGama;

	// F3
	eightForces[2].x *= oneMAlpha * oneMBeta * gama;
	eightForces[2].y *= oneMAlpha * oneMBeta * gama;
	eightForces[2].z *= oneMAlpha * oneMBeta * gama;

	// F4
	eightForces[3].x *= alpha * oneMBeta * gama;
	eightForces[3].y *= alpha * oneMBeta * gama;
	eightForces[3].z *= alpha * oneMBeta * gama;

	// F5
	eightForces[4].x *= oneMAlpha * beta * oneMGama;
	eightForces[4].y *= oneMAlpha * beta * oneMGama;
	eightForces[4].z *= oneMAlpha * beta * oneMGama;

	// F6
	eightForces[5].x *= alpha * beta * oneMGama;
	eightForces[5].y *= alpha * beta * oneMGama;
	eightForces[5].z *= alpha * beta * oneMGama;

	// F7
	eightForces[6].x *= oneMAlpha * beta * gama;
	eightForces[6].y *= oneMAlpha * beta * gama;
	eightForces[6].z *= oneMAlpha * beta * gama;

	// F8
	eightForces[7].x *= alpha * beta * gama;
	eightForces[7].y *= alpha * beta * gama;
	eightForces[7].z *= alpha * beta * gama;
	
	for(i = 0; i < 8; i++)
	{
		force[curN.x][curN.y][curN.z].x += eightForces[i].x;
		force[curN.x][curN.y][curN.z].y += eightForces[i].y;
		force[curN.x][curN.y][curN.z].z += eightForces[i].z;
	}
}

// Validates the Node passed as an argument
int checkNode(node N)
{

	if(N.x > noOfCubesInRow || N.x < 0 || N.y > noOfCubesInRow || N.y < 0 || N.z > noOfCubesInRow || N.z < 0)
		return 1;
	else 
		return 0;
}

// Validates the point passed as an argument
int checkPoint(point P)
{

	if(P.x > 2 || P.x < -2 || P.y > 2 || P.y < -2 || P.z > 2 || P.z < -2)
		return 1;
	else
		return 0;
}


void computeForceField(struct world *jello)
{

	// loop for all the nodes in the JCube, for every node check for the surrounding FFCube 
	// find the min and max of all the coords , find th jello node offset inside FFCube
	// find the FFCube 8 nodes positions ans node index values, validate them. Invoke interpolation
	// to find the value at the Jello node.
	
	int x = 0, y = 0, z = 0, X = 0, Y = 0, Z = 0;
	int xPos = 0, yPos = 0, zPos = 0, i = 0, flag = 0;
	point curP;
	node curN;

	for(x = 0; x < 8; x++)
		for(y = 0; y < 8; y++)
			for(z = 0; z < 8; z++)
			{
				for(i = 0; i < 8 ; i++)
				{
					memset( (void*)&FFCube[i], 0, sizeof(FFCube[i]));
					memset( (void*)&FFCubePos[i], 0, sizeof(FFCubePos[i]));
				}
				memset( (void*)&curP, 0, sizeof(curP));
				memset( (void*)&curN, 0, sizeof(curN));

				// position of the current jello node in the bounding box
				curP.x = jello->p[x][y][z].x;
				curP.y = jello->p[x][y][z].y;
				curP.z = jello->p[x][y][z].z;
				if(checkPoint(curP))
					continue;

				curN.x = x;
				curN.y = y;
				curN.z = z;
				if(checkNode(curN))
					continue;

				// find the jello node offset inside the surrounding FFCube
				xPos = fmod(jello->p[x][y][z].x, forceFieldCubeSize);
				yPos = fmod(jello->p[x][y][z].y, forceFieldCubeSize);
				zPos = fmod(jello->p[x][y][z].z, forceFieldCubeSize);

				X = jello->p[x][y][z].x - xPos;
				Y = jello->p[x][y][z].y - yPos;
				Z = jello->p[x][y][z].z - zPos;
				
				// Store the positions of all the 8 corners of the FF cube
				// left bottom front
				FFCubePos[0].x = X;
				FFCubePos[0].y = Y;
				FFCubePos[0].z = Z;
				if(checkPoint(FFCubePos[0]))
					continue;

				// right bottom front
				FFCubePos[1].x = X + forceFieldCubeSize;
				FFCubePos[1].y = Y;
				FFCubePos[1].z = Z;
				if(checkPoint(FFCubePos[1]))
					continue;

				// left bottom back
				FFCubePos[2].x = X;
				FFCubePos[2].y = Y;
				FFCubePos[2].z = Z + forceFieldCubeSize;
				if(checkPoint(FFCubePos[2]))
					continue;

				// right bottom back
				FFCubePos[3].x = X + forceFieldCubeSize;
				FFCubePos[3].y = Y;
				FFCubePos[3].z = Z + forceFieldCubeSize;
				if(checkPoint(FFCubePos[3]))
					continue;

				// left top front
				FFCubePos[4].x = X;
				FFCubePos[4].y = Y + forceFieldCubeSize;
				FFCubePos[4].z = Z;
				if(checkPoint(FFCubePos[4]))
					continue;

				// right top front
				FFCubePos[5].x = X + forceFieldCubeSize;
				FFCubePos[5].y = Y + forceFieldCubeSize;
				FFCubePos[5].z = Z;
				if(checkPoint(FFCubePos[5]))
					continue;

				// left top back
				FFCubePos[6].x = X;
				FFCubePos[6].y = Y + forceFieldCubeSize;
				FFCubePos[6].z = Z + forceFieldCubeSize;
				if(checkPoint(FFCubePos[6]))
					continue;

				// right top back
				FFCubePos[7].x = X + forceFieldCubeSize;
				FFCubePos[7].y = Y + forceFieldCubeSize;
				FFCubePos[7].z = Z + forceFieldCubeSize;
				if(checkPoint(FFCubePos[7]))
					continue;
				
				for(i = 0; i < 8; i++)
				{
					FFCube[i].x = FFCubePos[i].x / forceFieldCubeSize;
					FFCube[i].y = FFCubePos[i].y / forceFieldCubeSize;
					FFCube[i].z = FFCubePos[i].z / forceFieldCubeSize;
					if(checkNode(FFCube[i]))
					{
						flag = 1; 
						break;
					}
				}
				if(flag)
					continue;

				Interpolate(curP, curN, jello);
			}
}


// Computes acceleration to every control point of the jello cube, 
//   which is in state given by 'jello'.	
//   Returns result in array 'a'.									
void computeAcceleration(struct world * jello, struct point a[8][8][8])
{
  // for you to implement ...  

	int x = 0, y = 0, z = 0;	// Index variables for looping through all 512 points

	point typeP, curP;			// typeP = one of the 32 nodes to which the curP point is connected
	
	int collided = 0;

	memset( (void*)a, 0, sizeof(a));
	memset( (void*)&typeP, 0, sizeof(typeP));
	memset( (void*)&curP, 0, sizeof(curP));
	memset( (void*)force, 0, sizeof(force));
	
	// FORCE FIELD COMPUTATION
	if(ActivateFF)
	{
		// Check for force field
		if(jello->resolution)
		{
			// compute the force field cube size and store it in the global variable
			forceFieldCubeSize = ((double)4)/jello->resolution;
			noOfCubesInRow = (int)(((double)jello->resolution) / forceFieldCubeSize);

			// Compute the effect of force field and update the forces due to it on all the nodes
			computeForceField(jello);
		}
	}

	// COLLISION DETECTION AND RESPONSE
	for(z = 0; z < 8; z++)
	{
		for(y = 0; y < 8; y++)
		{
			for(x = 0; x < 8; x++)
			{
				memset( (void*)&curP, 0, sizeof(curP));
				curP.x = x;
				curP.y = y;
				curP.z = z;

				// Checks whether any of the nodes is colliding with the bounding box and Sphere
				checkForCollision(curP, jello);

			}
		}
	}

	for(z = 0; z < 8; z++)
	{
		for(y = 0; y < 8; y++)
		{
			for(x = 0; x < 8; x++)
			{
				curP.x = x;
				curP.y = y;
				curP.z = z;

				// Check for all the springs possible = 32
				
				// SAME HORIZONTAL LAYER
				//-----------------------------------------------------------------
				// TYPE 1 -> (x+1, y, z) structural spring
				typeP.x = x + 1;
				typeP.y = y;
				typeP.z = z;
				
				updateForce(curP, typeP, jello, STRUCTURAL);
				memset( (void*)&typeP, 0, sizeof(typeP));
				//-----------------------------------------------------------------
				// TYPE 2 -> (x-1, y, z)
				typeP.x = x - 1;
				typeP.y = y;
				typeP.z = z;
				
				updateForce(curP, typeP, jello, STRUCTURAL);
				memset( (void*)&typeP, 0, sizeof(typeP));
				//-----------------------------------------------------------------
				// TYPE 3 -> (x, y, z+1)
				typeP.x = x;
				typeP.y = y;
				typeP.z = z + 1;
				
				updateForce(curP, typeP, jello, STRUCTURAL);
				memset( (void*)&typeP, 0, sizeof(typeP));
				//-----------------------------------------------------------------
				// TYPE 4 -> (x, y, z-1)
				typeP.x = x;
				typeP.y = y;
				typeP.z = z - 1;
				
				updateForce(curP, typeP, jello, STRUCTURAL);
				memset( (void*)&typeP, 0, sizeof(typeP));
				//-----------------------------------------------------------------
				// TYPE 5 -> (x+1, y, z+1)
				typeP.x = x + 1;
				typeP.y = y;
				typeP.z = z + 1;
				
				updateForce(curP, typeP, jello, SHEARSIDE);
				memset( (void*)&typeP, 0, sizeof(typeP));
				//-----------------------------------------------------------------
				// TYPE 6 -> (x+1, y, z-1)
				typeP.x = x + 1;
				typeP.y = y;
				typeP.z = z - 1;
				
				updateForce(curP, typeP, jello, SHEARSIDE);
				memset( (void*)&typeP, 0, sizeof(typeP));
				//-----------------------------------------------------------------
				// TYPE 7 -> (x-1, y, z+1)
				typeP.x = x - 1;
				typeP.y = y;
				typeP.z = z + 1;
				
				updateForce(curP, typeP, jello, SHEARSIDE);
				memset( (void*)&typeP, 0, sizeof(typeP));
				//-----------------------------------------------------------------
				// TYPE 8 -> (x-1, y, z-1)
				typeP.x = x - 1;
				typeP.y = y;
				typeP.z = z - 1;
				
				updateForce(curP, typeP, jello, SHEARSIDE);
				memset( (void*)&typeP, 0, sizeof(typeP));
				//-----------------------------------------------------------------

				// BOTTOM LAYER

				//-----------------------------------------------------------------
				// TYPE 9 -> (x+1, y+1, z)
				typeP.x = x + 1;
				typeP.y = y + 1;
				typeP.z = z;
				
				updateForce(curP, typeP, jello, SHEARSIDE);
				memset( (void*)&typeP, 0, sizeof(typeP));
				//-----------------------------------------------------------------
				// TYPE 10 -> (x-1, y+1, z)
				typeP.x = x - 1;
				typeP.y = y + 1;
				typeP.z = z;
				
				updateForce(curP, typeP, jello, SHEARSIDE);
				memset( (void*)&typeP, 0, sizeof(typeP));
				//-----------------------------------------------------------------
				// TYPE 11 -> (x, y+1, z+1)
				typeP.x = x;
				typeP.y = y + 1;
				typeP.z = z + 1;
				
				updateForce(curP, typeP, jello, SHEARSIDE);
				memset( (void*)&typeP, 0, sizeof(typeP));
				//-----------------------------------------------------------------
				// TYPE 12 -> (x, y+1, z-1)
				typeP.x = x;
				typeP.y = y + 1;
				typeP.z = z - 1;
				
				updateForce(curP, typeP, jello, SHEARSIDE);
				memset( (void*)&typeP, 0, sizeof(typeP));
				//-----------------------------------------------------------------
				// TYPE 13 -> (x+1, y+1, z+1)
				typeP.x = x + 1;
				typeP.y = y + 1;
				typeP.z = z + 1;
				
				updateForce(curP, typeP, jello, SHEARDIAGONAL);
				memset( (void*)&typeP, 0, sizeof(typeP));
				//-----------------------------------------------------------------
				// TYPE 14 -> (x+1, y+1, z-1)
				typeP.x = x + 1;
				typeP.y = y + 1;
				typeP.z = z - 1;
				
				updateForce(curP, typeP, jello, SHEARDIAGONAL);
				memset( (void*)&typeP, 0, sizeof(typeP));
				//-----------------------------------------------------------------
				// TYPE 15 -> (x-1, y+1, z+1)
				typeP.x = x - 1;
				typeP.y = y + 1;
				typeP.z = z + 1;
				
				updateForce(curP, typeP, jello, SHEARDIAGONAL);
				memset( (void*)&typeP, 0, sizeof(typeP));
				//-----------------------------------------------------------------
				// TYPE 16 -> (x-1, y+1, z-1)
				typeP.x = x - 1;
				typeP.y = y + 1;
				typeP.z = z - 1;
				
				updateForce(curP, typeP, jello, SHEARDIAGONAL);
				memset( (void*)&typeP, 0, sizeof(typeP));
				//-----------------------------------------------------------------
				// TYPE 17 -> (x, y+1, z)
				typeP.x = x;
				typeP.y = y + 1;
				typeP.z = z;
				
				updateForce(curP, typeP, jello, STRUCTURAL);
				memset( (void*)&typeP, 0, sizeof(typeP));
				//-----------------------------------------------------------------

				// TOP LAYER

				// TYPE 18 -> (x+1, y-1, z)
				typeP.x = x + 1;
				typeP.y = y - 1;
				typeP.z = z;
				
				updateForce(curP, typeP, jello, SHEARSIDE);
				memset( (void*)&typeP, 0, sizeof(typeP));
				//-----------------------------------------------------------------
				// TYPE 19 -> (x-1, y-1, z)
				typeP.x = x - 1;
				typeP.y = y - 1;
				typeP.z = z;
				
				updateForce(curP, typeP, jello, SHEARSIDE);
				memset( (void*)&typeP, 0, sizeof(typeP));
				//-----------------------------------------------------------------
				// TYPE 20 -> (x, y-1, z+1)
				typeP.x = x;
				typeP.y = y - 1;
				typeP.z = z + 1;
				
				updateForce(curP, typeP, jello, SHEARSIDE);
				memset( (void*)&typeP, 0, sizeof(typeP));
				//-----------------------------------------------------------------
				// TYPE 21 -> (x, y-1, z-1)
				typeP.x = x;
				typeP.y = y - 1;
				typeP.z = z - 1;
				
				updateForce(curP, typeP, jello, SHEARSIDE);
				memset( (void*)&typeP, 0, sizeof(typeP));
				//-----------------------------------------------------------------
				// TYPE 22 -> (x+1, y-1, z+1)
				typeP.x = x + 1;
				typeP.y = y - 1;
				typeP.z = z + 1;
				
				updateForce(curP, typeP, jello, SHEARDIAGONAL);
				memset( (void*)&typeP, 0, sizeof(typeP));
				//-----------------------------------------------------------------
				// TYPE 23 -> (x+1, y-1, z-1)
				typeP.x = x + 1;
				typeP.y = y - 1;
				typeP.z = z - 1;
				
				updateForce(curP, typeP, jello, SHEARDIAGONAL);
				memset( (void*)&typeP, 0, sizeof(typeP));
				//-----------------------------------------------------------------
				// TYPE 24 -> (x-1, y-1, z+1)
				typeP.x = x - 1;
				typeP.y = y - 1;
				typeP.z = z + 1;
				
				updateForce(curP, typeP, jello, SHEARDIAGONAL);
				memset( (void*)&typeP, 0, sizeof(typeP));
				//-----------------------------------------------------------------
				// TYPE 25 -> (x-1, y-1, z-1)
				typeP.x = x - 1;
				typeP.y = y - 1;
				typeP.z = z - 1;
				
				updateForce(curP, typeP, jello, SHEARDIAGONAL);
				memset( (void*)&typeP, 0, sizeof(typeP));
				//-----------------------------------------------------------------
				// TYPE 26 -> (x, y-1, z)
				typeP.x = x;
				typeP.y = y - 1;
				typeP.z = z;
				
				updateForce(curP, typeP, jello, STRUCTURAL);
				memset( (void*)&typeP, 0, sizeof(typeP));
				//-----------------------------------------------------------------

				// BEND SPRINGS

				//-----------------------------------------------------------------
				// TYPE 27 -> (x+2, y, z)
				typeP.x = x + 2;
				typeP.y = y;
				typeP.z = z;
				
				updateForce(curP, typeP, jello, BEND);
				memset( (void*)&typeP, 0, sizeof(typeP));
				//-----------------------------------------------------------------
				// TYPE 28 -> (x-2, y, z)
				typeP.x = x - 2;
				typeP.y = y;
				typeP.z = z;
				
				updateForce(curP, typeP, jello, BEND);
				memset( (void*)&typeP, 0, sizeof(typeP));
				//-----------------------------------------------------------------
				// TYPE 29 -> (x, y+2, z)
				typeP.x = x;
				typeP.y = y + 2;
				typeP.z = z;
				
				updateForce(curP, typeP, jello, BEND);
				memset( (void*)&typeP, 0, sizeof(typeP));
				//-----------------------------------------------------------------
				// TYPE 30 -> (x, y-2, z)
				typeP.x = x;
				typeP.y = y - 2;
				typeP.z = z;
				
				updateForce(curP, typeP, jello, BEND);
				memset( (void*)&typeP, 0, sizeof(typeP));
				//-----------------------------------------------------------------
				// TYPE 31 -> (x, y, z+2)
				typeP.x = x;
				typeP.y = y;
				typeP.z = z + 2;
				
				updateForce(curP, typeP, jello, BEND);
				memset( (void*)&typeP, 0, sizeof(typeP));
				//-----------------------------------------------------------------
				// TYPE 32 -> (x, y, z-2)
				typeP.x = x;
				typeP.y = y;
				typeP.z = z - 2;
				
				updateForce(curP, typeP, jello, BEND);
				memset( (void*)&typeP, 0, sizeof(typeP));
				//-----------------------------------------------------------------
				
				// Check if the point obtained in this type is valid -> inside cube check
				// blah blah
				// check if the point obtained in this type is already parsed -> check for the x,y,z loop comparison
				// blah blah
				// if not parsed then find the hooks force and damping force.
				// add the force value to both the end points, i.e; current looping point and the type obtained point
				
			}	// for indexX
		}	// for indexY
	}	// for indexZ
	
	memset( (void*)a, 0, sizeof(a));
	for(z = 0; z < 8; z++)
	{
		for(y = 0; y < 8; y++)
		{
			for(x = 0; x < 8; x++)
			{
				a[x][y][z].x = force[x][y][z].x / jello->mass ;
				a[x][y][z].y = force[x][y][z].y / jello->mass ;
				a[x][y][z].z = force[x][y][z].z / jello->mass ;
			}
		}
	}


}



/* Performs one step of Euler Integration 
   as a result, updates the jello structure */
void Euler(struct world * jello)
{
  int i,j,k;
  point a[8][8][8];
  
  memset( (void*)force, 0, sizeof(force));
  memset( (void*)a, 0, sizeof(a));

  computeAcceleration(jello, a);
  
  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
			jello->p[i][j][k].x += jello->dt * jello->v[i][j][k].x;
			jello->p[i][j][k].y += jello->dt * jello->v[i][j][k].y;
			jello->p[i][j][k].z += jello->dt * jello->v[i][j][k].z;
	        jello->v[i][j][k].x += jello->dt * a[i][j][k].x;
		    jello->v[i][j][k].y += jello->dt * a[i][j][k].y;
			jello->v[i][j][k].z += jello->dt * a[i][j][k].z;
      }
}


/* performs one step of RK4 Integration */
/* as a result, updates the jello structure */
void RK4(struct world * jello)
{
  point F1p[8][8][8], F1v[8][8][8], 
        F2p[8][8][8], F2v[8][8][8],
        F3p[8][8][8], F3v[8][8][8],
        F4p[8][8][8], F4v[8][8][8];

  point a[8][8][8];


  struct world buffer;

  int i,j,k;

  buffer = *jello; // make a copy of jello

  computeAcceleration(jello, a);

  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
         pMULTIPLY(jello->v[i][j][k],jello->dt,F1p[i][j][k]);
         pMULTIPLY(a[i][j][k],jello->dt,F1v[i][j][k]);
         pMULTIPLY(F1p[i][j][k],0.5,buffer.p[i][j][k]);
         pMULTIPLY(F1v[i][j][k],0.5,buffer.v[i][j][k]);
         pSUM(jello->p[i][j][k],buffer.p[i][j][k],buffer.p[i][j][k]);
         pSUM(jello->v[i][j][k],buffer.v[i][j][k],buffer.v[i][j][k]);
      }

  computeAcceleration(&buffer, a);

  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
         // F2p = dt * buffer.v;
         pMULTIPLY(buffer.v[i][j][k],jello->dt,F2p[i][j][k]);
         // F2v = dt * a(buffer.p,buffer.v);     
         pMULTIPLY(a[i][j][k],jello->dt,F2v[i][j][k]);
         pMULTIPLY(F2p[i][j][k],0.5,buffer.p[i][j][k]);
         pMULTIPLY(F2v[i][j][k],0.5,buffer.v[i][j][k]);
         pSUM(jello->p[i][j][k],buffer.p[i][j][k],buffer.p[i][j][k]);
         pSUM(jello->v[i][j][k],buffer.v[i][j][k],buffer.v[i][j][k]);
      }

  computeAcceleration(&buffer, a);

  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
         // F3p = dt * buffer.v;
         pMULTIPLY(buffer.v[i][j][k],jello->dt,F3p[i][j][k]);
         // F3v = dt * a(buffer.p,buffer.v);     
         pMULTIPLY(a[i][j][k],jello->dt,F3v[i][j][k]);
         pMULTIPLY(F3p[i][j][k],0.5,buffer.p[i][j][k]);
         pMULTIPLY(F3v[i][j][k],0.5,buffer.v[i][j][k]);
         pSUM(jello->p[i][j][k],buffer.p[i][j][k],buffer.p[i][j][k]);
         pSUM(jello->v[i][j][k],buffer.v[i][j][k],buffer.v[i][j][k]);
      }
         
  computeAcceleration(&buffer, a);


  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
         // F3p = dt * buffer.v;
         pMULTIPLY(buffer.v[i][j][k],jello->dt,F4p[i][j][k]);
         // F3v = dt * a(buffer.p,buffer.v);     
         pMULTIPLY(a[i][j][k],jello->dt,F4v[i][j][k]);

         pMULTIPLY(F2p[i][j][k],2,buffer.p[i][j][k]);
         pMULTIPLY(F3p[i][j][k],2,buffer.v[i][j][k]);
         pSUM(buffer.p[i][j][k],buffer.v[i][j][k],buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],F1p[i][j][k],buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],F4p[i][j][k],buffer.p[i][j][k]);
         pMULTIPLY(buffer.p[i][j][k],1.0 / 6,buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],jello->p[i][j][k],jello->p[i][j][k]);

         pMULTIPLY(F2v[i][j][k],2,buffer.p[i][j][k]);
         pMULTIPLY(F3v[i][j][k],2,buffer.v[i][j][k]);
         pSUM(buffer.p[i][j][k],buffer.v[i][j][k],buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],F1v[i][j][k],buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],F4v[i][j][k],buffer.p[i][j][k]);
         pMULTIPLY(buffer.p[i][j][k],1.0 / 6,buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],jello->v[i][j][k],jello->v[i][j][k]);
      }

  return;  
}

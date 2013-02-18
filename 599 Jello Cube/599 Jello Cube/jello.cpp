/*

  CS 599, Physically Based Modeling for Interactive Simulation and Games
  USC/Viterbi/Computer Science
  Assignment 1 starter code

  Your name:
  <write your name here>

*/

#include "jello.h"
#include "showCube.h"
#include "input.h"
#include "physics.h"
#include <assert.h>
#include <fstream>

// MAIN MENU ITEM IDS
#define M1I1	1
#define M1I2	2
#define M1I3	3
#define M1I4	4

// WORLD FILES MENU ITEMS IDS
#define M2I1	5
#define M2I2	6
#define M2I3	7
#define M2I4	8
#define M2I5	9
#define M2I6    10

// LIGHTING MENU ITEMS IDS
#define M3I1	11
#define M3I2	12
#define M3I3	13
#define M3I4	14

// FORCE FIELD MENU ITEMS IDS
#define M4I1	15
#define M4I2	16

// RENDERING MENU ITEMS IDS
#define M5I1	17
#define M5I2	18

double SRLength;					//	Structural rest length
double SHSideRLength;				//	Shear face side rest length
double SHDiagonalRLength;			//	Shear diagonal rest length
double BRLength;					//	Bend rest length

// camera parameters
double Theta = pi / 6;
double Phi = pi / 6;
double R = 4;

// Sphere position in the bounding box
double SPHEREx = -1.2;
double SPHEREy = 0;
double SPHEREz = -1.2;
double SPHEREr = 0.8;
double restFactor = 1;

// mouse control
int g_iMenuId;
int g_vMousePos[2];
int g_iLeftMouseButton,g_iMiddleMouseButton,g_iRightMouseButton;

// number of images saved to disk so far
int sprite=0, mycount = 0;

// these variables control what is displayed on screen
int shear=1, bend=1, structural=1, pause=0, viewingMode=0, saveScreenToFile=0, ActivateFF = 0, sphereExists = 1;

struct world jello;

int windowWidth, windowHeight;

bool _highshineLevel = false; //Whether the shininess parameter is high
bool _lowspecLevel = false; //Whether the specularity parameter is high
bool _emissLevel = false; //Whether the emission parameter is turned on

int file = 0, render = 0, FF = 0, light = 0;

int vishakMainMenu = 0, WorldFilesMenu = 0, LightingMenu = 0, ForceFieldMenu = 0, RenderMenu = 0;

Snap::Snap(char* pA, int width, int height) : pixelArray(pA), w(width), h(height) 
{}

Snap::~Snap() {
	delete[] pixelArray;
}

GLuint ImageToGLTexture(Snap* snap) 
{
	
	GLuint textureResult;	// stores the resulting texture Id

	glGenTextures(1, &textureResult); 

	glBindTexture(GL_TEXTURE_2D, textureResult);	// Bind the texture Id, i.e; indicate openGL about this
	
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, snap->w, snap->h, 0, GL_RGB, GL_UNSIGNED_BYTE, snap->pixelArray);               

	return textureResult;					   
}

GLuint front_face_id, back_face_id, right_face_id;		// The ids of the 6 textures to be 
GLuint left_face_id, top_face_id, bottom_face_id;		// mapped onto the bounding box walls
GLuint front_face_ff_id, back_face_ff_id, right_face_ff_id, left_face_ff_id;
														
void myinit()
{

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(90.0,1.0,0.01,1000.0);

  // set background color
  glClearColor(0.6, 0.8, 0.196078, 1.0);

  glCullFace(GL_BACK);
  glEnable(GL_CULL_FACE);

  glShadeModel(GL_SMOOTH);
  glEnable(GL_POLYGON_SMOOTH);
  glEnable(GL_LINE_SMOOTH);

  Snap* snapBK = storeBitmap("back_face.bmp");
  back_face_id = ImageToGLTexture(snapBK);
  delete snapBK;

  Snap* snapF = storeBitmap("front_face.bmp");
  front_face_id = ImageToGLTexture(snapF);
  delete snapF;

  Snap* snapT = storeBitmap("top_face.bmp");
  top_face_id = ImageToGLTexture(snapT);
  delete snapT;

  Snap* snapB = storeBitmap("bottom_face.bmp");
  bottom_face_id = ImageToGLTexture(snapB);
  delete snapB;

  Snap* snapL = storeBitmap("left_face.bmp");
  left_face_id = ImageToGLTexture(snapL);
  delete snapL;

  Snap* snapR = storeBitmap("right_face.bmp");
  right_face_id = ImageToGLTexture(snapR);
  delete snapR;

  Snap* snapBKFF = storeBitmap("back_face_ff.bmp");
  back_face_ff_id = ImageToGLTexture(snapBKFF);
  delete snapBKFF;

  Snap* snapFFF = storeBitmap("front_face_ff.bmp");
  front_face_ff_id = ImageToGLTexture(snapFFF);
  delete snapFFF;

  Snap* snapLFF = storeBitmap("left_face_ff.bmp");
  left_face_ff_id = ImageToGLTexture(snapLFF);
  delete snapLFF;

  Snap* snapRFF = storeBitmap("right_face_ff.bmp");
  right_face_ff_id = ImageToGLTexture(snapRFF);
  delete snapRFF;

  glDisable(GL_COLOR_MATERIAL);

  return; 
}

void display()
{

	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHTING);
	
	glEnable(GL_LIGHT1);
	glEnable(GL_LIGHT2);
	glEnable(GL_LIGHT3);
	glEnable(GL_LIGHT4);
	glEnable(GL_LIGHT5);
	glEnable(GL_LIGHT6);
	glEnable(GL_LIGHT7);
	glEnable(GL_LIGHT0);

	glEnable(GL_NORMALIZE);
	glShadeModel(GL_SMOOTH);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_MODELVIEW); 
	glLoadIdentity();

	// camera parameters are Phi, Theta, R
	gluLookAt(R * cos(Phi) * cos (Theta), R * sin(Phi) * cos (Theta), R * sin (Theta),
			0, 0, 0, 0, 0.0,1.0); 


	GLfloat ambientLight[] = {0.2f, 0.2f, 0.2f, 1.0f};
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, ambientLight);

	float specLevel;
	if (_lowspecLevel) {
		specLevel = 0.3f;
	}
	else {
		specLevel = 1;
	}

	float emissLevel;
	if (_emissLevel) {
		emissLevel = 0.05f;
	}
	else {
		emissLevel = 0;
	}

	float shineLevel;
	if (_highshineLevel) {
		shineLevel = 25;
	}
	else {
		shineLevel = 12;
	}

//------------------------------
//		SCENE LIGHTING
//------------------------------
	// ---------------------
	//	SPHERE LIGHT 1
	// ---------------------
	GLfloat lightColor1[] = {0.6f, 0.6f, 0.6f, 1.0f};
	GLfloat lightPos1[] = {1.5f * 0.9, 2 * 0.9, 1.5 * 0.9, 1.0f};

	glLightfv(GL_LIGHT1, GL_DIFFUSE, lightColor1);
	glLightfv(GL_LIGHT1, GL_SPECULAR, lightColor1);
	glLightfv(GL_LIGHT1, GL_POSITION, lightPos1);

	// ---------------------
	//	CUBE LIGHT 2
	// ---------------------
	GLfloat lightColor2[] = {0.6f, 0.6f, 0.6f, 1.0f};
	GLfloat lightPos2[] = { -1.9, -1.9, -1.9, 1.0};

	glLightfv(GL_LIGHT2, GL_DIFFUSE, lightColor2);
	glLightfv(GL_LIGHT2, GL_SPECULAR, lightColor2);
	glLightfv(GL_LIGHT2, GL_POSITION, lightPos2);
    
	// ---------------------
	//	CUBE LIGHT 3
	// ---------------------
	GLfloat lightColor3[] = {0.6f, 0.6f, 0.6f, 1.0f};
	GLfloat lightPos3[] = { 1.9, -1.9, -1.9, 1.0};

	glLightfv(GL_LIGHT3, GL_DIFFUSE, lightColor3);
	glLightfv(GL_LIGHT3, GL_SPECULAR, lightColor3);
	glLightfv(GL_LIGHT3, GL_POSITION, lightPos3);

	// ---------------------
	//	CUBE LIGHT 4
	// ---------------------
	GLfloat lightColor4[] = {0.6f, 0.6f, 0.6f, 1.0f};
	GLfloat lightPos4[] = { 1.9, 1.9, -1.9, 1.0 };

	glLightfv(GL_LIGHT4, GL_DIFFUSE, lightColor3);
	glLightfv(GL_LIGHT4, GL_SPECULAR, lightColor3);
	glLightfv(GL_LIGHT4, GL_POSITION, lightPos3);

	// ---------------------
	//	CUBE LIGHT 5
	// ---------------------
	GLfloat lightColor5[] = {0.6f, 0.6f, 0.6f, 1.0f};
	GLfloat lightPos5[] = {-1.9, 1.9, -1.9, 1.0 };

	glLightfv(GL_LIGHT3, GL_DIFFUSE, lightColor3);
	glLightfv(GL_LIGHT3, GL_SPECULAR, lightColor3);
	glLightfv(GL_LIGHT3, GL_POSITION, lightPos3);

	// ---------------------
	//	SPHERE LIGHT 6
	// ---------------------
	GLfloat lightColor6[] = {0.6f, 0.6f, 0.6f, 1.0f};
	GLfloat lightPos6[] = {-1.9, -1.9, 1.9, 1.0};

	glLightfv(GL_LIGHT6, GL_DIFFUSE, lightColor6);
	glLightfv(GL_LIGHT6, GL_SPECULAR, lightColor6);
	glLightfv(GL_LIGHT6, GL_POSITION, lightPos6);

	// ---------------------
	//	SPHERE LIGHT 7
	// ---------------------
	GLfloat lightColor7[] = {0.6f, 0.6f, 0.6f, 1.0f};
	GLfloat lightPos7[] = {1.9, 1.9, 1.9, 1.0 };

	glLightfv(GL_LIGHT7, GL_DIFFUSE, lightColor7);
	glLightfv(GL_LIGHT7, GL_SPECULAR, lightColor7);
	glLightfv(GL_LIGHT7, GL_POSITION, lightPos7);

	// ---------------------
	//	SPHERE LIGHT 8
	// ---------------------
	GLfloat lightColor8[] = {0.6f, 0.6f, 0.6f, 1.0f};
	GLfloat lightPos8[] = {-1.9, 1.9, 1.9, 1.0};

	glLightfv(GL_LIGHT0, GL_DIFFUSE, lightColor8);
	glLightfv(GL_LIGHT0, GL_SPECULAR, lightColor8);
	glLightfv(GL_LIGHT0, GL_POSITION, lightPos8);

//------------------------------
//	SPHERE RENDERING
//------------------------------
	
	//The color of the SPHERE Object in the bounding box
	GLfloat materialColor1[] = {1, 1, 0, 1.0f};
	GLfloat materialSpecular1[] = {specLevel, specLevel, specLevel, 1.0f};
	GLfloat materialEmissLevel1[] = {emissLevel, emissLevel, emissLevel, 1.0f};

	if(sphereExists)
	{
		glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, materialColor1);
		glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, materialSpecular1);
		glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, materialEmissLevel1);
		glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, shineLevel); //The shininess parameter

		glDisable(GL_CULL_FACE);
		glPushMatrix();
		glTranslatef(SPHEREx, SPHEREy, SPHEREz);
		glutSolidSphere(SPHEREr, 150, 100);
		glPopMatrix();
	}

// ----------------------------
//	CUBE RENDERING
// ---------------------------
	//The color of the SPHERE Object in the bounding box
	GLfloat materialColor2[] = {1, 0, 0, 1};

	//The specular (shiny) component of the materialf
	GLfloat materialSpecular2[] = {specLevel, specLevel, specLevel, 1.0f};

	//The color emitted by the material
	GLfloat materialEmissLevel2[] = {emissLevel, emissLevel, emissLevel, 1.0f};

	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, materialColor2);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, materialSpecular2);
	glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, materialEmissLevel2);
	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, shineLevel); //The shininess parameter

	// show the cube once all the lighting is setup for the scene
	showCube(&jello, left_face_id);

	glDisable(GL_LIGHTING);

	glEnable(GL_CULL_FACE);
	glCullFace(GL_FRONT);
	//  glDisable(GL_CULL_FACE);

	// show the bounding box
	showBoundingBox(front_face_id, back_face_id, left_face_id, right_face_id, top_face_id, bottom_face_id,
	  front_face_ff_id, back_face_ff_id, right_face_ff_id, left_face_ff_id);

	glutSwapBuffers();
}


void doIdle()
{
  char s[20]="picxxxx.ppm";
  
  // save screen to file
  s[3] = 48 + (sprite / 1000);
  s[4] = 48 + (sprite % 1000) / 100;
  s[5] = 48 + (sprite % 100 ) / 10;
  s[6] = 48 + sprite % 10;

  mycount++;

  if (saveScreenToFile==1 && mycount%4 == 0)
  {
    saveScreenshot(windowWidth, windowHeight, s);
//    saveScreenToFile=0; // save only once, change this if you want continuos image generation (i.e. animation)
    sprite++;
  }

  if (sprite >= 300) // allow only 300 snapshots
  {
    exit(0);	
  }

  if (pause == 0)
  {
    // insert code which appropriately performs one step of the cube simulation:
//	 Euler(&jello);
	 RK4(&jello);
//	 pause = 1;
  }

  glutPostRedisplay();
}

// Convert a char array of two chars into short value
short charArrayToShort(const char* chars) {
	
	return (short)(
					((unsigned char)chars[1] << 8) |
					(unsigned char)chars[0]
				   );

}
short readShort(std::ifstream &in) {

	char store[2];
	in.read(store, 2);
	return charArrayToShort(store);


}


// Convert a char array of four chars ainto an integer value
int charArrayToInt(const char* chars) {
	
	return (int)(
				 ((unsigned char)chars[3] << 24) | ((unsigned char)chars[2] << 16) |
				 ((unsigned char)chars[1] << 8) | (unsigned char)chars[0]
				 );

}

int readInteger(std::ifstream &in) {

	char store[4];

	in.read(store, 4);
	return charArrayToInt(store);

}


Snap* storeBitmap(const char* fname) 
{
	// Open the file to read the bitmap snap content
	std::ifstream in;
	in.open(fname, std::ifstream::binary);
	assert(!in.fail() || !"Could not find file");

	char store[2];
	in.read(store, 2);

	// Check for the first two characters of the snap file, if it has 
	// "B" and "M" then its a bmp file
	assert(store[0] == 'B' && store[1] == 'M' || !"Not a bitmap file");
	in.ignore(8);
	int info = readInteger(in);
	
	// Fetch the header content
	int sizeH = readInteger(in);
	int w, h;
	
	w = readInteger(in);
	h = readInteger(in);
	in.ignore(2);
	assert(readShort(in) == 24 || !"Image is not 24 bits per pixel");
	assert(readShort(in) == 0 || !"Image is compressed");
	
	int BPerRCount = ((w * 3 + 3) / 4) * 4 - (w * 3 % 4);
	int sizeA = BPerRCount * h;
	
	vishakArray<char> pixelArrayObj(new char[sizeA]);
	
	in.seekg(info, std::ios_base::beg);
	in.read(pixelArrayObj.get(), sizeA);
	
	//Get the data into the right format
	vishakArray<char> pixelArrayObj2(new char[w * h * 3]);

	for(int y = 0; y < h; y++) {
		for(int x = 0; x < w; x++) {
			for(int c = 0; c < 3; c++) {
				pixelArrayObj2[3 * (w * y + x) + c] =
					pixelArrayObj[BPerRCount * y + 3 * x + (2 - c)];
			}
		}
	}
	
	in.close();
	return new Snap(pixelArrayObj2.release(), w, h);
}

void vishakMainMenuCallBack (int id) {

  glutPostRedisplay();
}


void WorldFilesMenuCallback (int id) {

  switch (id) {

  case M2I1:
		glutChangeToMenuEntry(1,"(moveleft.w)",M2I1);
		glutChangeToMenuEntry(2,"rotate.w",M2I2);
		glutChangeToMenuEntry(3,"jello.w", M2I3);
		glutChangeToMenuEntry(4,"rotate.w",M2I4);
		glutChangeToMenuEntry(5,"vishak.w",M2I5);
		glutChangeToMenuEntry(6,"skewedCorner.w",M2I6);
		file = 1;

		readWorld("moveleft.w",&jello);
  
		// Compute the rest length of all the springs by using the center most node
		SRLength = sqrt(((jello.p[4][4][4].x - jello.p[5][4][4].x)*(jello.p[4][4][4].x - jello.p[5][4][4].x)) + ((jello.p[4][4][4].y - jello.p[5][4][4].y)*(jello.p[4][4][4].y - jello.p[5][4][4].y)) + ((jello.p[4][4][4].z - jello.p[5][4][4].z)*(jello.p[4][4][4].z - jello.p[5][4][4].z)));  
		SHSideRLength = sqrt((double)2) * SRLength;
		SHDiagonalRLength = sqrt((double)3) * SRLength;
		BRLength = 2 * SRLength;

    break;

	case M2I2:
		glutChangeToMenuEntry(1,"moveleft.w",M2I1);
		glutChangeToMenuEntry(2,"(rotate.w)",M2I2);
		glutChangeToMenuEntry(3,"jello.w", M2I3);
		glutChangeToMenuEntry(4,"rotate.w",M2I4);
		glutChangeToMenuEntry(5,"vishak.w",M2I5);
		glutChangeToMenuEntry(6,"skewedCorner.w",M2I6);
		file = 2;

		readWorld("rotate.w",&jello);
  
		// Compute the rest length of all the springs by using the center most node
		SRLength = sqrt(((jello.p[4][4][4].x - jello.p[5][4][4].x)*(jello.p[4][4][4].x - jello.p[5][4][4].x)) + ((jello.p[4][4][4].y - jello.p[5][4][4].y)*(jello.p[4][4][4].y - jello.p[5][4][4].y)) + ((jello.p[4][4][4].z - jello.p[5][4][4].z)*(jello.p[4][4][4].z - jello.p[5][4][4].z)));  
		SHSideRLength = sqrt((double)2) * SRLength;
		SHDiagonalRLength = sqrt((double)3) * SRLength;
		BRLength = 2 * SRLength;

    break;

	case M2I3:
		glutChangeToMenuEntry(1,"moveleft.w",M2I1);
		glutChangeToMenuEntry(2,"rotate.w",M2I2);
		glutChangeToMenuEntry(3,"(jello.w)", M2I3);
		glutChangeToMenuEntry(4,"gravity.w",M2I4);
		glutChangeToMenuEntry(5,"vishak.w",M2I5);
		glutChangeToMenuEntry(6,"skewedCorner.w",M2I6);
		file = 3;

		readWorld("jello.w",&jello);
  
		// Compute the rest length of all the springs by using the center most node
		SRLength = sqrt(((jello.p[4][4][4].x - jello.p[5][4][4].x)*(jello.p[4][4][4].x - jello.p[5][4][4].x)) + ((jello.p[4][4][4].y - jello.p[5][4][4].y)*(jello.p[4][4][4].y - jello.p[5][4][4].y)) + ((jello.p[4][4][4].z - jello.p[5][4][4].z)*(jello.p[4][4][4].z - jello.p[5][4][4].z)));  
		SHSideRLength = sqrt((double)2) * SRLength;
		SHDiagonalRLength = sqrt((double)3) * SRLength;
		BRLength = 2 * SRLength;

    break;

	case M2I4:
		glutChangeToMenuEntry(1,"moveleft.w",M2I1);
		glutChangeToMenuEntry(2,"rotate.w",M2I2);
		glutChangeToMenuEntry(3,"jello.w", M2I3);
		glutChangeToMenuEntry(4,"(gravity.w)",M2I4);
		glutChangeToMenuEntry(5,"vishak.w",M2I5);
		glutChangeToMenuEntry(6,"skewedCorner.w",M2I6);
		file = 4;

		readWorld("gravity.w",&jello);
  
		// Compute the rest length of all the springs by using the center most node
		SRLength = sqrt(((jello.p[4][4][4].x - jello.p[5][4][4].x)*(jello.p[4][4][4].x - jello.p[5][4][4].x)) + ((jello.p[4][4][4].y - jello.p[5][4][4].y)*(jello.p[4][4][4].y - jello.p[5][4][4].y)) + ((jello.p[4][4][4].z - jello.p[5][4][4].z)*(jello.p[4][4][4].z - jello.p[5][4][4].z)));  
		SHSideRLength = sqrt((double)2) * SRLength;
		SHDiagonalRLength = sqrt((double)3) * SRLength;
		BRLength = 2 * SRLength;
    break;

	case M2I5:
		glutChangeToMenuEntry(1,"moveleft.w",M2I1);
		glutChangeToMenuEntry(2,"rotate.w",M2I2);
		glutChangeToMenuEntry(3,"jello.w", M2I3);
		glutChangeToMenuEntry(4,"gravity.w",M2I4);
		glutChangeToMenuEntry(5,"(vishak.w)",M2I5);
		glutChangeToMenuEntry(6,"skewedCorner.w",M2I6);
		file = 5;

		readWorld("vishak.w",&jello);
  
		// Compute the rest length of all the springs by using the center most node
		SRLength = sqrt(((jello.p[4][4][4].x - jello.p[5][4][4].x)*(jello.p[4][4][4].x - jello.p[5][4][4].x)) + ((jello.p[4][4][4].y - jello.p[5][4][4].y)*(jello.p[4][4][4].y - jello.p[5][4][4].y)) + ((jello.p[4][4][4].z - jello.p[5][4][4].z)*(jello.p[4][4][4].z - jello.p[5][4][4].z)));  
		SHSideRLength = sqrt((double)2) * SRLength;
		SHDiagonalRLength = sqrt((double)3) * SRLength;
		BRLength = 2 * SRLength;
    break;

	case M2I6:
		glutChangeToMenuEntry(1,"moveleft.w",M2I1);
		glutChangeToMenuEntry(2,"rotate.w",M2I2);
		glutChangeToMenuEntry(3,"jello.w", M2I3);
		glutChangeToMenuEntry(4,"gravity.w",M2I4);
		glutChangeToMenuEntry(5,"vishak.w",M2I5);
		glutChangeToMenuEntry(6,"(skewedCorner.w)",M2I6);
		file = 5;

		readWorld("skewedCorner.w",&jello);
  
		// Compute the rest length of all the springs by using the center most node
		SRLength = sqrt(((jello.p[4][4][4].x - jello.p[5][4][4].x)*(jello.p[4][4][4].x - jello.p[5][4][4].x)) + ((jello.p[4][4][4].y - jello.p[5][4][4].y)*(jello.p[4][4][4].y - jello.p[5][4][4].y)) + ((jello.p[4][4][4].z - jello.p[5][4][4].z)*(jello.p[4][4][4].z - jello.p[5][4][4].z)));  
		SHSideRLength = sqrt((double)2) * SRLength;
		SHDiagonalRLength = sqrt((double)3) * SRLength;
		BRLength = 2 * SRLength;
    break;

	default:
		break;
  }
 glutPostRedisplay();
}
 

void LightingMenuCallback (int id)
{ 
	 switch (id) 
	 {
		case M3I1:
			glutChangeToMenuEntry(1,"(Shininess)", M3I1);
			glutChangeToMenuEntry(2,"Specularity", M3I2);
			glutChangeToMenuEntry(3,"Emissivity", M3I3);
			_highshineLevel = !_highshineLevel;
		break;

		case M3I2:
			glutChangeToMenuEntry(1,"Shininess", M3I1);
			glutChangeToMenuEntry(2,"(Specularity)", M3I2);
			glutChangeToMenuEntry(3,"Emissivity", M3I3);
			_lowspecLevel = !_lowspecLevel;
		break;

		case M3I3:
			glutChangeToMenuEntry(1,"Shininess", M3I1);
			glutChangeToMenuEntry(2,"Specularity", M3I2);
			glutChangeToMenuEntry(3,"(Emissivity)", M3I3);
			_emissLevel = !_emissLevel;
		break;

		default:
			break;
	 }
  glutPostRedisplay();
}


void ForceFieldMenuCallback (int id) 
{ 
	switch(id)
	{
		case M4I1:
			glutChangeToMenuEntry(1,"(SWITCH ON)", M4I1);
			glutChangeToMenuEntry(2,"SWITCH OFF", M4I2);
			ActivateFF = 1;
			break;

		case M4I2:
			glutChangeToMenuEntry(1,"SWITCH ON", M4I1);
			glutChangeToMenuEntry(2,"(SWITCH OFF)", M4I2);
			ActivateFF = 0;
			break;

		default:
			break;
	}
  glutPostRedisplay();
}


void RenderMenuCallback (int id) 
{
	switch(id)
	{
	 case M5I1:
		glutChangeToMenuEntry(1,"(Wireframe)", M5I1);
		glutChangeToMenuEntry(2,"Triangle Mode", M5I2);
		viewingMode = 0;
		break;

	case M5I2:
		glutChangeToMenuEntry(1,"Wireframe", M5I1);
		glutChangeToMenuEntry(2,"(Triangle Mode)", M5I2);
		viewingMode = 1;
		break;

	default:
		break;
	}
  glutPostRedisplay();
}


int main (int argc, char ** argv)
{
  if (argc<2)
  {  
    printf ("Oops! You didn't say the jello world file!\n");
    printf ("Usage: %s [worldfile]\n", argv[0]);
    exit(0);
  }

  writeWorld("vishak.w", &jello);

  // Reads jello structure from world file and saves them into jello structure reference passed to it
  // file -> structure
  readWorld(argv[1],&jello);
  
  // Compute the rest length of all the springs by using the center most node
  SRLength = sqrt(((jello.p[4][4][4].x - jello.p[5][4][4].x)*(jello.p[4][4][4].x - jello.p[5][4][4].x)) + ((jello.p[4][4][4].y - jello.p[5][4][4].y)*(jello.p[4][4][4].y - jello.p[5][4][4].y)) + ((jello.p[4][4][4].z - jello.p[5][4][4].z)*(jello.p[4][4][4].z - jello.p[5][4][4].z)));  
  SHSideRLength = sqrt((double)2) * SRLength;
  SHDiagonalRLength = sqrt((double)3) * SRLength;
  BRLength = 2 * SRLength;

  glutInit(&argc,argv);
  
  /* double buffered window, use depth testing, 640x480 */
  glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
  
  windowWidth = 800;
  windowHeight = 600;
  glutInitWindowSize (windowWidth, windowHeight);
  glutInitWindowPosition (0,0);
  glutCreateWindow ("Jello cube Vishak Nag Ashoka");

  /* tells glut to use a particular display function to redraw */
  glutDisplayFunc(display);

  /* replace with any animate code */
  glutIdleFunc(doIdle);

  /* callback for mouse drags */
  glutMotionFunc(mouseMotionDrag);

  /* callback for mouse movement */
  glutPassiveMotionFunc(mouseMotion);

  /* callback for mouse button changes */
  glutMouseFunc(mouseButton);

  /* register for keyboard events */
  glutKeyboardFunc(keyboardFunc);

  /* do initialization */
  myinit();

  WorldFilesMenu=glutCreateMenu(WorldFilesMenuCallback);
  glutAddMenuEntry("moveleft.w", M2I1);
  glutAddMenuEntry("rotate.w", M2I2);
  glutAddMenuEntry("jello.w", M2I3);
  glutAddMenuEntry("gravity.w", M2I4);
  glutAddMenuEntry("vishak.w", M2I5);
  glutAddMenuEntry("skewedCorner.w",M2I6);

  LightingMenu=glutCreateMenu(LightingMenuCallback);
  glutAddMenuEntry("Shininess", M3I1);
  glutAddMenuEntry("Specularity", M3I2);
  glutAddMenuEntry("Emissivity", M3I3);

  ForceFieldMenu=glutCreateMenu(ForceFieldMenuCallback);
  glutAddMenuEntry("SWITCH ON", M4I1);
  glutAddMenuEntry("SWITCH OFF", M4I2);
  
  RenderMenu=glutCreateMenu(RenderMenuCallback);
  glutAddMenuEntry("Wireframe", M5I1);
  glutAddMenuEntry("Triangle Mode", M5I2);

  glutCreateMenu(vishakMainMenuCallBack);
  glutAddSubMenu("World Files >", WorldFilesMenu);
  glutAddSubMenu("Lighting >", LightingMenu);
  glutAddSubMenu("Force Field >", ForceFieldMenu);
  glutAddSubMenu("Render Type >", RenderMenu);

  glutAttachMenu(GLUT_LEFT_BUTTON);

  /* Forever sink in the black hole */
  glutMainLoop();

  return(0);
}


/*

  CS 599, Physically Based Modeling for Interactive Simulation and Games
  USC/Viterbi/Computer Science
  Assignment 1 starter code

*/

#include "jello.h"
#include "showCube.h"

//using namespace std;

int pointMap(int side, int i, int j)
{
  int r;

  switch (side)
  {
  case 1: //[i][j][0] bottom face
    r = 64 * i + 8 * j;
    break;
  case 6: //[i][j][7] top face
    r = 64 * i + 8 * j + 7;
    break;
  case 2: //[i][0][j] front face
    r = 64 * i + j;
    break;
  case 5: //[i][7][j] back face
    r = 64 * i + 56 + j;
    break;
  case 3: //[0][i][j] left face
    r = 8 * i + j;
    break;
  case 4: //[7][i][j] right face
    r = 448 + 8 * i + j;
    break;
  }

  return r;
}

void showCube(struct world * jello, GLuint left_face_id)
{
  int i,j,k,ip,jp,kp;
  point r1,r2,r3; // aux variables
  
  /* normals buffer and counter for Gourand shading*/
  struct point normal[8][8];
  int counter[8][8];

  int face;
  double faceFactor, length;
	
  if (fabs(jello->p[0][0][0].x) > 10)
  {
    printf ("Your cube somehow escaped way out of the box.\n");
    exit(0);
  }

  #define NODE(face,i,j) (*((struct point * )(jello->p) + pointMap((face),(i),(j))))

  // i, j , k vertex and the new position shifted by the di, dj, dk parameters will be rendered if the 
  // vertex i,j,k is inside the bounding box and if it is not the surface node

  #define PROCESS_NEIGHBOUR(di,dj,dk) \
    ip=i+(di);\
    jp=j+(dj);\
    kp=k+(dk);\
    if\
    (!( (ip>7) || (ip<0) ||\
      (jp>7) || (jp<0) ||\
    (kp>7) || (kp<0) ) && ((i==0) || (i==7) || (j==0) || (j==7) || (k==0) || (k==7))\
       && ((ip==0) || (ip==7) || (jp==0) || (jp==7) || (kp==0) || (kp==7))) \
    {\
      glVertex3f(jello->p[i][j][k].x,jello->p[i][j][k].y,jello->p[i][j][k].z);\
      glVertex3f(jello->p[ip][jp][kp].x,jello->p[ip][jp][kp].y,jello->p[ip][jp][kp].z);\
    }\

 
  if (viewingMode==0) // render wireframe
  {
    glLineWidth(1);
    glPointSize(5);
    glDisable(GL_LIGHTING);
    for (i=0; i<=7; i++)
      for (j=0; j<=7; j++)
        for (k=0; k<=7; k++)
        {
          if (i*j*k*(7-i)*(7-j)*(7-k) != 0) // not surface point
            continue;

          glBegin(GL_POINTS); // draw point
            glColor4f(0,0,0,0);  
            glVertex3f(jello->p[i][j][k].x,jello->p[i][j][k].y,jello->p[i][j][k].z);        
          glEnd();

          //
          //if ((i!=7) || (j!=7) || (k!=7))
          //  continue;

          glBegin(GL_LINES);      
          // structural
          if (structural == 1)
          {
            glColor4f(1,0,0,1);
            PROCESS_NEIGHBOUR(1,0,0)
            PROCESS_NEIGHBOUR(0,1,0)
            PROCESS_NEIGHBOUR(0,0,1)
            PROCESS_NEIGHBOUR(-1,0,0)
            PROCESS_NEIGHBOUR(0,-1,0)
            PROCESS_NEIGHBOUR(0,0,-1)
          }
          
          // shear
          if (shear == 1)
          {
            glColor4f(0.0f,1.0f,0.0f,1.0f);

			// if the point is at the center, 
			// layer parallel to screen - right top, left top, right bottom, left bottom
            PROCESS_NEIGHBOUR(1,1,0)
            PROCESS_NEIGHBOUR(-1,1,0)
            PROCESS_NEIGHBOUR(-1,-1,0)
            PROCESS_NEIGHBOUR(1,-1,0)

			// layer perpendicular to the screen and is vertical - front top, front bottom, back top, back bottom
            PROCESS_NEIGHBOUR(0,1,1)
            PROCESS_NEIGHBOUR(0,-1,1)
            PROCESS_NEIGHBOUR(0,-1,-1)
            PROCESS_NEIGHBOUR(0,1,-1)

			// layer perpendicular to the screen and is horizontal - front right, front left, back right, back left
            PROCESS_NEIGHBOUR(1,0,1)
            PROCESS_NEIGHBOUR(-1,0,1)
            PROCESS_NEIGHBOUR(-1,0,-1)
            PROCESS_NEIGHBOUR(1,0,-1)

			// layer parallel to screen behind the point into the screen - front right top, front left top, front right bottom, front left bottom  
            PROCESS_NEIGHBOUR(1,1,1)
            PROCESS_NEIGHBOUR(-1,1,1)
            PROCESS_NEIGHBOUR(-1,-1,1)
            PROCESS_NEIGHBOUR(1,-1,1)

			// layer parallel to screen in front of the point out of the screen - back right top, back left top, back right bottom, back left bottom
            PROCESS_NEIGHBOUR(1,1,-1)
            PROCESS_NEIGHBOUR(-1,1,-1)
            PROCESS_NEIGHBOUR(-1,-1,-1)
            PROCESS_NEIGHBOUR(1,-1,-1)
          }
          
          // bend
          if (bend == 1)
          {
            glColor4f(0,0,1,1);

			// connecting the current point cube with the adjacent cubes
            PROCESS_NEIGHBOUR(2,0,0)
            PROCESS_NEIGHBOUR(0,2,0)
            PROCESS_NEIGHBOUR(0,0,2)
            PROCESS_NEIGHBOUR(-2,0,0)
            PROCESS_NEIGHBOUR(0,-2,0)
            PROCESS_NEIGHBOUR(0,0,-2)
          }           
          glEnd();
        }
    glEnable(GL_LIGHTING);
  }
  
  else
  {
    glPolygonMode(GL_FRONT, GL_FILL);
    
    for (face=1; face <= 6; face++) 
      // face == face of a cube
      // 1 = bottom, 2 = front, 3 = left, 4 = right, 5 = far, 6 = top
    {
      
      if ((face==1) || (face==3) || (face==5))
        faceFactor=-1; // flip orientation
      else
        faceFactor=1;
      
      for (i=0; i <= 7; i++) // reset buffers
        for (j=0; j <= 7; j++)
        {
          normal[i][j].x=0; normal[i][j].y=0; normal[i][j].z=0;
          counter[i][j]=0;
        }

      /* process triangles, accumulate normals for Gourad shading */
  
      for (i=0; i <= 6; i++)
        for (j=0; j <= 6; j++) // process block (i,j)
        {
          pDIFFERENCE(NODE(face,i+1,j),NODE(face,i,j),r1); // first triangle
          pDIFFERENCE(NODE(face,i,j+1),NODE(face,i,j),r2);
          CROSSPRODUCTp(r1,r2,r3); pMULTIPLY(r3,faceFactor,r3);
          pNORMALIZE(r3);
          pSUM(normal[i+1][j],r3,normal[i+1][j]);
          counter[i+1][j]++;
          pSUM(normal[i][j+1],r3,normal[i][j+1]);
          counter[i][j+1]++;
          pSUM(normal[i][j],r3,normal[i][j]);
          counter[i][j]++;

          pDIFFERENCE(NODE(face,i,j+1),NODE(face,i+1,j+1),r1); // second triangle
          pDIFFERENCE(NODE(face,i+1,j),NODE(face,i+1,j+1),r2);
          CROSSPRODUCTp(r1,r2,r3); pMULTIPLY(r3,faceFactor,r3);
          pNORMALIZE(r3);
          pSUM(normal[i+1][j],r3,normal[i+1][j]);
          counter[i+1][j]++;
          pSUM(normal[i][j+1],r3,normal[i][j+1]);
          counter[i][j+1]++;
          pSUM(normal[i+1][j+1],r3,normal[i+1][j+1]);
          counter[i+1][j+1]++;
        }

      
        /* the actual rendering */
        for (j=1; j<=7; j++) 
        {

          if (faceFactor  > 0)
            glFrontFace(GL_CCW); // the usual definition of front face
          else
            glFrontFace(GL_CW); // flip definition of orientation
         
		  glBindTexture(GL_TEXTURE_2D, left_face_id);

			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

          glBegin(GL_TRIANGLE_STRIP);
          for (i=0; i<=7; i++)
          {
			
            glNormal3f(normal[i][j].x / counter[i][j],normal[i][j].y / counter[i][j],
              normal[i][j].z / counter[i][j]);
			glTexCoord2f(0.0f, 0.0f);
            glVertex3f(NODE(face,i,j).x, NODE(face,i,j).y, NODE(face,i,j).z);

            glNormal3f(normal[i][j-1].x / counter[i][j-1],normal[i][j-1].y/ counter[i][j-1],
              normal[i][j-1].z / counter[i][j-1]);
			glTexCoord2f(1.0f, 1.0f);
            glVertex3f(NODE(face,i,j-1).x, NODE(face,i,j-1).y, NODE(face,i,j-1).z);
          }
          glEnd();
        }
        
    }  
  } // end for loop over faces
  glFrontFace(GL_CCW);
}

void showBoundingBox(GLuint front_face_id, GLuint back_face_id, GLuint right_face_id, GLuint left_face_id, GLuint top_face_id, GLuint bottom_face_id,
					 GLuint front_face_ff_id, GLuint back_face_ff_id, GLuint right_face_ff_id, GLuint left_face_ff_id)
{
  
  glColor4f(1,1,1,1);

  glEnable(GL_TEXTURE_2D);

  // LEFT FACE
  if(ActivateFF)
  {
      glBindTexture(GL_TEXTURE_2D, right_face_ff_id);
  }
  else if(!ActivateFF)
  {
	  glBindTexture(GL_TEXTURE_2D, left_face_id);
  }

	  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	  
	  glBegin(GL_QUADS);
	  
	  glNormal3f(0.0, 1.0f, 0.0f);
	  glTexCoord2f(0.0f, 0.0f);
	  glVertex3f(-2, 2, -2);
	  glTexCoord2f(1.0f, 0.0f);
	  glVertex3f(-2, -2, -2);
	  glTexCoord2f(1.0f, 1.0f);
	  glVertex3f(-2, -2, 2);
	  glTexCoord2f(0.0f, 1.0f);
	  glVertex3f(-2, 2, 2);

	  glEnd();

  // RIGHT FACE
  if(ActivateFF)
  {
      glBindTexture(GL_TEXTURE_2D, left_face_ff_id);
  }
  else if(!ActivateFF)
  {
	  glBindTexture(GL_TEXTURE_2D, right_face_id);
  }

  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  
  glBegin(GL_QUADS);

  glNormal3f(0.0, 1.0f, 0.0f);
  glTexCoord2f(0.0f, 0.0f);
  glVertex3f(2, -2, -2);
  glTexCoord2f(1.0f, 0.0f);
  glVertex3f(2, 2, -2);
  glTexCoord2f(1.0f, 1.0f);
  glVertex3f(2, 2, 2);
  glTexCoord2f(0.0f, 1.0f);
  glVertex3f(2, -2, 2);

  glEnd();

  // BACK FACE
  if(ActivateFF)
  {
      glBindTexture(GL_TEXTURE_2D, back_face_ff_id);
  }
  else if(!ActivateFF)
  {
	  glBindTexture(GL_TEXTURE_2D, back_face_id);
  }

  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  
  glBegin(GL_QUADS);

  glNormal3f(0.0, 1.0f, 0.0f);
  glTexCoord2f(0.0f, 0.0f);
  glVertex3f(-2, -2, -2);
  glTexCoord2f(1.0f, 0.0f);
  glVertex3f(2, -2, -2);
  glTexCoord2f(1.0f, 1.0f);
  glVertex3f(2, -2, 2);
  glTexCoord2f(0.0f, 1.0f);
  glVertex3f(-2, -2, 2);
  
  glEnd();

  // FRONT FACE
  if(ActivateFF)
  {
      glBindTexture(GL_TEXTURE_2D, front_face_ff_id);
  }
  else if(!ActivateFF)
  {
	  glBindTexture(GL_TEXTURE_2D, front_face_id);
  }

  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  
  glBegin(GL_QUADS);

  glNormal3f(0.0, 1.0f, 0.0f);
  glTexCoord2f(0.0f, 0.0f);
  glVertex3f(2, 2, -2);
  glTexCoord2f(1.0f, 0.0f);
  glVertex3f(-2, 2, -2);
  glTexCoord2f(1.0f, 1.0f);
  glVertex3f(-2, 2, 2);
  glTexCoord2f(0.0f, 1.0f);
  glVertex3f(2, 2, 2);
  
  glEnd();


  // BOTTOM FACE
  glBindTexture(GL_TEXTURE_2D, bottom_face_id);

  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  
  glBegin(GL_QUADS);

  glNormal3f(0.0, 1.0f, 0.0f);
  glTexCoord2f(0.0f, 0.0f);
  glVertex3f(-2, 2, -2);
  glTexCoord2f(1.0f, 0.0f);
  glVertex3f(2, 2, -2);
  glTexCoord2f(1.0f, 1.0f);
  glVertex3f(2, -2, -2);
  glTexCoord2f(0.0f, 1.0f);
  glVertex3f(-2, -2, -2);
  
  glEnd();


  // TOP FACE
  glBindTexture(GL_TEXTURE_2D, top_face_id);

  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	
  glBegin(GL_QUADS);

  glNormal3f(0.0, -1.0f, 0.0f);
  glTexCoord2f(0.0f, 0.0f);
  glVertex3f(-2, -2, 2);
  glTexCoord2f(1.0f, 0.0f);
  glVertex3f(2, -2, 2);
  glTexCoord2f(1.0f, 1.0f);
  glVertex3f(2, 2, 2);
  glTexCoord2f(0.0f, 1.0f);
  glVertex3f(-2, 2, 2);

  glEnd();

  return;
}


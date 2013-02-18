/*

  CS 599, Physically Based Modeling for Interactive Simulation and Games
  USC/Viterbi/Computer Science
  Assignment 1 starter code

*/


#ifndef _SHOWCUBE_H_
#define _SHOWCUBE_H_

void showCube(struct world * jello, GLuint left_face_id);

void showBoundingBox(GLuint front_face_id, GLuint back_face_id, GLuint right_face_id, GLuint left_face_id, GLuint top_face_id, GLuint bottom_face_id,
					 GLuint front_face_ff_id, GLuint back_face_ff_id, GLuint right_face_ff_id, GLuint left_face_ff_id);

#endif

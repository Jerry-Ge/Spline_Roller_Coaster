/*
  CSCI 420 Assignment 2
  Name: Yuzhou Ge
  ID: 7057669325
 */

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <stdlib.h>
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h>
#include "pic.h"
#include <math.h>

/*manipulation variable*/
int g_iMenuId;
typedef enum { ROTATE, TRANSLATE, SCALE } CONTROLSTATE;
CONTROLSTATE g_ControlState = ROTATE;
int g_vMousePos[2] = {0, 0};
int g_iLeftMouseButton = 0;    /* 1 if pressed, 0 if not */
int g_iMiddleMouseButton = 0;
int g_iRightMouseButton = 0;

/* state of the world */
float g_vLandRotate[3] = {0.0, 0.0, 0.0};
float g_vLandTranslate[3] = {0.0, 0.0, 0.0};
float g_vLandScale[3] = {1.0, 1.0, 1.0};

/*count the total number of points of the whole spline*/
int numOfSplinePoints;


/*data for texture mapping*/
GLuint textures[] = {0, 1, 2, 3, 4, 5, 6};//up, down, front, back, right, left, 
const int UP = 0, DOWN = 1, FRONT = 2, BACK = 3, RIGHT = 4, LEFT = 5, WOOD = 6;

char* T_UP = "images/sky.jpg";
char* T_DOWN = "images/down.jpg";
char* T_FRONT = "images/sky.jpg";
char* T_BACK = "images/sky.jpg";
char* T_RIGHT = "images/sky.jpg";
char* T_LEFT = "images/sky.jpg";
char* T_Cross = "images/wood.jpg";
const float SCALE_MAPPING = 256.0;

//variables to store display list.
GLuint groundSkyID;
GLuint railID;
GLuint crossID;


/*the ride*/
double currentU = 0.0;
double MAX_HEIGHT = 0.0; // this is for calculating gravity speed.
int ride_counter = 0;

/*constant variables for camera movement*/
const double SPEED = 4.0;
const double TIMESTEP = SPEED / 1000; 
const double EYE_UP = 0.2 / 10 * 6;


/*constant variables for drawing the double rail and cross section*/
const double CROSS_RAIL_DISTANCE = 0.0;
const double RAIL_SIZE = CROSS_RAIL_DISTANCE / 15;
const int DISTANCE_BETWEEN_CROSSES = 1000 / 10;
const double CROSS_HEIGHT = 0.2 / 10;
const double CROSS_WIDTH = 0.2 / 3 * 5;
const int CROSS_LENGTH = DISTANCE_BETWEEN_CROSSES / 10 * 3;

/* represents one control point along the spline */
struct point {
   double x;
   double y;
   double z;
};

point V0; //initial vector to calculate first Normal Vector

/*spline points arrays*/
point* splinePoints;
point* leftSplinePoints;
point* rightSplinePoints;
point* tangentOfSpline;
point* normalOfSpline;
point* binormalOfSpline;


//boolean for control screen shots
bool isScreenShot = false;
int numScreenShots = 0;


//time variables
clock_t current_time;
/* Write a screenshot to the specified filename */
void saveScreenshot (char *filename)
{
  int i, j;
  Pic *in = NULL;

  if (filename == NULL)
    return;

  /* Allocate a picture buffer */
  in = pic_alloc(640, 480, 3, NULL);

  printf("File to save to: %s\n", filename);

  for (i=479; i>=0; i--) {
    glReadPixels(0, 479-i, 640, 1, GL_RGB, GL_UNSIGNED_BYTE,
                 &in->pix[i*in->nx*in->bpp]);
  }

  if (jpeg_write(filename, in))
    printf("File saved Successfully\n");
  else
    printf("Error in Saving\n");

  pic_free(in);
}



/* spline struct which contains how many control points, and an array of control points */
struct spline {
   int numControlPoints;
   struct point *points;
};

/* the spline array */
struct spline *g_Splines;

/* total number of splines */
int g_iNumOfSplines;


/*function for calculating Catmull-Rom Spline points*/
double splinePointCalc(double p0, double p1, double p2, double p3, double u) {
  double retval;
  retval = 0.5 * (
                  (-p0 + 3*p1 - 3*p2 + p3) * pow(u,3) + 
                  (2*p0 - 5*p1 + 4*p2 - p3) * pow(u,2) + 
                  (-p0 + p2)*u +
                  (2*p1)
                  );
  return retval;
}

/*function for calculating Catmull-Rom Spline tangent*/
double splineTangentCalc(double p0, double p1, double p2, double p3, double u) {
  double retval;
  retval = 0.5 * (
                  (-p0 + 3*p1 - 3*p2 + p3) * pow(u,2) * 3 + 
                  (2*p0 - 5*p1 + 4*p2 - p3) * u * 2 + 
                  (-p0 + p2)
                  );
  return retval;
}


/*vector normilization helper function*/
point normalize(point p) {
  double leng = sqrt(pow(p.x, 2) + pow(p.y, 2) + pow(p.z, 2));
  p.x = p.x/leng;
  p.y = p.y/leng;
  p.z = p.z/leng;
  return p;
}


/*vector addtion helper function*/
point addV(point a, point b) {
  point c;
  c.x = a.x + b.x;
  c.y = a.y + b.y;
  c.z = a.z + b.z;
  return c;
}

/*vector cross product helper function*/
//!!!!the order does matter!!!!!!!!
point crossProduct(point a, point b) {
  point c;
  c.x = a.y*b.z - a.z*b.y;
  c.y = - (a.x*b.z - a.z*b.x);
  c.z = a.x*b.y - b.x*a.y;
  return c;
}

/*with provided control points, draw interval points of Catmull-Rom spline*/
void splineGenerator(spline* g_Splines) {
  int numControlPoints = g_Splines[0].numControlPoints;
  float uIncrement = 0.001;

  //set up the initial vector for finding normal
  V0.x = 0.0; V0.y = 0.0; V0.z = -1.0;


  /*arrays that contain all spline info*/
  splinePoints = new point[numControlPoints * 1000];
  leftSplinePoints = new point[numControlPoints * 1000];
  rightSplinePoints = new point[numControlPoints * 1000];
  tangentOfSpline = new point[numControlPoints * 1000];
  normalOfSpline = new point[numControlPoints * 1000];
  binormalOfSpline = new point[numControlPoints * 1000];



  int arrayIndex = 0;
  /*loop from the first control point to the 4th to the last.
    each group of four points controls a spline segement
    for each segement, with increment 0.001, draw gl_line segenment
    to create the real line
  */
  for (int i = 0; i < numControlPoints - 3; i++) {
    for (float u = 0.0; u < 1.0; u += uIncrement) {
      point p0, p1, p2, p3;
      p0 = g_Splines[0].points[i];
      p1 = g_Splines[0].points[i + 1];
      p2 = g_Splines[0].points[i + 2];
      p3 = g_Splines[0].points[i + 3];

      //calculate the interval points with parameter u
      point intervalPoint;
      intervalPoint.x = splinePointCalc(p0.x, p1.x, p2.x, p3.x, u);
      intervalPoint.y = splinePointCalc(p0.y, p1.y, p2.y, p3.y, u);
      intervalPoint.z = splinePointCalc(p0.z, p1.z, p2.z, p3.z, u);
      splinePoints[arrayIndex] = intervalPoint;
      if (intervalPoint.z > MAX_HEIGHT) {
        MAX_HEIGHT = intervalPoint.z;
      }
      
      //tangents of spline
      point tangent;
      tangent.x = splineTangentCalc(p0.x, p1.x, p2.x, p3.x, u);
      tangent.y = splineTangentCalc(p0.y, p1.y, p2.y, p3.y, u);
      tangent.z = splineTangentCalc(p0.z, p1.z, p2.z, p3.z, u);
      tangentOfSpline[arrayIndex] = tangent;
      tangent = normalize(tangent);

      //calculate normal and binormal
      point normal, binormal;
      if (arrayIndex == 0) { //initial condition
        point T0 = tangent;
        normal = normalize(crossProduct(T0, V0));
        binormal = normalize(crossProduct(T0, normal));
      } else {
        point B0 = binormalOfSpline[arrayIndex - 1];
        point T1 = tangent;
        normal = normalize(crossProduct(B0, T1));
        binormal = normalize(crossProduct(T1, normal));
      }
      normalOfSpline[arrayIndex] = normal;
      binormalOfSpline[arrayIndex] = binormal;

      //generate double rails
      double halfDistance = 0.1;
      point leftSpline, rightSpline;

      //left rail
      leftSpline.x = intervalPoint.x - halfDistance * normal.x;
      leftSpline.y = intervalPoint.y - halfDistance * normal.y;
      leftSpline.z = intervalPoint.z - halfDistance * normal.z;
      leftSplinePoints[arrayIndex] = leftSpline;

      //right rail
      rightSpline.x = intervalPoint.x + halfDistance * normal.x;
      rightSpline.y = intervalPoint.y + halfDistance * normal.y;
      rightSpline.z = intervalPoint.z + halfDistance * normal.z;
      rightSplinePoints[arrayIndex] = rightSpline;

      //increment the total array index, calculate next interval point
      arrayIndex++;
    }
  }
  numOfSplinePoints = arrayIndex;
}


/*function for loading texture image files*/
void loadTexture(char* filename, GLuint textureID) {
  Pic* myTexture = jpeg_read(filename, NULL);
  if (myTexture == NULL) {
    printf("can not open file %sn \n", filename);
    exit(1);
  }

  glBindTexture(GL_TEXTURE_2D, textureID);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_CLAMP);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_CLAMP);
  gluBuild2DMipmaps(GL_TEXTURE_2D, 4, myTexture->nx, myTexture->ny, GL_RGB, GL_UNSIGNED_BYTE, myTexture->pix);

}


/*for generating textures for sky and ground*/
void textureGnerator() {
  glGenTextures(1, &textures[UP]);
  loadTexture(T_UP, textures[UP]);
  glGenTextures(1, &textures[DOWN]);
  loadTexture(T_DOWN, textures[DOWN]);
  glGenTextures(1, &textures[FRONT]);
  loadTexture(T_FRONT, textures[FRONT]);
  glGenTextures(1, &textures[BACK]);
  loadTexture(T_BACK, textures[BACK]);
  glGenTextures(1, &textures[RIGHT]);
  loadTexture(T_RIGHT, textures[RIGHT]);
  glGenTextures(1, &textures[LEFT]);
  loadTexture(T_LEFT, textures[LEFT]);
  glGenTextures(1, &textures[WOOD]);
  loadTexture(T_Cross, textures[WOOD]);
}

//the displaylist for ground, sky texture mapping
void environmentDisplayList() {
  groundSkyID = glGenLists(1);
  glNewList(groundSkyID, GL_COMPILE);
  glEnable(GL_TEXTURE_2D);
  glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT);
  glEnable(GL_COLOR_MATERIAL);

  glBindTexture(GL_TEXTURE_2D,textures[DOWN]);
  glBegin(GL_QUADS);
  glTexCoord2f(0,0); glVertex3f(-SCALE_MAPPING,-SCALE_MAPPING,-SCALE_MAPPING);
  glTexCoord2f(1,0); glVertex3f(+SCALE_MAPPING,-SCALE_MAPPING,-SCALE_MAPPING);
  glTexCoord2f(1,1); glVertex3f(+SCALE_MAPPING,+SCALE_MAPPING,-SCALE_MAPPING);
  glTexCoord2f(0,1); glVertex3f(-SCALE_MAPPING,+SCALE_MAPPING,-SCALE_MAPPING);
  glEnd();
  glBindTexture(GL_TEXTURE_2D,textures[UP]);
  glBegin(GL_QUADS);
  glTexCoord2f(0,0); glVertex3f(-SCALE_MAPPING,+SCALE_MAPPING,+SCALE_MAPPING);
  glTexCoord2f(1,0); glVertex3f(+SCALE_MAPPING,+SCALE_MAPPING,+SCALE_MAPPING);
  glTexCoord2f(1,1); glVertex3f(+SCALE_MAPPING,-SCALE_MAPPING,+SCALE_MAPPING);
  glTexCoord2f(0,1); glVertex3f(-SCALE_MAPPING,-SCALE_MAPPING,+SCALE_MAPPING);
  glEnd();
  glBindTexture(GL_TEXTURE_2D,textures[FRONT]);
  glBegin(GL_QUADS);
  glTexCoord2f(0,0); glVertex3f(+SCALE_MAPPING,-SCALE_MAPPING,+SCALE_MAPPING);
  glTexCoord2f(1,0); glVertex3f(+SCALE_MAPPING,+SCALE_MAPPING,+SCALE_MAPPING);
  glTexCoord2f(1,1); glVertex3f(+SCALE_MAPPING,+SCALE_MAPPING,-SCALE_MAPPING);
  glTexCoord2f(0,1); glVertex3f(+SCALE_MAPPING,-SCALE_MAPPING,-SCALE_MAPPING);
  glEnd();
  glBindTexture(GL_TEXTURE_2D,textures[BACK]);
  glBegin(GL_QUADS);
  glTexCoord2f(0,0); glVertex3f(-SCALE_MAPPING,+SCALE_MAPPING,+SCALE_MAPPING);
  glTexCoord2f(1,0); glVertex3f(-SCALE_MAPPING,-SCALE_MAPPING,+SCALE_MAPPING);
  glTexCoord2f(1,1); glVertex3f(-SCALE_MAPPING,-SCALE_MAPPING,-SCALE_MAPPING);
  glTexCoord2f(0,1); glVertex3f(-SCALE_MAPPING,+SCALE_MAPPING,-SCALE_MAPPING);
  glEnd();
  glBindTexture(GL_TEXTURE_2D,textures[LEFT]);
  glBegin(GL_QUADS);
  glTexCoord2f(0,0); glVertex3f(+SCALE_MAPPING,+SCALE_MAPPING,+SCALE_MAPPING);
  glTexCoord2f(1,0); glVertex3f(-SCALE_MAPPING,+SCALE_MAPPING,+SCALE_MAPPING);
  glTexCoord2f(1,1); glVertex3f(-SCALE_MAPPING,+SCALE_MAPPING,-SCALE_MAPPING);
  glTexCoord2f(0,1); glVertex3f(+SCALE_MAPPING,+SCALE_MAPPING,-SCALE_MAPPING);
  glEnd();
  glBindTexture(GL_TEXTURE_2D,textures[RIGHT]);
  glBegin(GL_QUADS);
  glTexCoord2f(1,0); glVertex3f(+SCALE_MAPPING,-SCALE_MAPPING,+SCALE_MAPPING);
  glTexCoord2f(1,1); glVertex3f(+SCALE_MAPPING,-SCALE_MAPPING,-SCALE_MAPPING);
  glTexCoord2f(0,1); glVertex3f(-SCALE_MAPPING,-SCALE_MAPPING,-SCALE_MAPPING);
  glTexCoord2f(0,0); glVertex3f(-SCALE_MAPPING,-SCALE_MAPPING,+SCALE_MAPPING);
  glEnd();

  glDisable(GL_COLOR_MATERIAL);
  glDisable(GL_TEXTURE_2D);
  glEndList();
}

/*given all points of the spline, including interval ones
  use OpenGL to draw them out use gl_line_strip*/
void drawSpline(point* spline) {
  glColor3f(1, 0, 0); //set line to red for testing
  glLineWidth(10.0); //set line width for testing
  
  glBegin(GL_LINE_STRIP);
  for (int i = 0; i < g_iNumOfSplines; i++) {
    for (int j = 0; j < numOfSplinePoints; j++) {
      point p = spline[j];
      glVertex3d(p.x, p.y, p.z);
    }
  }
  glEnd();
  glColor3f(1,1,1);
}

//the function to draw rail corner lines
void drawRailLine(point* spline) {
  
  //first line
  glBegin(GL_LINE_STRIP);
  for (int i = 0; i < g_iNumOfSplines; i++) {
    for (int j = 0; j < numOfSplinePoints; j++) {
      double f = (0.2/15)/2;
      point p = spline[j];
      point n = normalOfSpline[j];
      point b = binormalOfSpline[j];

      glVertex3d(p.x + f*(n.x + b.x), p.y + f*(n.y + b.y), p.z + f*(n.z + b.z));
    }
  }
  glEnd();

  //second line
  glBegin(GL_LINE_STRIP);
  for (int i = 0; i < g_iNumOfSplines; i++) {
    for (int j = 0; j < numOfSplinePoints; j++) {
      double f = (0.2/15)/2;
      point p = spline[j];
      point n = normalOfSpline[j];
      point b = binormalOfSpline[j];

      glVertex3d(p.x + f*(b.x - n.x), p.y + f*(b.y - n.y), p.z + f*(b.z - n.z));
    }
  }
  glEnd();

  //third line
  glBegin(GL_LINE_STRIP);
  for (int i = 0; i < g_iNumOfSplines; i++) {
    for (int j = 0; j < numOfSplinePoints; j++) {
      double f = (0.2/15)/2;
      point p = spline[j];
      point n = normalOfSpline[j];
      point b = binormalOfSpline[j];

      glVertex3d(p.x + f*(-b.x - n.x), p.y + f*(-b.y - n.y), p.z + f*(-b.z - n.z));
    }
  }
  glEnd();

  //forth line
  glBegin(GL_LINE_STRIP);
  for (int i = 0; i < g_iNumOfSplines; i++) {
    for (int j = 0; j < numOfSplinePoints; j++) {
      double f = (0.2/15)/2;
      point p = spline[j];
      point n = normalOfSpline[j];
      point b = binormalOfSpline[j];

      glVertex3d(p.x + f*(n.x - b.x), p.y + f*(n.y - b.y), p.z + f*(n.z - b.z));
    }
  }
  glEnd();

}


void drawRailSide(point p, point n, point b, point p1, point n1, point b1, double f) {
      // up
      glVertex3d(p.x+f*(n.x+b.x), p.y+f*(n.y+b.y), p.z+f*(n.z+b.z));
      glVertex3d(p1.x+f*(n1.x+b1.x), p1.y+f*(n1.y+b1.y), p1.z+f*(n1.z+b1.z));
      glVertex3d(p1.x+f*(b1.x-n1.x), p1.y+f*(b1.y-n1.y), p1.z+f*(b1.z-n1.z));
      glVertex3d(p.x+f*(b.x-n.x), p.y+f*(b.y-n.y), p.z+f*(b.z-n.z));
      
      // down
      glVertex3d(p.x+f*(n.x-b.x), p.y+f*(n.y-b.y), p.z+f*(n.z-b.z));
      glVertex3d(p1.x+f*(n1.x-b1.x), p1.y+f*(n1.y-b1.y), p1.z+f*(n1.z-b1.z));
      glVertex3d(p1.x+f*(-b1.x-n1.x), p1.y+f*(-b1.y-n1.y), p1.z+f*(-b1.z-n1.z));
      glVertex3d(p.x+f*(-b.x-n.x), p.y+f*(-b.y-n.y), p.z+f*(-b.z-n.z));
      
      // left
      glVertex3d(p.x+f*(-n.x+b.x), p.y+f*(-n.y+b.y), p.z+f*(-n.z+b.z));
      glVertex3d(p1.x+f*(-n1.x+b1.x), p1.y+f*(-n1.y+b1.y), p1.z+f*(-n1.z+b1.z));
      glVertex3d(p1.x+f*(-b1.x-n1.x), p1.y+f*(-b1.y-n1.y), p1.z+f*(-b1.z-n1.z));
      glVertex3d(p.x+f*(-b.x-n.x), p.y+f*(-b.y-n.y), p.z+f*(-b.z-n.z));
      
      // right
      glVertex3d(p.x+f*(n.x+b.x), p.y+f*(n.y+b.y), p.z+f*(n.z+b.z));
      glVertex3d(p1.x+f*(n1.x+b1.x), p1.y+f*(n1.y+b1.y), p1.z+f*(n1.z+b1.z));
      glVertex3d(p1.x+f*(n1.x-b1.x), p1.y+f*(n1.y-b1.y), p1.z+f*(n1.z-b1.z));
      glVertex3d(p.x+f*(n.x-b.x), p.y+f*(n.y-b.y), p.z+f*(n.z-b.z));
}

void railDisplayList() {
  railID = glGenLists(1);
  glNewList(railID, GL_COMPILE);

  glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT);
  glEnable(GL_COLOR_MATERIAL);

  //draw rail lines
  glLineWidth(1.0);
  glColor3f(0, 0, 0);
  drawRailLine(leftSplinePoints);
  drawRailLine(rightSplinePoints);

  glColor3f(0.82, 0.82, 0.82);
  glBegin(GL_QUADS);
  for (int i = 0; i < g_iNumOfSplines; i++) {
    for (int j = 0; j < numOfSplinePoints; j++) {
      double f = 0.008; //the dimension of rail box. 
     
      //left rail
      point p = leftSplinePoints[j];
      point n = normalOfSpline[j];
      point b = binormalOfSpline[j];
      point p1 = leftSplinePoints[j + 1];
      point n1 = normalOfSpline[j + 1];
      point b1 = binormalOfSpline[j + 1];
      drawRailSide(p, n, b, p1, n1, b1, (0.2/15)/2);

      //right rail
      p = rightSplinePoints[j];
      p1 = rightSplinePoints[j + 1];
      drawRailSide(p, n, b, p1, n1, b1, (0.2/15)/2);
    }
  }
  glEnd();

  glDisable(GL_COLOR_MATERIAL);
  glColor3f(1.0, 1.0, 1.0);
  glEndList();
}


void drawCross(point p, point n, point b, point np, double h, double w) {
  glBegin(GL_QUADS);//draw all 6 faces.
        //front
        glTexCoord2f(0.0, 0.0); 
        glVertex3d(p.x-w/2.0*n.x, p.y-w/2.0*n.y, p.z-w/2.0*n.z);
        glTexCoord2f(0.0, 1.0); 
        glVertex3d(p.x-w/2.0*n.x-h*b.x, p.y-w/2.0*n.y-h*b.y, p.z-w/2.0*n.z-h*b.z);
        glTexCoord2f(1.0, 1.0); 
        glVertex3d(p.x+w/2.0*n.x-h*b.x, p.y+w/2.0*n.y-h*b.y, p.z+w/2.0*n.z-h*b.z);
        glTexCoord2f(1.0, 0.0); 
        glVertex3d(p.x+w/2.0*n.x, p.y+w/2.0*n.y, p.z+w/2.0*n.z);
        //back
        glTexCoord2f(0.0, 0.0); 
        glVertex3d(np.x-w/2.0*n.x, np.y-w/2.0*n.y, np.z-w/2.0*n.z);
        glTexCoord2f(0.0, 1.0); 
        glVertex3d(np.x-w/2.0*n.x-h*b.x, np.y-w/2.0*n.y-h*b.y, np.z-w/2.0*n.z-h*b.z);
        glTexCoord2f(1.0, 1.0); 
        glVertex3d(np.x+w/2.0*n.x-h*b.x, np.y+w/2.0*n.y-h*b.y, np.z+w/2.0*n.z-h*b.z);
        glTexCoord2f(1.0, 0.0); 
        glVertex3d(np.x+w/2.0*n.x, np.y+w/2.0*n.y, np.z+w/2.0*n.z);
        //bottom
        glTexCoord2f(0.0, 0.0); 
        glVertex3d(p.x-w/2.0*n.x-h*b.x, p.y-w/2.0*n.y-h*b.y, p.z-w/2.0*n.z-h*b.z);
        glTexCoord2f(0.0, 1.0); 
        glVertex3d(np.x-w/2.0*n.x-h*b.x, np.y-w/2.0*n.y-h*b.y, np.z-w/2.0*n.z-h*b.z);
        glTexCoord2f(1.0, 1.0); 
        glVertex3d(np.x+w/2.0*n.x-h*b.x, np.y+w/2.0*n.y-h*b.y, np.z+w/2.0*n.z-h*b.z);
        glTexCoord2f(1.0, 0.0); 
        glVertex3d(p.x+w/2.0*n.x-h*b.x, p.y+w/2.0*n.y-h*b.y, p.z+w/2.0*n.z-h*b.z);
        //up
        glTexCoord2f(0.0, 0.0); 
        glVertex3d(p.x-w/2.0*n.x, p.y-w/2.0*n.y, p.z-w/2.0*n.z);
        glTexCoord2f(0.0, 1.0); 
        glVertex3d(np.x-w/2.0*n.x, np.y-w/2.0*n.y, np.z-w/2.0*n.z);
        glTexCoord2f(1.0, 1.0); 
        glVertex3d(np.x+w/2.0*n.x, np.y+w/2.0*n.y, np.z+w/2.0*n.z);
        glTexCoord2f(1.0, 0.0); 
        glVertex3d(p.x+w/2.0*n.x, p.y+w/2.0*n.y, p.z+w/2.0*n.z);
        //right
        glTexCoord2f(0.0, 0.0); 
        glVertex3d(p.x+w/2.0*n.x, p.y+w/2.0*n.y, p.z+w/2.0*n.z);
        glTexCoord2f(0.0, 1.0); 
        glVertex3d(np.x+w/2.0*n.x, np.y+w/2.0*n.y, np.z+w/2.0*n.z);
        glTexCoord2f(1.0, 1.0); 
        glVertex3d(np.x+w/2.0*n.x-h*b.x, np.y+w/2.0*n.y-h*b.y, np.z+w/2.0*n.z-h*b.z);
        glTexCoord2f(1.0, 0.0); 
        glVertex3d(p.x+w/2.0*n.x-h*b.x, p.y+w/2.0*n.y-h*b.y, p.z+w/2.0*n.z-h*b.z);
        //left
        glTexCoord2f(0.0, 0.0); 
        glVertex3d(p.x-w/2.0*n.x, p.y-w/2.0*n.y, p.z-w/2.0*n.z);
        glTexCoord2f(0.0, 1.0); 
        glVertex3d(np.x-w/2.0*n.x, np.y-w/2.0*n.y, np.z-w/2.0*n.z);
        glTexCoord2f(1.0, 1.0); 
        glVertex3d(np.x-w/2.0*n.x-h*b.x, np.y-w/2.0*n.y-h*b.y, np.z-w/2.0*n.z-h*b.z);
        glTexCoord2f(1.0, 0.0); 
        glVertex3d(p.x-w/2.0*n.x-h*b.x, p.y-w/2.0*n.y-h*b.y, p.z-w/2.0*n.z-h*b.z);
        glEnd();

}

void crossDisplayList() {
  //create the displaylist
  crossID = glGenLists(1);
  glNewList(crossID, GL_COMPILE);
  glEnable(GL_TEXTURE_2D);

  int counter = 0; //use for seperate crosses

  for (int i = 0; i < g_iNumOfSplines; i++) {
    for (int j = 0; j < numOfSplinePoints - CROSS_LENGTH; j++) {
      if (counter == DISTANCE_BETWEEN_CROSSES) {
        counter = 0;
        //get all the vector points we need
        point p = splinePoints[j];
        point n = normalOfSpline[j];
        point b = binormalOfSpline[j];

        point np = splinePoints[j + CROSS_LENGTH];
        double h = CROSS_HEIGHT;
        double w = CROSS_WIDTH;

        p.x = p.x - (RAIL_SIZE) * b.x;
        p.y = p.y - (RAIL_SIZE) * b.y;
        p.z = p.z - (RAIL_SIZE) * b.z;
        np.x = np.x - (RAIL_SIZE) * b.x;
        np.y = np.y - (RAIL_SIZE) * b.y;
        np.z = np.z - (RAIL_SIZE) * b.z;

        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT);
        glEnable(GL_COLOR_MATERIAL);

        glBindTexture(GL_TEXTURE_2D, textures[WOOD]); // map the wood texture to cross section
        
        drawCross(p, n, b, np, h, w); //call the drawing fucntion

        glDisable(GL_COLOR_MATERIAL);
      }
      counter++;
    }
  }
  glDisable(GL_TEXTURE_2D);
  glEndList();
}


int loadSplines(char *argv) {
  char *cName = (char *)malloc(128 * sizeof(char));
  FILE *fileList;
  FILE *fileSpline;
  int iType, i = 0, j, iLength;


  /* load the track file */
  fileList = fopen(argv, "r");
  if (fileList == NULL) {
    printf ("can't open file\n");
    exit(1);
  }
  
  /* stores the number of splines in a global variable */
  fscanf(fileList, "%d", &g_iNumOfSplines);

  g_Splines = (struct spline *)malloc(g_iNumOfSplines * sizeof(struct spline));

  /* reads through the spline files */
  for (j = 0; j < g_iNumOfSplines; j++) {
    i = 0;
    fscanf(fileList, "%s", cName);
    fileSpline = fopen(cName, "r");

    if (fileSpline == NULL) {
      printf ("can't open file\n");
      exit(1);
    }

    /* gets length for spline file */
    fscanf(fileSpline, "%d %d", &iLength, &iType);

    /* allocate memory for all the points */
    g_Splines[j].points = (struct point *)malloc(iLength * sizeof(struct point));
    g_Splines[j].numControlPoints = iLength;

    /* saves the data to the struct */
    while (fscanf(fileSpline, "%lf %lf %lf", 
	   &g_Splines[j].points[i].x, 
	   &g_Splines[j].points[i].y, 
	   &g_Splines[j].points[i].z) != EOF) {
      i++;
    }
  }

  free(cName);

  return 0;
}

/*callback function of menu manipulation*/
void menufunc(int value)
{
  switch (value)
  {
    case 0:
      exit(0);
      break;
  }
}

/* callback function for keyboard manipulation*/
void keyPress(unsigned char key, int x, int y) {
  //key 'X' or 'x' for take screen shot.s
  if (key == 120 || key == 88) {
      isScreenShot = true;
      current_time = clock();
      std::cout << current_time << std::endl;
  }
}

/*calculate the speed by gravity constant g
http://run.usc.edu/cs420-s14/assignments/assign2/RollerCoasterVelocity.pdf
*/
double getSpeedbyG(double currentU, point p, point t) {
  //based on the formula provided
  double numerator = sqrt(2 * 9.78 * (MAX_HEIGHT - p.z));
  double denominator = sqrt(pow(t.x, 2) + pow(t.y, 2) + pow(t.z, 2));
  double retval = currentU + TIMESTEP * (numerator / denominator);
  return retval;
}


void setUpCamera() {
  point p, t, n, b, e, c;
  p = splinePoints[(int) (ride_counter + 1000 * currentU)];
  t = normalize(tangentOfSpline[(int) (ride_counter + 1000 * currentU)]);
  n = normalOfSpline[(int) (ride_counter + 1000 * currentU)];
  b = binormalOfSpline[(int) (ride_counter + 1000 * currentU)];

  //camera center vector
  //eye position plus tangent vector
  c.x = p.x + EYE_UP * b.x + t.x;
  c.y = p.y + EYE_UP * b.y + t.y;
  c.z = p.z + EYE_UP * b.z + t.z;

  //eye position
  e.x = p.x + EYE_UP * b.x;
  e.y = p.y + EYE_UP * b.y;
  e.z = p.z + EYE_UP * b.z;

  //setup camera
  gluLookAt(e.x, e.y, e.z, c.x, c.y, c.z, b.x, b.y, b.z);

  //get the non-normalized tangent vector
  t = tangentOfSpline[(int) (ride_counter + 1000 * currentU)];
  
  //calculate the new parameter based on current position and tangent
  double newU = getSpeedbyG(currentU, p, t);

  //update newU based on speed parameter
  if (newU - currentU < SPEED/1000) {
    newU = SPEED/1000 + currentU;
  }

  /*if finished a spline segment, update counter, reset parameter
  */
  if (newU >= 1.0) {
    ride_counter += 1000;
    newU = 0.0;
  }

  /*if all spline points have been visited*/
  if (ride_counter >= numOfSplinePoints) {
    ride_counter = 0;
  }
  currentU = newU;
}


void display() {
  //call screen shots function here
  clock_t this_time = clock(); //get current time

  //if there is more than 15ms elapsed and we can do the screen shots now
  if (this_time - current_time >= (double)1/15 * 1000 && numScreenShots <= 999 && isScreenShot == true) {
    char fname [2048];
    sprintf(fname, "imageOutput/anim%04d.jpg", numScreenShots);
    saveScreenshot(fname);
    numScreenShots += 1;
  }

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  
  glPushMatrix();
  glLoadIdentity();
  
  //set up the camera for ride animation
  setUpCamera(); 

  //call display list to render sky ground, rails, and cross sections.
  glCallList(groundSkyID);
  glCallList(railID);
  glCallList(crossID);

  glPopMatrix();
  glFlush();
  glutSwapBuffers();
}


/* callback function of doing idle*/
void doIdle() {
  glutPostRedisplay();
}

/* converts mouse drags into information about 
rotation/translation/scaling */
void mousedrag(int x, int y)
{
  int vMouseDelta[2] = {x-g_vMousePos[0], y-g_vMousePos[1]};
  
  switch (g_ControlState)
  {
    case TRANSLATE:  
      if (g_iLeftMouseButton)
      {
        g_vLandTranslate[0] += vMouseDelta[0]*0.10;
        g_vLandTranslate[1] -= vMouseDelta[1]*0.10;
      }
      if (g_iMiddleMouseButton)
      {
        g_vLandTranslate[2] += vMouseDelta[1]*0.01;
      }
      break;
    case ROTATE:
      if (g_iLeftMouseButton)
      {
        g_vLandRotate[0] += vMouseDelta[1];
        g_vLandRotate[1] += vMouseDelta[0];
      }
      if (g_iMiddleMouseButton)
      {
        g_vLandRotate[2] += vMouseDelta[1];
      }
      break;
    case SCALE:
      if (g_iLeftMouseButton)
      {
        g_vLandScale[0] *= 1.0+vMouseDelta[0]*0.01;
        g_vLandScale[1] *= 1.0-vMouseDelta[1]*0.01;
      }
      if (g_iMiddleMouseButton)
      {
        g_vLandScale[2] *= 1.0-vMouseDelta[1]*0.01;
      }
      break;
  }
  g_vMousePos[0] = x;
  g_vMousePos[1] = y;
}
void mouseidle(int x, int y)
{
  g_vMousePos[0] = x;
  g_vMousePos[1] = y;
}
void mousebutton(int button, int state, int x, int y)
{

  switch (button)
  {
    case GLUT_LEFT_BUTTON:
      g_iLeftMouseButton = (state==GLUT_DOWN);
      break;
    case GLUT_MIDDLE_BUTTON:
      g_iMiddleMouseButton = (state==GLUT_DOWN);
      break;
    case GLUT_RIGHT_BUTTON:
      g_iRightMouseButton = (state==GLUT_DOWN);
      break;
  }
 
  switch(glutGetModifiers())
  {
    case GLUT_ACTIVE_CTRL:
      g_ControlState = TRANSLATE;
      break;
    case GLUT_ACTIVE_SHIFT:
      g_ControlState = SCALE;
      break;
    default:
      g_ControlState = ROTATE;
      break;
  }

  g_vMousePos[0] = x;
  g_vMousePos[1] = y;
}

/*callback function of window reshape*/
void reshape(int w, int h) {
  GLfloat aspect = (GLfloat) w / (GLfloat) h;
  glViewport(0, 0, w, h);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(90.0f, aspect, 0.01, 1000.0);
  glMatrixMode(GL_MODELVIEW);
}

/*initiation function*/
void myinit() {

  glEnable(GL_DEPTH_TEST);
  glEnable(GL_TEXTURE_2D);
  
  //generate spline points and textures
  splineGenerator(g_Splines);
  textureGnerator();

  //create rail, 360 degree environment and cross bar displaylist
  railDisplayList();
  environmentDisplayList();
  crossDisplayList();
}

int main (int argc, char ** argv)
{
  if (argc<2)
  {  
  printf ("usage: %s <trackfile>\n", argv[0]);
  exit(0);
  }

  loadSplines(argv[1]);


  glutInit(&argc,argv);
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_DEPTH | GLUT_RGB);
  
  /*set window size*/
  glutInitWindowSize(640, 480);

  /*set window position*/
  glutInitWindowPosition(0,0);

  /*now create a window*/
  glutCreateWindow("Roller Coaster of Yuzhou Ge");

  /* allow the user to quit using the right mouse button menu */
   g_iMenuId = glutCreateMenu(menufunc);
   glutSetMenu(g_iMenuId);
   glutAddMenuEntry("Quit",0);
   glutAttachMenu(GLUT_RIGHT_BUTTON);
  
  /*callback functions for keyboard manipulation*/
   glutKeyboardFunc(keyPress);

  /*callback functions for mouse manipulation*/
   glutMotionFunc(mousedrag);
   glutPassiveMotionFunc(mouseidle);
   glutMouseFunc(mousebutton);
  
  /*call back functions for display*/
   glutReshapeFunc(reshape);
   glutIdleFunc(doIdle);
   glutDisplayFunc(display);

  /* do initialization */
   myinit();

   glutMainLoop();
   return 0;
}




















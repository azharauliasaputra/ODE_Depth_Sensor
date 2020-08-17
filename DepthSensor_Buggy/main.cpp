//
//  main.cpp
//  DepthSensor_Buggy
//
//  Created by Azhar Aulia Saputra on 2020/05/31.
//  Copyright © 2020 Azhar Aulia Saputra. All rights reserved.
//



#include <ode/ode.h>
#include <drawstuff/drawstuff.h>

#include "projection.h"

#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif

// select correct drawing functions

#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#endif


// some constants

#define LENGTH 0.7    // chassis length
#define WIDTH 0.5    // chassis width
#define HEIGHT 0.2    // chassis height
#define RADIUS 0.18    // wheel radius
#define STARTZ 0.5    // starting height of chassis
#define CMASS 1        // chassis mass
#define WMASS 0.2    // wheel mass


#define rad 57.295779513082320876798154814105
#define phi 3.1415926535897932384626433832795
#define sp2 1.4142135623730950488016887242097


// dynamics and collision objects (chassis, 3 wheels, environment)

static dWorldID world;
static dSpaceID space;
static dBodyID body[4];
static dJointID joint[3];    // joint[0] is the front wheel
static dJointGroupID contactgroup;
static dGeomID ground;
static dSpaceID car_space;
static dGeomID box[1];
static dGeomID sphere[3];
static dGeomID ground_box;

typedef struct {
    
    dBodyID body;
    dGeomID geom;
    dJointID joint;
    dReal size[3];
    dReal weight;
    dReal position[3];
    
} MyObject;

bool sensorEnable = 1;
bool showSimulation = 1;
int startScan = 1;
// things that the user controls

static dReal speed=0,steer=0;    // user commands



// this is called by dSpaceCollide when two objects in space are
// potentially colliding.




// start simulation - set viewpoint

static void start()
{
  dAllocateODEDataForThread(dAllocateMaskAll);

  static float xyz[3] = {0.8317f,-0.9817f,0.8000f};
  static float hpr[3] = {121.0000f,-27.5000f,0.0000f};
  dsSetViewpoint (xyz,hpr);
  printf ("Press:\t'a' to increase speed.\n"
      "\t'z' to decrease speed.\n"
      "\t',' to steer left.\n"
      "\t'.' to steer right.\n"
      "\t' ' to reset speed and steering.\n"
      "\t'1' to save the current state to 'state.dif'.\n");
}


// called when a key pressed

static void command (int cmd)
{
  switch (cmd) {
  case 'a': case 'A':
    speed += 0.3;
    break;
  case 'z': case 'Z':
    speed -= 0.3;
    break;
  case ',':
    steer -= 0.5;
    break;
  case '.':
    steer += 0.5;
    break;
  case ' ':
    speed = 0;
    steer = 0;
    break;
  case '1': {
      FILE *f = fopen ("state.dif","wt");
      if (f) {
        dWorldExportDIF (world,f,"");
        fclose (f);
      }
    }
  }
}

int multiplicationRotation(const dReal *R, double pos0[], double pos1[])
{
    double a[3][3];
    
    for(int i=0; i<3; i++){
        a[0][i] = R[0+i];
        a[1][i] = R[4+i];
        a[2][i] = R[8+i];
    }
    int i,j;
    double sum;
    
    for(i=0;i<3;i++)
    {
        sum=0;
        for(j=0;j<3;j++)
        {
            sum=sum+a[i][j]*pos0[j];
//            printf("%.2f\t%.2f\t",a[i][j],pos0[j]);
        }
        pos1[i]=sum;
    }
//     for(i=0;i<3;i++)
//     {
//         printf("%.2f\t",pos0[i]);
//         printf("%.2f\t",pos1[i]);
//         printf("\n");
//     }
    return 0;
}

#define SNUM 1
#define GAIN 300
#define yNUM 1
#define xNUM 300

#define yMin -M_PI/6
//#define yMin 0.0
#define yMax M_PI/6
//#define yMax M_PI/2
//#define xMin -0.41421
//#define xMax 0.41421
#define xMin -M_PI/4
#define xMax M_PI/4

//#define yRes (yMax - yMin)/(yNUM-1)
#define xRes (xMax - xMin)/(xNUM-1)

dContactGeom c[100000];
MyObject pico_data[SNUM][300][600];
MyObject Sensor[SNUM];
dReal dist[SNUM][300][300];
dGeomID coob[300];
int conum = 0;


double PointCloudActive[500000][3];
int pointCloudActiveNum = 0;

void sensor(int step){
    
    int h,i,j,k;
    static int flag = 1;
    double error;
    static int t = 0;
    int n = 0;
    
    int ct = 0;
    dVector3 bbpos;
    dVector3 sides;
    dReal * Pos;
    const dReal * Rot = dBodyGetRotation(Sensor[0].body);
    const dReal * Posi = dBodyGetPosition(Sensor[0].body);

    dMatrix3 R_l;
    dRFromAxisAndAngle(R_l ,0,0, 1, -M_PI/2);
//    for(i=0; i<12; i++){
//        printf("%.2f\t",R_l[i]);
//    }
//    printf("\n");
//    dReal pitch, yaw, roll;
//    dReal r11, r12, r13, r21, r22, r23, r31, r32, r33;
//    r11 = Rot[0]; r12 = Rot[1]; r13 = Rot[2];
//    r21 = Rot[4]; r22 = Rot[5]; r23 = Rot[6];
//    r31 = Rot[8]; r32 = Rot[9]; r33 = Rot[10];
//    pitch = atan2(-r31, sqrt(r32*r32 + r33*r33));
//    yaw = atan2(r21, r11);
//    roll = atan2(r32, r33);
//    printf("pitch = %.3f\troll = %.3f\tyaw = %.3f\n",pitch, roll, yaw);

    dReal tmp = dJointGetHingeAngle(Sensor[0].joint);
    dReal u = 10 * (0.4-tmp);
    dJointSetHingeParam(Sensor[0].joint, dParamVel, u);
    dJointSetHingeParam(Sensor[0].joint, dParamFMax, 1000);

    dsSetColor(0,0,1.0);
    dGeomBoxGetLengths(Sensor[0].geom,sides);
    dsDrawBox(dBodyGetPosition(Sensor[0].body),
              dBodyGetRotation(Sensor[0].body),sides);
    
    pointCloudActiveNum = 0;
    int ScanNumX = 1;
    int ScanNumY = 300;
    for (int i=0; i<3; i++) sides[i] = 0.005;
    int pointCount = 0;
//    for()
    double aty = 0;
    double atx = 0;
    int countY = 0;
    for(aty = yMin; aty < yMin+(yMax-yMin); aty+=(yMax-yMin)/ScanNumY){
        countY++;
        for(atx = xMin; atx < xMin + (xMax-xMin)/xNUM; atx+=(xMax-xMin)/(ScanNumX*xNUM)){
        
    for(i=0; i<SNUM; i++){
        for(j=0; j<yNUM; j++){
//            double angle = aty + M_PI + M_PI/6 + 1;
            double angle = aty;
            for(k=0; k<xNUM; k++){
                ct = 0;
                dMatrix3 Ra,Rb,Rc;
                
                double angle2 = -0.6+k*(1.2/(double)(xNUM));
                double rot0[3] = {1,angle2, angle};
//                double rot0[3] = {1,0, 0};
                double rot[3] = {1,0, 0};
                double pos[3] = {Posi[0],Posi[1], Posi[2]};
//                double pos[3] = {0,0,1};
                multiplicationRotation(Rot, rot0, rot);
//
                dGeomRaySet(pico_data[i][j][k].geom, pos[0],pos[1], pos[2], rot[0], rot[1], rot[2]);
                
                
                if(countY%10 == 0 && k%10 == 0){
                    dVector3 end,start,direction;

                    dGeomRayGet(pico_data[i][j][k].geom, start, direction);
//                    for(int q = 0; q < 3; q++ ){
//                        direction[q] = rot[q];
//                        start[q] = pos[q];
//                    }
                    end[0] = start[0] + (direction[0]*2);
                    end[1] = start[1] + (direction[1]*2);
                    end[2] = start[2] + (direction[2]*2);
                    end[3] = start[3] + (direction[3]*2);

                    dsSetColorAlpha(1.0, 1.0, 0.0, 1.5);
//                    dsDrawLineD(start, end);
                }
                
//                dRFromAxisAndAngle ()
//                multiplicationRotation(Rc, Ra, Rb);
//                dGeomSetRotation(pico_data[i][j][k].geom,Ra);
                
                for(h=0;h<conum;h++){
                    int numc = dCollide(pico_data[i][j][k].geom, coob[h], 1, &c[i], sizeof( dContact ) );
                    if (numc > 0) {
                        if(h == 0){
                            dist[i][j][k] = c[i].depth; // depthはrayの始点位置からの距離
                            Pos = c[i].pos;
                            for (int i=0; i<3; i++) bbpos[i] = Pos[i];
                        }
                        else if(ct == 0){
                            dist[i][j][k] = c[i].depth;
                            Pos = c[i].pos;
                            for (int i=0; i<3; i++) bbpos[i] = Pos[i];
                        }else if(dist[i][j][k] > c[i].depth){
                            dist[i][j][k] = c[i].depth;
                            Pos = c[i].pos;
                            for (int i=0; i<3; i++) bbpos[i] = Pos[i];
                        }
                        ct++;
                    }
                }
                if(ct == 0){
                    dist[i][j][k] = 0.0;
                    for (int i=0; i<3; i++) bbpos[i] = 0;
                }
                if(showSimulation && countY%10 == 0 && k%10 == 0){
                    dMatrix3 RI;
                    dRSetIdentity (RI);
                    dsSetColor(1,0,0);
                    dsDrawBox(bbpos,RI,sides);
                }
                if(startScan && ct > 0){
                    for (int i=0; i<3; i++)
//                    PointCloudScan[step][pointCloudActiveNum][i] = bbpos[i];
                        PointCloudActive[pointCloudActiveNum][i] = bbpos[i];
                    
                }
                pointCloudActiveNum++;
            }
        }
    }
    }
    }
}

void makeDataSensor(){

    dMass m;
    
    conum = 0;
    coob[conum] = ground;
    conum++;
    
    Sensor[0].body = dBodyCreate (world);
    dBodySetPosition (Sensor[0].body,LENGTH/2,0,STARTZ+0.3);
    dMassSetBox (&m,1,0.01,0.04,0.02);
    dMassAdjust (&m,0.01);
    dBodySetMass (Sensor[0].body,&m);
    Sensor[0].geom = dCreateBox (0,0.01,0.04,0.02);
    dGeomSetBody (Sensor[0].geom,Sensor[0].body);
    
    Sensor[0].joint = dJointCreateHinge(world, 0);
    dJointAttach(Sensor[0].joint, Sensor[0].body, body[0]);
    dJointSetHingeAnchor (Sensor[0].joint,LENGTH/2,0,STARTZ+0.3);
    dJointSetHingeAxis (Sensor[0].joint,0,1,0);
    
    int i,j,k;
    double *pos = (double *) malloc(sizeof (double) * 3);
    double *posz = (double *) malloc(sizeof (double) * 3);
    double *pos1 = (double *) malloc(sizeof (double) * 3);
    pos[0] = 0;
    pos[1] = 0;
    pos[2] = 1;
    
    pos1[1] = 0;
    float yRes = (yMax - yMin)/(yNUM-1);
    for(i=0; i<SNUM; i++){
        for(j=0; j<yNUM; j++){
            
            if(yNUM == 1)
                pos1[2] = tan(yMin+(yMax - yMin)/2);
            else
                pos1[2] = tan(yMin+j*yRes-M_PI/4);
            
            double angle = yMin+(double)j*yRes + M_PI - M_PI/6;
            if(yNUM == 1)
                angle = yMin + M_PI - M_PI/6;
            
            pos1 = rotation_x(angle, pos);
            for(k=0; k<xNUM; k++){
                double angle2 = xMin+k*xRes;
                pico_data[i][j][k].geom = dCreateRay(space, 2);
                dGeomRaySet(pico_data[i][j][k].geom, 0, 0, 0.2, 1, angle2, 0);
//                dGeomSetPosition(pico_data[i][j][k].geom , 0, 0, 0.7);
//                dGeomSetBody(pico_data[i][j][k].geom, pico[i].body);
//                dGeomSetOffsetPosition(pico_data[i][j][k].geom, 0.0, 0.0, 0.0);
                dMatrix3 R_l;
//                pos1[0] = tan(xMin+k*xRes);
//                pos1 = rotation_z(xMin+(double)k*xRes, pos);
                
//                float *pos1 = rotation_y((-121+(yRes*(dReal)j))/rad, pos);
//                printf("%.2f\t%.2f\t%.2f\n", pos1[0], pos1[1], pos1[2]);
//                float *pos2 = rotation_z(0, pos1);
//                printf("%.2f\t%.2f\t%.2f\n", pos2[0], pos2[1], pos2[2]);
                
                
                double *pos3 = crossProductD(pos, pos1);
                float ang = acos((pos[0]*pos1[0] + pos[1]*pos1[1] + pos[2]*pos1[2])/(norm(pos, 3) * norm(pos1, 3)));
                
//                dRFrom2Axes(R_l, 0.0, 0.0, 0.0, (-22.5+(xRes*(dReal)k))/rad, 0.0, (-22.5+(xRes*(dReal)k))/rad);
//                dRFromEulerAngles(R_l , cos((121-(yRes*(dReal)j))/rad)*(-22.5+(xRes*(dReal)k))/rad, (121-(yRes*(dReal)j))/rad, sin((121-(yRes*(dReal)j))/rad)*(-22.5+(xRes*(dReal)k))/rad);
//                dGeomSetOffsetRotation(pico_data[i][j][k].geom,R_l);
//                printf("%.2f\t%.2f\t%.2f\n",pos1[0],pos1[1],pos1[2]);
//                printf("%.2f\t%.2f\t%.2f\n",pos3[0],pos3[1],pos3[2]);
//                dRFromAxisAndAngle(R_l ,pos3[0], pos3[1], pos3[2], ang);
//                dRFrom2Axes (R_l, pos[0],pos[1],pos[2],pos1[0],pos1[1],pos1[2]);
//                dRFromEulerAngles (R_l,angle, angle2,0);
//                dRFromAxisAndAngle(R_l ,0,1,0, 0.1);
//                dGeomSetOffsetRotation(pico_data[i][j][k].geom,R_l);
//                dGeomSetRotation(pico_data[i][j][k].geom,R_l);
            }
        }
    }
}

void drawDataSensor(){
    int i,j,k;
    for(i=0; i<SNUM; i++){
        if(i == 0){
            dsSetColorAlpha(1.0, 1.0, 0.0, 0.5);
        }
        if(i == 1){
            dsSetColorAlpha(0.5, 1.0, 0.5, 0.5);
        }
        if(i == 2){
            dsSetColorAlpha(0.1, 0.3, 0.3, 0.5);
        }
        if(i == 3){
            dsSetColorAlpha(0.3, 0.3, 0.8, 0.5);
        }
        for(j=0; j<yNUM; j+=1){
            for(k=0; k<xNUM; k+=1){
                
                dVector3 start,direction,end;
                dGeomRayGet(pico_data[i][j][k].geom, start, direction);
                dReal length = dGeomRayGetLength(pico_data[i][j][k].geom);
                
                end[0] = start[0] + (direction[0]*length);
                end[1] = start[1] + (direction[1]*length);
                end[2] = start[2] + (direction[2]*length);
                end[3] = start[3] + (direction[3]*length);
                
//                dsDrawLineD(start, end);
            }
        }
    }
}


void drawRobot(){

    int i;
    dsSetColor (0,1,1);
    dsSetTexture (DS_WOOD);
    dReal sides[3] = {LENGTH,WIDTH,HEIGHT};
    dsDrawBox (dBodyGetPosition(body[0]),dBodyGetRotation(body[0]),sides);
    dsSetColor (1,1,1);
    for (i=1; i<=3; i++) dsDrawCylinder (dBodyGetPosition(body[i]),
                         dBodyGetRotation(body[i]),0.02f,RADIUS);
}
void makeRobot(){

    int i;
    dMass m;
    
    // chassis body
    body[0] = dBodyCreate (world);
    dBodySetPosition (body[0],0,0,STARTZ);
    dMassSetBox (&m,1,LENGTH,WIDTH,HEIGHT);
    dMassAdjust (&m,CMASS);
    dBodySetMass (body[0],&m);
    box[0] = dCreateBox (0,LENGTH,WIDTH,HEIGHT);
    dGeomSetBody (box[0],body[0]);

    // wheel bodies
    for (i=1; i<=3; i++) {
      body[i] = dBodyCreate (world);
      dQuaternion q;
      dQFromAxisAndAngle (q,1,0,0,M_PI*0.5);
      dBodySetQuaternion (body[i],q);
      dMassSetSphere (&m,1,RADIUS);
      dMassAdjust (&m,WMASS);
      dBodySetMass (body[i],&m);
      sphere[i-1] = dCreateSphere (0,RADIUS);
      dGeomSetBody (sphere[i-1],body[i]);
    }
    dBodySetPosition (body[1],0.5*LENGTH,0,STARTZ-HEIGHT*0.5);
    dBodySetPosition (body[2],-0.5*LENGTH, WIDTH*0.5,STARTZ-HEIGHT*0.5);
    dBodySetPosition (body[3],-0.5*LENGTH,-WIDTH*0.5,STARTZ-HEIGHT*0.5);

    // front and back wheel hinges
    for (i=0; i<3; i++) {
      joint[i] = dJointCreateHinge2 (world,0);
      dJointAttach (joint[i],body[0],body[i+1]);
      const dReal *a = dBodyGetPosition (body[i+1]);
      dJointSetHinge2Anchor (joint[i],a[0],a[1],a[2]);
      dJointSetHinge2Axis1 (joint[i],0,0,1);
      dJointSetHinge2Axis2 (joint[i],0,1,0);
    }

    // set joint suspension
    for (i=0; i<3; i++) {
      dJointSetHinge2Param (joint[i],dParamSuspensionERP,0.4);
      dJointSetHinge2Param (joint[i],dParamSuspensionCFM,0.8);
    }

    // lock back wheels along the steering axis
    for (i=1; i<3; i++) {
      // set stops to make sure wheels always stay in alignment
      dJointSetHinge2Param (joint[i],dParamLoStop,0);
      dJointSetHinge2Param (joint[i],dParamHiStop,0);
      // the following alternative method is no good as the wheels may get out
      // of alignment:
      //   dJointSetHinge2Param (joint[i],dParamVel,0);
      //   dJointSetHinge2Param (joint[i],dParamFMax,dInfinity);
    }

    // create car space and add it to the top level space
    car_space = dSimpleSpaceCreate (space);
    dSpaceSetCleanup (car_space,0);
    dSpaceAdd (car_space,box[0]);
    dSpaceAdd (car_space,sphere[0]);
    dSpaceAdd (car_space,sphere[1]);
    dSpaceAdd (car_space,sphere[2]);

}
// simulation loop
void controlBuggy(){
    dJointSetHinge2Param (joint[0],dParamVel2,-speed);
    dJointSetHinge2Param (joint[0],dParamFMax2,0.1);

    // steering
    dReal v = steer - dJointGetHinge2Angle1 (joint[0]);
    if (v > 0.1) v = 0.1;
    if (v < -0.1) v = -0.1;
    v *= 10.0;
    dJointSetHinge2Param (joint[0],dParamVel,v);
    dJointSetHinge2Param (joint[0],dParamFMax,0.2);
    dJointSetHinge2Param (joint[0],dParamLoStop,-0.75);
    dJointSetHinge2Param (joint[0],dParamHiStop,0.75);
    dJointSetHinge2Param (joint[0],dParamFudgeFactor,0.1);
}
static void nearCallback (void *data, dGeomID o1, dGeomID o2)
{
     int i, j;
  //int c;
  static const int N = 100;
  double pos[3], d;
  dContact contact[N];
  
  dBodyID b1 = dGeomGetBody(o1);
  dBodyID b2 = dGeomGetBody(o2);
  if (b1 && b2 && dAreConnected(b1, b2)) return;
  
  int check = 0;
  int n = dCollide(o1, o2, N, &contact[0].geom, sizeof(dContact));

  if (n > 0) {
      int i,j,k;
      if(sensorEnable)
          for(i=0; i<SNUM; i++){
              for(j=0; j<yNUM; j++){
                  for(k=0; k<xNUM; k++){
                      if (o1==pico_data[i][j][k].geom || o2==pico_data[i][j][k].geom) {
                          check = 1;
                      }
                      if(check == 1) break;
                  }
              }
          }
      if(check == 0){
          for (int i = 0; i < n; i++) {
              contact[i].surface.mode = dContactSlip1 | dContactSlip2 | dContactSoftERP | dContactSoftCFM | dContactApprox1;
              contact[i].surface.mu = dInfinity;
              contact[i].surface.slip1 = 0.005;
              contact[i].surface.slip2 = 0.01;
              contact[i].surface.soft_erp = 0.1;
              contact[i].surface.soft_cfm = 1e-4;
              dJointID c = dJointCreateContact(world, contactgroup, &contact[i]);
              dJointAttach(c, dGeomGetBody(contact[i].geom.g1),dGeomGetBody(contact[i].geom.g2));
          }
      }
  }
}
static void simLoop (int pause)
{
    if (!pause) {
    // motor
        controlBuggy();

        dSpaceCollide (space,0,&nearCallback);
        dWorldStep (world,0.05);

        // remove all contact joints
        dJointGroupEmpty (contactgroup);
    }
    
    drawRobot();
    
    if(sensorEnable)
        sensor(0);
//    if(sensorEnable)
//        drawDataSensor();

}


int main (int argc, char **argv)
{

  // setup pointers to drawstuff callback functions
  dsFunctions fn;
  fn.version = DS_VERSION;
  fn.start = &start;
  fn.step = &simLoop;
  fn.command = &command;
  fn.stop = 0;
  fn.path_to_textures = "/Users/Azhar/ode-0.13/drawstuff/textures";

  // create world
  dInitODE2(0);
  world = dWorldCreate();
  space = dHashSpaceCreate (0);
  contactgroup = dJointGroupCreate (0);
  dWorldSetGravity (world,0,0,-0.5);
  ground = dCreatePlane (space,0,0,1,0);

    makeRobot();
    if(sensorEnable){
        makeDataSensor();
    }
    
  // run simulation
  dsSimulationLoop (argc,argv,1000,1000,&fn);

  dGeomDestroy (box[0]);
  dGeomDestroy (sphere[0]);
  dGeomDestroy (sphere[1]);
  dGeomDestroy (sphere[2]);
  dJointGroupDestroy (contactgroup);
  dSpaceDestroy (space);
  dWorldDestroy (world);
  dCloseODE();
  return 0;
}

#define GL_SILENCE_DEPRECATION
#include "Visualizer_gl.h"
#include </opt/homebrew/include/GLFW/glfw3.h>
#include <OpenGL/gl.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <array>

static double camYaw=30, camPitch=20, camDist=2.5;
static double lastX, lastY; static bool dragging=false;

static void mouseButtonCB(GLFWwindow* w,int btn,int action,int){
    if(btn==GLFW_MOUSE_BUTTON_LEFT){dragging=(action==GLFW_PRESS);glfwGetCursorPos(w,&lastX,&lastY);}
}
static void cursorCB(GLFWwindow*,double x,double y){
    if(!dragging)return;
    camYaw+=(x-lastX)*0.4; camPitch+=(y-lastY)*0.4;
    camPitch=std::max(-89.0,std::min(89.0,camPitch));
    lastX=x;lastY=y;
}
static void scrollCB(GLFWwindow*,double,double dy){
    camDist-=dy*0.15; camDist=std::max(1.2,std::min(8.0,camDist));
}

static void drawSphere(double r,int sl,int st){
    for(int i=0;i<st;i++){
        double lat0=M_PI*(-0.5+(double)i/st),lat1=M_PI*(-0.5+(double)(i+1)/st);
        double z0=sin(lat0),zr0=cos(lat0),z1=sin(lat1),zr1=cos(lat1);
        glBegin(GL_QUAD_STRIP);
        for(int j=0;j<=sl;j++){
            double lng=2*M_PI*(double)j/sl,x=cos(lng),y=sin(lng);
            glVertex3d(r*x*zr0,r*y*zr0,r*z0);
            glVertex3d(r*x*zr1,r*y*zr1,r*z1);
        }
        glEnd();
    }
}

static void drawCircle(double r,int seg){
    glBegin(GL_LINE_LOOP);
    for(int i=0;i<seg;i++){double a=2*M_PI*i/seg; glVertex3d(r*cos(a),r*sin(a),0);}
    glEnd();
}

static void applyCamera(){
    double yr=camYaw*M_PI/180,pr=camPitch*M_PI/180;
    double cx=camDist*cos(pr)*sin(yr),cy=camDist*sin(pr),cz=camDist*cos(pr)*cos(yr);
    double fx=-cx,fy=-cy,fz=-cz,fl=sqrt(fx*fx+fy*fy+fz*fz);
    fx/=fl;fy/=fl;fz/=fl;
    double sx=fy*0-fz*1,sy=fz*0-fx*0,sz=fx*1-fy*0,sl2=sqrt(sx*sx+sy*sy+sz*sz);
    sx/=sl2;sy/=sl2;sz/=sl2;
    double ux=sy*fz-sz*fy,uy=sz*fx-sx*fz,uz=sx*fy-sy*fx;
    double mv[16]={sx,ux,-fx,0,sy,uy,-fy,0,sz,uz,-fz,0,
        -(sx*cx+sy*cy+sz*cz),-(ux*cx+uy*cy+uz*cz),(fx*cx+fy*cy+fz*cz),1};
    glLoadMatrixd(mv);
}

void runGLVisualizer(std::vector<Satellite>& satellites,
                     double earthRadius, double angularSpeed)
{
    if(!glfwInit()) return;
    GLFWwindow* win=glfwCreateWindow(1000,800,"GNSS Satellite Visualizer",nullptr,nullptr);
    if(!win){glfwTerminate();return;}
    glfwMakeContextCurrent(win);
    glfwSetMouseButtonCallback(win,mouseButtonCB);
    glfwSetCursorPosCallback(win,cursorCB);
    glfwSetScrollCallback(win,scrollCB);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_POINT_SMOOTH);

    const double S=1.0/earthRadius;
    const double satR=26571000.0*S;
    std::vector<std::array<double,3>> trail;
    double simTime=0, prevWall=glfwGetTime();

    while(!glfwWindowShouldClose(win)){
        double now=glfwGetTime(), dt=now-prevWall; prevWall=now;
        simTime+=dt*200.0;

        double theta=angularSpeed*simTime;
        double rx=cos(theta),ry=sin(theta),rz=0.0;
        trail.push_back({rx,ry,rz});
        if(trail.size()>500) trail.erase(trail.begin());
        for(auto& sat:satellites) sat.update(simTime);

        int W,H; glfwGetFramebufferSize(win,&W,&H);
        glViewport(0,0,W,H);
        glClearColor(0.02f,0.02f,0.08f,1);
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

        glMatrixMode(GL_PROJECTION); glLoadIdentity();
        double asp=(double)W/H, f=1.0/tan(22.5*M_PI/180);
        double zN=0.01,zF=200;
        double pr[16]={f/asp,0,0,0,0,f,0,0,0,0,(zF+zN)/(zN-zF),-1,0,0,2*zF*zN/(zN-zF),0};
        glLoadMatrixd(pr);
        glMatrixMode(GL_MODELVIEW); glLoadIdentity();
        applyCamera();

        // Earth
        glColor3f(0.1f,0.25f,0.55f); drawSphere(1.0,36,18);
        glLineWidth(1); glColor3f(0.25f,0.25f,0.5f); drawCircle(1.001,64);

        // Orbital rings
        for(int p=0;p<6;p++){
            glPushMatrix(); glRotated(p*60,0,0,1); glRotated(55,1,0,0);
            glColor3f(0.15f,0.15f,0.15f); drawCircle(satR,64);
            glPopMatrix();
        }

        // Satellites + signal lines
        double recMag=sqrt(rx*rx+ry*ry+rz*rz);
        for(auto& sat:satellites){
            double sx=sat.getX()*S,sy=sat.getY()*S,sz=sat.getZ()*S;
            glPointSize(6); glColor3f(1,0.9f,0.2f);
            glBegin(GL_POINTS); glVertex3d(sx,sy,sz); glEnd();
            double dot=sx*rx+sy*ry+sz*rz,sm=sqrt(sx*sx+sy*sy+sz*sz);
            if(recMag>0&&dot/(sm*recMag)>0){
                glColor4f(0.2f,1,0.3f,0.18f);
                glBegin(GL_LINES); glVertex3d(sx,sy,sz); glVertex3d(rx,ry,rz); glEnd();
            }
        }

        // Receiver trail
        glLineWidth(2); glColor3f(0.8f,0.2f,0.2f);
        glBegin(GL_LINE_STRIP);
        for(auto& p:trail) glVertex3d(p[0],p[1],p[2]);
        glEnd();

        // Receiver dot
        glPointSize(10); glColor3f(1,0.2f,0.2f);
        glBegin(GL_POINTS); glVertex3d(rx,ry,rz); glEnd();

        glfwSwapBuffers(win); glfwPollEvents();
    }
    glfwDestroyWindow(win); glfwTerminate();
}
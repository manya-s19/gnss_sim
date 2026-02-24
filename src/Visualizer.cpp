#define GL_SILENCE_DEPRECATION
#include "Visualizer.h"
#include </opt/homebrew/include/GLFW/glfw3.h>
#include <OpenGL/gl.h>
#include <cmath>
#include <vector>
#include <array>
#include <string>

// ===== Camera =====
static double camYaw=30, camPitch=25, camDist=3.5;
static double lastX, lastY;
static bool dragging=false;

static void mouseButtonCB(GLFWwindow* w,int btn,int action,int){
    if(btn==GLFW_MOUSE_BUTTON_LEFT){ dragging=(action==GLFW_PRESS); glfwGetCursorPos(w,&lastX,&lastY); }
}
static void cursorCB(GLFWwindow*,double x,double y){
    if(!dragging) return;
    camYaw  +=(x-lastX)*0.4; camPitch+=(y-lastY)*0.4;
    camPitch=std::max(-89.0,std::min(89.0,camPitch));
    lastX=x; lastY=y;
}
static void scrollCB(GLFWwindow*,double,double dy){
    camDist-=dy*0.2; camDist=std::max(1.5,std::min(10.0,camDist));
}

// ===== Draw helpers =====
static void applyCamera(){
    double yr=camYaw*M_PI/180, pr=camPitch*M_PI/180;
    double cx=camDist*cos(pr)*sin(yr);
    double cy=camDist*sin(pr);
    double cz=camDist*cos(pr)*cos(yr);
    double fx=-cx,fy=-cy,fz=-cz;
    double fl=sqrt(fx*fx+fy*fy+fz*fz); fx/=fl;fy/=fl;fz/=fl;
    double sx=fy*0-fz*1,sy=fz*0-fx*0,sz=fx*1-fy*0;
    double sl=sqrt(sx*sx+sy*sy+sz*sz); sx/=sl;sy/=sl;sz/=sl;
    double ux=sy*fz-sz*fy,uy=sz*fx-sx*fz,uz=sx*fy-sy*fx;
    double mv[16]={sx,ux,-fx,0,sy,uy,-fy,0,sz,uz,-fz,0,
        -(sx*cx+sy*cy+sz*cz),-(ux*cx+uy*cy+uz*cz),(fx*cx+fy*cy+fz*cz),1};
    glLoadMatrixd(mv);
}

static void drawSphere(double r,int sl,int st){
    for(int i=0;i<st;i++){
        double lat0=M_PI*(-0.5+(double)i/st), lat1=M_PI*(-0.5+(double)(i+1)/st);
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

static void drawCircleXY(double r,int seg){
    glBegin(GL_LINE_LOOP);
    for(int i=0;i<seg;i++){
        double a=2*M_PI*i/seg;
        glVertex3d(r*cos(a),r*sin(a),0);
    }
    glEnd();
}

// Draw a circle in 3D around a center point (approximate no-fly zone ring)
static void drawNoFlyZone(double cx,double cy,double cz,double r,int seg){
    // Draw a circle in the plane perpendicular to the position vector
    double len=sqrt(cx*cx+cy*cy+cz*cz);
    double nx=cx/len,ny=cy/len,nz=cz/len;
    // Find two perpendicular vectors
    double ax=1,ay=0,az=0;
    if(fabs(nx)>0.9){ax=0;ay=1;}
    double bx=ny*az-nz*ay,by=nz*ax-nx*az,bz=nx*ay-ny*ax;
    double bl=sqrt(bx*bx+by*by+bz*bz); bx/=bl;by/=bl;bz/=bl;
    double ex=ny*bz-nz*by,ey=nz*bx-nx*bz,ez=nx*by-ny*bx;

    glBegin(GL_LINE_LOOP);
    for(int i=0;i<seg;i++){
        double a=2*M_PI*i/seg;
        double px=cx+r*(cos(a)*bx+sin(a)*ex);
        double py=cy+r*(cos(a)*by+sin(a)*ey);
        double pz=cz+r*(cos(a)*bz+sin(a)*ez);
        glVertex3d(px,py,pz);
    }
    glEnd();
}

void runVisualizer(std::vector<Satellite>& satellites,
                   const ScenarioState& scenario)
{
    if(!glfwInit()) return;
    GLFWwindow* win=glfwCreateWindow(1100,800,
        scenario.spoofMode ? "GNSS Simulator — SPOOFING MODE" : "GNSS Simulator — Normal Mode",
        nullptr,nullptr);
    if(!win){glfwTerminate();return;}
    glfwMakeContextCurrent(win);
    glfwSetMouseButtonCallback(win,mouseButtonCB);
    glfwSetCursorPosCallback(win,cursorCB);
    glfwSetScrollCallback(win,scrollCB);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);

    const double earthR = 6378137.0;
    const double S = 1.0/earthR;  // scale to unit sphere

    // Scale scenario data
    auto scale=[&](std::array<double,3> p)->std::array<double,3>{
        return {p[0]*S, p[1]*S, p[2]*S};
    };

    double simTime=360.0;
    double prevWall=glfwGetTime();
    int animStep=0;
    double stepTimer=0;

    while(!glfwWindowShouldClose(win)){
        double now=glfwGetTime();
        double dt=now-prevWall; prevWall=now;
        stepTimer+=dt;

        // Advance animation step every 1.5 seconds
        if(stepTimer>1.5 && animStep<(int)scenario.truePath.size()-1){
            animStep++;
            stepTimer=0;
            simTime+=360.0;
        }

        for(auto& sat:satellites) sat.update(simTime);

        int W,H; glfwGetFramebufferSize(win,&W,&H);
        glViewport(0,0,W,H);
        glClearColor(0.02f,0.02f,0.10f,1);
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

        // Projection
        glMatrixMode(GL_PROJECTION); glLoadIdentity();
        double asp=(double)W/H, f=1.0/tan(22.5*M_PI/180);
        double zN=0.001,zF=100;
        double pr[16]={f/asp,0,0,0,0,f,0,0,0,0,(zF+zN)/(zN-zF),-1,0,0,2*zF*zN/(zN-zF),0};
        glLoadMatrixd(pr);
        glMatrixMode(GL_MODELVIEW); glLoadIdentity();
        applyCamera();

        // Earth
        glColor3f(0.1f,0.25f,0.55f);
        drawSphere(1.0,48,24);

        // Equator
        glColor3f(0.25f,0.25f,0.5f);
        glLineWidth(1);
        drawCircleXY(1.001,64);

        // Satellite orbital rings
        glLineWidth(1);
        for(int p=0;p<6;p++){
            glPushMatrix();
            glRotated(p*60,0,0,1);
            glRotated(55,1,0,0);
            glColor3f(0.15f,0.15f,0.15f);
            drawCircleXY(26571000.0*S,64);
            glPopMatrix();
        }

        // Satellites
        double recPos[3]={0,0,0};
        if(!scenario.truePath.empty()){
            auto p=scale(scenario.truePath[animStep]);
            recPos[0]=p[0];recPos[1]=p[1];recPos[2]=p[2];
        }
        double recMag=sqrt(recPos[0]*recPos[0]+recPos[1]*recPos[1]+recPos[2]*recPos[2]);

        for(auto& sat:satellites){
            double sx=sat.getX()*S,sy=sat.getY()*S,sz=sat.getZ()*S;
            glPointSize(5);
            glColor3f(1,0.9f,0.2f);
            glBegin(GL_POINTS); glVertex3d(sx,sy,sz); glEnd();

            // Signal lines to current drone position
            double dot=sx*recPos[0]+sy*recPos[1]+sz*recPos[2];
            double sm=sqrt(sx*sx+sy*sy+sz*sz);
            if(recMag>0 && dot/(sm*recMag)>0){
                glColor4f(0.2f,1,0.3f,0.15f);
                glBegin(GL_LINES);
                glVertex3d(sx,sy,sz);
                glVertex3d(recPos[0],recPos[1],recPos[2]);
                glEnd();
            }
        }

        // No-fly zone ring — red
        auto pear=scale(scenario.pearsonPos);
        glColor3f(1,0.1f,0.1f);
        glLineWidth(2);
        drawNoFlyZone(pear[0],pear[1],pear[2], scenario.noFlyRadius*S, 64);

        // True path — white
        glLineWidth(2);
        glColor3f(0.9f,0.9f,0.9f);
        glBegin(GL_LINE_STRIP);
        for(int i=0;i<=animStep && i<(int)scenario.truePath.size();i++){
            auto p=scale(scenario.truePath[i]);
            glVertex3d(p[0],p[1],p[2]);
        }
        glEnd();

        // Estimated path — green if normal, orange if spoofed
        glLineWidth(2);
        glBegin(GL_LINE_STRIP);
        for(int i=0;i<=animStep && i<(int)scenario.estPath.size();i++){
            bool det = i<(int)scenario.spoofDetected.size() && scenario.spoofDetected[i];
            if(det) glColor3f(1,0.4f,0);      // orange = detected
            else    glColor3f(0.2f,1,0.4f);   // green  = clean
            auto p=scale(scenario.estPath[i]);
            glVertex3d(p[0],p[1],p[2]);
        }
        glEnd();

        // Current drone dot
        bool curDet = animStep<(int)scenario.spoofDetected.size() && scenario.spoofDetected[animStep];
        glPointSize(12);
        glColor3f(curDet ? 1.0f : 0.2f, curDet ? 0.2f : 1.0f, 0.2f);
        glBegin(GL_POINTS);
        glVertex3d(recPos[0],recPos[1],recPos[2]);
        glEnd();

        // Estimated drone dot
        if(animStep<(int)scenario.estPath.size()){
            auto ep=scale(scenario.estPath[animStep]);
            glPointSize(8);
            glColor3f(1,1,0.2f);
            glBegin(GL_POINTS); glVertex3d(ep[0],ep[1],ep[2]); glEnd();
        }

        glfwSwapBuffers(win);
        glfwPollEvents();
    }

    glfwDestroyWindow(win);
    glfwTerminate();
}
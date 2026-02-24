#define GL_SILENCE_DEPRECATION
#include "Visualizer.h"
#include </opt/homebrew/include/GLFW/glfw3.h>
#include <OpenGL/gl.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <array>

// Map lat/lon to local coords in [-1,1]
static std::array<double,2> toLocal(double lat, double lon) {
    const double cLat=43.6602, cLon=-79.5059, scale=3.5;
    return { -(lon-cLon)*scale, (lat-cLat)*scale };
}

static void drawCircle2D(double cx,double cy,double r,int seg,bool filled){
    glBegin(filled?GL_TRIANGLE_FAN:GL_LINE_LOOP);
    if(filled) glVertex2d(cx,cy);
    for(int i=0;i<=seg;i++){double a=2*M_PI*i/seg; glVertex2d(cx+r*cos(a),cy+r*sin(a));}
    glEnd();
}

// Draw one panel (left or right half of screen)
static void drawPanel(const ScenarioState& s, int animStep,
                      double xOffset, double panelW, double panelH,
                      bool isSpoof)
{
    // Precompute screen coords
    auto sc=[&](std::array<double,2> p){ return toLocal(p[0],p[1]); };

    auto pearsonL = sc(s.pearsonLatLon);

    // No-fly zone
    glColor4f(1,0.15f,0.15f,0.13f);
    drawCircle2D(pearsonL[0],pearsonL[1],s.noFlyRadiusDeg*3.5,64,true);
    glColor4f(1,0.15f,0.15f,1); glLineWidth(2.5f);
    drawCircle2D(pearsonL[0],pearsonL[1],s.noFlyRadiusDeg*3.5,64,false);

    // Pearson starburst
    glLineWidth(2); glColor4f(1,0.3f,0.3f,0.8f);
    for(int i=0;i<8;i++){
        double a=i*M_PI/4;
        glBegin(GL_LINES);
        glVertex2d(pearsonL[0]+0.04*cos(a),pearsonL[1]+0.04*sin(a));
        glVertex2d(pearsonL[0]+0.09*cos(a),pearsonL[1]+0.09*sin(a));
        glEnd();
    }

    // CN Tower diamond
    if(!s.truePath.empty()){
        auto p=sc(s.truePath[0]); double ds=0.06;
        glLineWidth(2); glColor4f(0.3f,1,0.4f,0.9f);
        glBegin(GL_LINE_LOOP);
        glVertex2d(p[0],p[1]+ds); glVertex2d(p[0]+ds,p[1]);
        glVertex2d(p[0],p[1]-ds); glVertex2d(p[0]-ds,p[1]);
        glEnd();
    }

    // True path — white
    glLineWidth(2.5f); glColor4f(0.9f,0.9f,0.9f,0.9f);
    glBegin(GL_LINE_STRIP);
    for(int i=0;i<=animStep&&i<(int)s.truePath.size();i++){
        auto p=sc(s.truePath[i]); glVertex2d(p[0],p[1]);
    }
    glEnd();
    // True waypoint dots
    for(int i=0;i<=animStep&&i<(int)s.truePath.size();i++){
        auto p=sc(s.truePath[i]);
        glPointSize(10); glColor3f(1,1,1);
        glBegin(GL_POINTS); glVertex2d(p[0],p[1]); glEnd();
    }

    if(isSpoof){
        // Spoof mode: estimated stuck at CN Tower
        auto ep=sc(s.estPath[0]);
        double pulse=0.6+0.4*sin(glfwGetTime()*4);

        // Line from fake pos to real current pos
        if(animStep>0){
            auto rp=sc(s.truePath[animStep]);
            glLineWidth(3); glColor4f(1,0.4f,0.1f,0.7f);
            glBegin(GL_LINES); glVertex2d(ep[0],ep[1]); glVertex2d(rp[0],rp[1]); glEnd();
        }
        // Pulsing orange X
        glLineWidth(5); glColor4f(1,0.3f*(float)pulse,0,1);
        double cs=0.08;
        glBegin(GL_LINES); glVertex2d(ep[0]-cs,ep[1]-cs); glVertex2d(ep[0]+cs,ep[1]+cs); glEnd();
        glBegin(GL_LINES); glVertex2d(ep[0]+cs,ep[1]-cs); glVertex2d(ep[0]-cs,ep[1]+cs); glEnd();
        // Pulsing ring
        glLineWidth(2); glColor4f(1,0.3f,0,(float)pulse*0.7f);
        drawCircle2D(ep[0],ep[1],0.13,32,false);
    } else {
        // Normal mode: green path follows true
        glLineWidth(2.5f);
        glBegin(GL_LINE_STRIP);
        for(int i=0;i<=animStep&&i<(int)s.estPath.size();i++){
            glColor4f(0.2f,1,0.4f,1); auto p=sc(s.estPath[i]);
            glVertex2d(p[0],p[1]-0.03);
        }
        glEnd();
        for(int i=0;i<=animStep&&i<(int)s.estPath.size();i++){
            auto p=sc(s.estPath[i]);
            glPointSize(8); glColor3f(0.2f,1,0.4f);
            glBegin(GL_POINTS); glVertex2d(p[0],p[1]-0.03); glEnd();
        }
    }

    // Current drone pulsing dot
    if(animStep<(int)s.truePath.size()){
        auto p=sc(s.truePath[animStep]);
        bool nofly=animStep<(int)s.inNoFly.size()&&s.inNoFly[animStep];
        double pulse=0.7+0.3*sin(glfwGetTime()*5);
        glPointSize(18*(float)pulse);
        glColor3f(nofly?1.0f:0.2f, nofly?0.2f:0.9f, 0.2f);
        glBegin(GL_POINTS); glVertex2d(p[0],p[1]); glEnd();
    }

    // Panel label
    glLineWidth(2);
    if(isSpoof){
        // Draw "SPOOFED" indicator — orange horizontal bar at top
        glColor4f(1,0.4f,0,0.8f);
        glBegin(GL_LINES); glVertex2d(-1.2,0.88); glVertex2d(1.2,0.88); glEnd();
    } else {
        // Green bar = normal
        glColor4f(0.2f,1,0.4f,0.6f);
        glBegin(GL_LINES); glVertex2d(-1.2,0.88); glVertex2d(1.2,0.88); glEnd();
    }
}

void runVisualizer(const ScenarioState& normal, const ScenarioState& spoofed)
{
    std::cout<<"Opening split-screen visualization...\n";
    if(!glfwInit()){ std::cout<<"GLFW init failed\n"; return; }

    GLFWwindow* win=glfwCreateWindow(1400,700,
        "GNSS Drone Simulator  |  LEFT: Normal Mode  |  RIGHT: Spoofing Mode",
        nullptr,nullptr);
    if(!win){ glfwTerminate(); return; }
    glfwMakeContextCurrent(win);
    glEnable(GL_BLEND);
    glEnable(GL_POINT_SMOOTH);
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);

    int animStep=0; double stepTimer=0, prevWall=glfwGetTime();
    int totalSteps=(int)normal.truePath.size();

    while(!glfwWindowShouldClose(win)){
        double now=glfwGetTime(), dt=now-prevWall; prevWall=now;
        stepTimer+=dt;
        if(stepTimer>1.2&&animStep<totalSteps-1){ animStep++; stepTimer=0; }

        int W,H; glfwGetFramebufferSize(win,&W,&H);
        glClearColor(0.07f,0.09f,0.13f,1); glClear(GL_COLOR_BUFFER_BIT);

        double asp=(double)W/H;

        // ===== LEFT PANEL: Normal Mode =====
        glViewport(0,0,W/2,H);
        glMatrixMode(GL_PROJECTION); glLoadIdentity();
        glOrtho(-asp/2,asp/2,-1,1,-1,1);
        glMatrixMode(GL_MODELVIEW); glLoadIdentity();

        // Grid
        glLineWidth(1); glColor4f(0.18f,0.2f,0.28f,1);
        for(int i=-20;i<=20;i++){
            glBegin(GL_LINES); glVertex2d(i*0.2,-2); glVertex2d(i*0.2,2); glEnd();
            glBegin(GL_LINES); glVertex2d(-2,i*0.2); glVertex2d(2,i*0.2); glEnd();
        }
        // Divider hint
        glLineWidth(1); glColor4f(0.4f,0.4f,0.4f,0.5f);
        glBegin(GL_LINES); glVertex2d(asp/2-0.01,-2); glVertex2d(asp/2-0.01,2); glEnd();

        drawPanel(normal, animStep, 0, W/2, H, false);

        // ===== RIGHT PANEL: Spoof Mode =====
        glViewport(W/2,0,W/2,H);
        glMatrixMode(GL_PROJECTION); glLoadIdentity();
        glOrtho(-asp/2,asp/2,-1,1,-1,1);
        glMatrixMode(GL_MODELVIEW); glLoadIdentity();

        // Grid
        glLineWidth(1); glColor4f(0.18f,0.2f,0.28f,1);
        for(int i=-20;i<=20;i++){
            glBegin(GL_LINES); glVertex2d(i*0.2,-2); glVertex2d(i*0.2,2); glEnd();
            glBegin(GL_LINES); glVertex2d(-2,i*0.2); glVertex2d(2,i*0.2); glEnd();
        }

        drawPanel(spoofed, animStep, W/2, W/2, H, true);

        glfwSwapBuffers(win);
        glfwPollEvents();
    }
    glfwDestroyWindow(win);
    glfwTerminate();
}
#include "Visualizer.h"

#include </opt/homebrew/include/GLFW/glfw3.h>
#include <OpenGL/gl.h>
#include <cmath>
#include <vector>

// ===== Camera state =====
static double camYaw   = 30.0;
static double camPitch = 20.0;
static double camDist  = 2.5;
static double lastX, lastY;
static bool dragging = false;

static void mouseButtonCallback(GLFWwindow* w, int button, int action, int) {
    if (button == GLFW_MOUSE_BUTTON_LEFT) {
        dragging = (action == GLFW_PRESS);
        glfwGetCursorPos(w, &lastX, &lastY);
    }
}

static void cursorPosCallback(GLFWwindow*, double x, double y) {
    if (!dragging) return;
    camYaw   += (x - lastX) * 0.4;
    camPitch += (y - lastY) * 0.4;
    camPitch = std::max(-89.0, std::min(89.0, camPitch));
    lastX = x; lastY = y;
}

static void scrollCallback(GLFWwindow*, double, double dy) {
    camDist -= dy * 0.15;
    camDist = std::max(1.2, std::min(8.0, camDist));
}

// ===== Draw helpers =====
static void drawSphere(double r, int slices, int stacks) {
    for (int i = 0; i < stacks; i++) {
        double lat0 = M_PI * (-0.5 + (double)i / stacks);
        double lat1 = M_PI * (-0.5 + (double)(i+1) / stacks);
        double z0 = sin(lat0), zr0 = cos(lat0);
        double z1 = sin(lat1), zr1 = cos(lat1);
        glBegin(GL_QUAD_STRIP);
        for (int j = 0; j <= slices; j++) {
            double lng = 2 * M_PI * (double)j / slices;
            double x = cos(lng), y = sin(lng);
            glNormal3d(x*zr0, y*zr0, z0);
            glVertex3d(r*x*zr0, r*y*zr0, r*z0);
            glNormal3d(x*zr1, y*zr1, z1);
            glVertex3d(r*x*zr1, r*y*zr1, r*z1);
        }
        glEnd();
    }
}

static void drawCircle(double r, int segments) {
    glBegin(GL_LINE_LOOP);
    for (int i = 0; i < segments; i++) {
        double a = 2 * M_PI * i / segments;
        glVertex3d(r * cos(a), r * sin(a), 0.0);
    }
    glEnd();
}

static void drawDot(double x, double y, double z, double size,
                    float r, float g, float b) {
    glPointSize(size);
    glColor3f(r, g, b);
    glBegin(GL_POINTS);
    glVertex3d(x, y, z);
    glEnd();
}

static void drawLine(double x0,double y0,double z0,
                     double x1,double y1,double z1,
                     float r,float g,float b,float alpha) {
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glColor4f(r, g, b, alpha);
    glBegin(GL_LINES);
    glVertex3d(x0,y0,z0);
    glVertex3d(x1,y1,z1);
    glEnd();
    glDisable(GL_BLEND);
}

// ===== Main entry =====
void runVisualizer(std::vector<Satellite>& satellites,
                   double earthRadius,
                   double angularSpeed)
{
    if (!glfwInit()) return;

    GLFWwindow* window = glfwCreateWindow(1000, 800, "GNSS Simulator", nullptr, nullptr);
    if (!window) { glfwTerminate(); return; }
    glfwMakeContextCurrent(window);

    glfwSetMouseButtonCallback(window, mouseButtonCallback);
    glfwSetCursorPosCallback(window,   cursorPosCallback);
    glfwSetScrollCallback(window,      scrollCallback);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_POINT_SMOOTH);

    // Scale everything so Earth radius = 1.0 in GL units
    const double S = 1.0 / earthRadius;
    const double satOrbitalRadius = 26571000.0 * S;

    // Receiver trail
    std::vector<std::array<double,3>> trail;

    double simTime = 0.0;
    double prevWall = glfwGetTime();

    while (!glfwWindowShouldClose(window)) {

        // ---- Timing ----
        double now = glfwGetTime();
        double dt  = now - prevWall;
        prevWall   = now;
        simTime   += dt * 200.0;   // sim runs 200x real time

        // ---- Receiver position ----
        double theta = angularSpeed * simTime;
        double rx = cos(theta);   // already scaled (radius = 1.0)
        double ry = sin(theta);
        double rz = 0.0;

        trail.push_back({rx, ry, rz});
        if (trail.size() > 500) trail.erase(trail.begin());

        // ---- Update satellites ----
        for (auto& sat : satellites)
            sat.update(simTime);

        // ---- Viewport / projection ----
        int W, H;
        glfwGetFramebufferSize(window, &W, &H);
        glViewport(0, 0, W, H);

        glClearColor(0.02f, 0.02f, 0.08f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        double aspect = (double)W / H;
        double fov = 45.0 * M_PI / 180.0;
        double zNear = 0.01, zFar = 200.0;
        double f = 1.0 / tan(fov / 2);
        double proj[16] = {
            f/aspect,0,0,0,
            0,f,0,0,
            0,0,(zFar+zNear)/(zNear-zFar),-1,
            0,0,2*zFar*zNear/(zNear-zFar),0
        };
        glLoadMatrixd(proj);

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        // Camera
        double yawR   = camYaw   * M_PI / 180.0;
        double pitchR = camPitch * M_PI / 180.0;
        double cx = camDist * cos(pitchR) * sin(yawR);
        double cy = camDist * sin(pitchR);
        double cz = camDist * cos(pitchR) * cos(yawR);

        // Simple lookat
        double fx = -cx, fy = -cy, fz = -cz;
        double flen = sqrt(fx*fx+fy*fy+fz*fz);
        fx/=flen; fy/=flen; fz/=flen;
        double ux=0,uy=1,uz=0;
        double sx2=fy*uz-fz*uy, sy2=fz*ux-fx*uz, sz2=fx*uy-fy*ux;
        double slen=sqrt(sx2*sx2+sy2*sy2+sz2*sz2);
        sx2/=slen;sy2/=slen;sz2/=slen;
        double ux2=sy2*fz-sz2*fy,uy2=sz2*fx-sx2*fz,uz2=sx2*fy-sy2*fx;
        double mv[16]={sx2,ux2,-fx,0, sy2,uy2,-fy,0, sz2,uz2,-fz,0,
                       -(sx2*cx+sy2*cy+sz2*cz),-(ux2*cx+uy2*cy+uz2*cz),(fx*cx+fy*cy+fz*cz),1};
        glLoadMatrixd(mv);

        // ---- Draw Earth ----
        glColor3f(0.1f, 0.3f, 0.6f);
        drawSphere(1.0, 36, 18);

        // Equator ring
        glLineWidth(1.0f);
        glColor3f(0.3f, 0.3f, 0.6f);
        drawCircle(1.001, 64);

        // ---- Draw satellite orbital rings (faint) ----
        glLineWidth(1.0f);
        glColor3f(0.2f, 0.2f, 0.2f);
        int numPlanes = satellites.size() / 4;
        for (int p = 0; p < numPlanes; p++) {
            glPushMatrix();
            double planeRot = p * (360.0 / numPlanes);
            glRotated(planeRot, 0, 0, 1);
            glRotated(55.0, 1, 0, 0);  // inclination
            drawCircle(satOrbitalRadius, 64);
            glPopMatrix();
        }

        // ---- Draw satellites + signal lines ----
        double recMag = sqrt(rx*rx + ry*ry + rz*rz);

        for (auto& sat : satellites) {
            double sx = sat.getX() * S;
            double sy = sat.getY() * S;
            double sz = sat.getZ() * S;

            // Satellite dot — yellow
            drawDot(sx, sy, sz, 6.0f, 1.0f, 0.9f, 0.2f);

            // Visibility check
            double dot = sx*rx + sy*ry + sz*rz;
            double satMag = sqrt(sx*sx + sy*sy + sz*sz);
            if (dot / (satMag * recMag) > 0.0) {
                // Signal line — green, semi-transparent
                drawLine(sx, sy, sz, rx, ry, rz, 0.2f, 1.0f, 0.3f, 0.25f);
            }
        }

        // ---- Draw receiver trail ----
        glLineWidth(2.0f);
        glColor3f(0.8f, 0.2f, 0.2f);
        glBegin(GL_LINE_STRIP);
        for (auto& p : trail)
            glVertex3d(p[0], p[1], p[2]);
        glEnd();

        // ---- Draw receiver dot — red ----
        drawDot(rx, ry, rz, 10.0f, 1.0f, 0.2f, 0.2f);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwDestroyWindow(window);
    glfwTerminate();
}
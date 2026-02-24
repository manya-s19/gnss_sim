#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>
#include <string>
#include <array>

#include "Satellite.h"
#include "Receiver.h"
#include "Spoofer.h"
#include "Detector.h"
#include "Visualizer.h"

std::array<double,3> lla_to_ecef(double lat_deg,double lon_deg,double alt_m){
    const double a=6378137.0, e2=0.00669437999014;
    double lat=lat_deg*M_PI/180, lon=lon_deg*M_PI/180;
    double N=a/sqrt(1-e2*sin(lat)*sin(lat));
    return {(N+alt_m)*cos(lat)*cos(lon),(N+alt_m)*cos(lat)*sin(lon),(N*(1-e2)+alt_m)*sin(lat)};
}

std::vector<double> solvePositionLeastSquares(
    const std::vector<std::vector<double>>& satPositions,
    const std::vector<double>& pseudoranges,
    double initX,double initY,double initZ)
{
    double x=initX,y=initY,z=initZ,clockBias=0;
    for(int iter=0;iter<20;iter++){
        double HtH[4][4]={0},Htr[4]={0};
        for(int i=0;i<(int)satPositions.size();i++){
            double dx=x-satPositions[i][0],dy=y-satPositions[i][1],dz=z-satPositions[i][2];
            double range=sqrt(dx*dx+dy*dy+dz*dz);
            if(range<1) continue;
            double res=pseudoranges[i]-(range+clockBias);
            double H[4]={dx/range,dy/range,dz/range,1};
            for(int r=0;r<4;r++){Htr[r]+=H[r]*res;for(int c=0;c<4;c++)HtH[r][c]+=H[r]*H[c];}
        }
        double A[4][5];
        for(int i=0;i<4;i++){for(int j=0;j<4;j++)A[i][j]=HtH[i][j];A[i][4]=Htr[i];}
        for(int i=0;i<4;i++){
            int mr=i;double mv=fabs(A[i][i]);
            for(int k=i+1;k<4;k++)if(fabs(A[k][i])>mv){mv=fabs(A[k][i]);mr=k;}
            if(mr!=i)for(int j=0;j<5;j++)std::swap(A[i][j],A[mr][j]);
            double piv=A[i][i];if(fabs(piv)<1e-10)continue;
            for(int j=i;j<5;j++)A[i][j]/=piv;
            for(int k=0;k<4;k++){if(k==i)continue;double f=A[k][i];for(int j=i;j<5;j++)A[k][j]-=f*A[i][j];}
        }
        double du=A[0][4],dv=A[1][4],dw=A[2][4],db=A[3][4];
        const double ms=1e5;
        x+=std::max(-ms,std::min(ms,du)); y+=std::max(-ms,std::min(ms,dv));
        z+=std::max(-ms,std::min(ms,dw)); clockBias+=db;
        if(fabs(du)<1e-4&&fabs(dv)<1e-4&&fabs(dw)<1e-4)break;
    }
    return {x,y,z,clockBias};
}

ScenarioState runScenario(bool spoofMode) {
    std::vector<std::array<double,3>> waypoints={
        lla_to_ecef(43.6426,-79.3871,200),
        lla_to_ecef(43.6500,-79.4500,200),
        lla_to_ecef(43.6600,-79.5200,200),
        lla_to_ecef(43.6700,-79.5800,200),
        lla_to_ecef(43.6777,-79.6248,200),
    };
    std::vector<std::array<double,2>> waypoints_ll = {
        {43.6426,-79.3871},{43.6500,-79.4500},{43.6600,-79.5200},
        {43.6700,-79.5800},{43.6777,-79.6248},
    };

    auto pearson=lla_to_ecef(43.6777,-79.6248,173);
    const double noFlyRadius=5000.0;
    auto fake=lla_to_ecef(43.6426,-79.3871,200);
    Spoofer spoofer(fake[0],fake[1],fake[2]);

    std::vector<Satellite> satellites;
    const double incl=55.0*M_PI/180;
    for(int p=0;p<6;p++) for(int s=0;s<4;s++)
        satellites.emplace_back(26571000.0,(s*(2*M_PI/4))+(p*(2*M_PI/6)),incl);

    Detector detector;
    const double clockBiasTrue=0.12;

    ScenarioState scenario;
    scenario.pearsonLatLon={43.6777,-79.6248};
    scenario.noFlyRadiusDeg=2000.0/111000.0;
    scenario.spoofMode=spoofMode;

    double simTime=0;
    for(int wp=0;wp<(int)waypoints.size();wp++){
        auto& pos=waypoints[wp];
        double rx=pos[0],ry=pos[1],rz=pos[2];
        simTime+=360;
        for(auto& sat:satellites) sat.update(simTime);

        Receiver trueReceiver(rx,ry,rz);
        std::vector<double> pseudoranges;
        std::vector<std::vector<double>> satPositions;
        double recMag=sqrt(rx*rx+ry*ry+rz*rz);
        for(auto& sat:satellites){
            double sx=sat.getX(),sy=sat.getY(),sz=sat.getZ();
            if((sx*rx+sy*ry+sz*rz)/(sqrt(sx*sx+sy*sy+sz*sz)*recMag)>0){
                pseudoranges.push_back(trueReceiver.distanceTo(sx,sy,sz)+clockBiasTrue);
                satPositions.push_back({sx,sy,sz});
            }
        }
        if((int)satPositions.size()<4) continue;
        if(spoofMode)
            pseudoranges=spoofer.spoofPseudoranges(satPositions,pseudoranges,clockBiasTrue);

        auto sol=solvePositionLeastSquares(satPositions,pseudoranges,rx,ry,rz);
        double estX=sol[0],estY=sol[1],estZ=sol[2],clockBias=sol[3];
        double dx=rx-pearson[0],dy2=ry-pearson[1],dz=rz-pearson[2];
        bool inNoFly=sqrt(dx*dx+dy2*dy2+dz*dz)<noFlyRadius;

        DetectionResult det=detector.analyze(estX,estY,estZ,clockBias,360,satPositions,pseudoranges);

        scenario.truePath.push_back({waypoints_ll[wp][0],waypoints_ll[wp][1]});
        if(spoofMode)
            scenario.estPath.push_back({43.6426,-79.3871}); // stuck at CN Tower
        else
            scenario.estPath.push_back({waypoints_ll[wp][0],waypoints_ll[wp][1]});
        scenario.spoofDetected.push_back(det.spoofingDetected);
        scenario.inNoFly.push_back(inNoFly);
    }
    return scenario;
}

int main(int argc,char* argv[]){
    std::cout<<"\n==========================================\n";
    std::cout<<"  GNSS DRONE NO-FLY ZONE SIMULATOR\n";
    std::cout<<"  Split-screen: Normal vs Spoofing\n";
    std::cout<<"==========================================\n\n";

    ScenarioState normal = runScenario(false);
    ScenarioState spoofed = runScenario(true);

    // Visualization is handled by src/visualize.py
    // Run: python src/visualize.py
    return 0;
}
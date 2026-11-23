// Odtworzenie offline: URDF i pięć wierszy q/lower/upper/vmax/amax, wszystko SI.
#include "robot_kinematics/tcp_jog.hpp"
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <stdexcept>
using namespace robot_kinematics;
int main(int argc,char **argv) {
  try {
    if(argc!=4)throw std::invalid_argument("Użycie: tcp_jog_replay model.urdf limits.txt samples.csv");
    std::ifstream xml_file(argv[1]);const std::string xml{std::istreambuf_iterator<char>(xml_file),{}};
    std::ifstream input(argv[2]);Joints start,lower,upper,vmax,amax;
    for(auto array:{&start,&lower,&upper,&vmax,&amax})for(auto &v:*array)if(!(input>>v))throw std::runtime_error("Niepełne dane startu/limitów");
    Model model(xml,lower,upper);TcpJog jog(model,start,vmax,amax,{0,1,TcpFrame::base,.002,.1,.01,.085,1e-6,1e-5,.5});
    std::ofstream csv(argv[3]);csv<<std::setprecision(17)<<"t,s,v,a,jerk,ik_wall_ms,fk_error_m";
    for(unsigned j=0;j<6;++j)csv<<",q"<<j+1;csv<<'\n';
    double maximum=0,max_position_error=0,max_acceleration=0,max_jerk=0;unsigned ramp_samples=0;
    TcpJogSample s;
    for(unsigned k=1;k<=10000;++k) {
      const auto begin=std::chrono::steady_clock::now();
      s=jog.next(false,[&]{return std::chrono::steady_clock::now()-begin>=std::chrono::milliseconds(7);});
      const double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();maximum=std::max(maximum,ms);
      max_position_error=std::max(max_position_error,(model.fk(s.positions).translation()-jog.target(s.progress).translation()).norm());
      max_acceleration=std::max(max_acceleration,std::abs(s.scalar_acceleration));max_jerk=std::max(max_jerk,std::abs(s.scalar_jerk));
      if(s.progress>0 && s.speed<.002-1e-10 && !s.braking)++ramp_samples;
      csv<<k*.01<<','<<s.progress<<','<<s.speed<<','<<s.scalar_acceleration<<','<<s.scalar_jerk<<','<<ms<<','<<max_position_error;
      for(double q:s.positions)csv<<','<<q;csv<<'\n';
      if(s.done) {std::cout<<std::setprecision(12)<<"samples="<<k<<" progress_m="<<s.progress<<" end_error_m="<<std::abs(s.progress-.01)
        <<" max_ik_wall_ms="<<maximum<<" max_fk_error_m="<<max_position_error<<" max_acceleration="<<max_acceleration
        <<" max_jerk="<<max_jerk<<" acceleration_ramp_samples="<<ramp_samples<<" reason="<<jog.stop_reason()<<std::endl;break;}
    }
    if(!s.done || std::abs(s.progress-.01)>=1e-12 || s.speed!=0 || s.scalar_acceleration!=0 || maximum>=7)
      throw std::runtime_error("Odtworzenie nie spełnia kryteriów końca lub budżetu 7 ms");
    return 0;
  }catch(const std::exception &e){std::cerr<<e.what()<<std::endl;return 1;}
}

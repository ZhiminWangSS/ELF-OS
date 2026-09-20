// Standalone SDK process: never load ROS DDS libraries into this executable.
#include "nav_policy.hpp"
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include "remote_data.hpp"
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <chrono>
#include <csignal>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <mutex>
#include <sstream>
#include <thread>
using State=unitree_go::msg::dds_::SportModeState_;
using Remote=RemoteData;
using RemoteState=unitree_go::msg::dds_::LowState_;
using Clock=std::chrono::steady_clock;
volatile sig_atomic_t interrupted=0;
void signal_handler(int){interrupted=1;}
double seconds(){return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();}
struct Packet { unsigned long long sequence=0;double stamp=0,vx=0,wz=0;int healthy=0; };
bool parse(const char*data,Packet &p){
 std::istringstream in(data);std::string extra;
 return bool(in>>p.sequence>>p.stamp>>p.vx>>p.wz>>p.healthy)&&!(in>>extra)&&std::isfinite(p.stamp)&&std::isfinite(p.vx)&&std::isfinite(p.wz)&&p.vx>=0&&p.vx<=.20&&std::abs(p.wz)<=.35&&(p.healthy==0||p.healthy==1);
}
int main(int argc,char**argv){
 if(argc==3&&(std::string(argv[2])=="--inspect"||std::string(argv[2])=="--watch-remote")) {
   try {
     unitree::robot::ChannelFactory::Instance()->Init(0,argv[1]);
     std::mutex mutex;State state{};Remote remote{};double state_at=0,remote_at=0;
     unitree::robot::ChannelSubscriber<State> ss("rt/sportmodestate");
     unitree::robot::ChannelSubscriber<RemoteState> rs("rt/lowstate");
     ss.InitChannel([&](const void*p){std::lock_guard<std::mutex>g(mutex);state=*static_cast<const State*>(p);state_at=seconds();},1);
     rs.InitChannel([&](const void*p){std::lock_guard<std::mutex>g(mutex);remote=Remote::decode(static_cast<const RemoteState*>(p)->wireless_remote());remote_at=seconds();},1);
     if(std::string(argv[2])=="--watch-remote") {
       unsigned last=~0u;bool was_active=false;
       for(int i=0;i<1200;++i){
         {std::lock_guard<std::mutex>g(mutex);const char* reason=elf_nav::remote_rejection(seconds()-remote_at,remote.keys(),{remote.lx(),remote.ly(),remote.rx(),remote.ry()});
         bool active=reason&&std::string(reason)=="remote_override";
         if(remote.keys()!=last||active!=was_active||i%20==0){
           std::cout<<"{\"read_only\":true,\"keys\":"<<remote.keys()<<",\"active\":"<<(active?"true":"false")<<",\"state_age_s\":"<<seconds()-remote_at<<"}"<<std::endl;
           last=remote.keys();was_active=active;
         }}
         std::this_thread::sleep_for(std::chrono::milliseconds(50));
       }
       return 0;
     }
     std::this_thread::sleep_for(std::chrono::seconds(2));
     std::lock_guard<std::mutex>g(mutex);
     std::cout << "{\"read_only\":true,\"state_age_s\":" << seconds()-state_at
       << ",\"remote_age_s\":" << seconds()-remote_at << ",\"error_code\":" << state.error_code()
       << ",\"roll\":" << state.imu_state().rpy()[0] << ",\"pitch\":" << state.imu_state().rpy()[1]
       << ",\"mode\":" << unsigned(state.mode()) << ",\"remote_keys\":" << remote.keys()
       << ",\"remote_axes\":[" << remote.lx() << ',' << remote.ly() << ',' << remote.rx() << ',' << remote.ry() << "]}" << std::endl;
     return 0;
   } catch(const std::exception&e){std::cerr<<e.what()<<std::endl;return 1;}
 }
 bool execute=argc==3&&std::string(argv[2])=="--execute";
 bool mock=argc==3&&std::string(argv[2])=="--mock";
 if(!execute&&!mock){std::cerr<<"usage: nav_worker eth0 --mock|--execute\n";return 2;}
 std::signal(SIGINT,signal_handler);std::signal(SIGTERM,signal_handler);
 std::string dir=(mock?"/tmp/elf-nav-mock-":"/tmp/elf-go2-")+std::to_string(getuid());
 if(mkdir(dir.c_str(),0700)!=0&&errno!=EEXIST)return 2;
 struct stat st{};if(lstat(dir.c_str(),&st)!=0||!S_ISDIR(st.st_mode)||st.st_uid!=getuid()||(st.st_mode&077)!=0)return 2;
 int lock=open((dir+"/motion.lock").c_str(),O_CREAT|O_RDWR|O_NOFOLLOW,0600);
 if(lock<0||flock(lock,LOCK_EX|LOCK_NB)!=0){std::cerr<<"Another ELF motion owner is active\n";return 2;}
 std::string cancel=dir+"/cancel",path=dir+"/nav.sock";
 std::filesystem::remove(cancel);unlink(path.c_str());
 int socket_fd=socket(AF_UNIX,SOCK_DGRAM|SOCK_NONBLOCK,0);sockaddr_un address{};address.sun_family=AF_UNIX;
 if(path.size()>=sizeof(address.sun_path))return 2;std::strcpy(address.sun_path,path.c_str());
 if(bind(socket_fd,reinterpret_cast<sockaddr*>(&address),sizeof(address))!=0)return 2;
 chmod(path.c_str(),0600);
 std::mutex mutex;State state{};Remote remote{};double state_time=0,remote_time=0;
 unitree::robot::ChannelSubscriberPtr<State> state_sub;
 unitree::robot::ChannelSubscriberPtr<RemoteState> remote_sub;
 std::unique_ptr<unitree::robot::go2::SportClient> client;
 bool moving=false,latched=false;int exit_code=0;double last_log=0,last_stop=0,last_warning=0,last_move=0;std::string latch_reason;
 auto stop=[&](){int rc=0;if(client){try{rc=client->StopMove();}catch(...){rc=-1;}}moving=false;last_stop=seconds();std::cout<<"{\"event\":\"stop\",\"mock\":"<<(mock?"true":"false")<<",\"code\":"<<rc<<"}"<<std::endl;if(rc!=0)exit_code=1;return rc;};
 try{
   if(execute){
     unitree::robot::ChannelFactory::Instance()->Init(0,argv[1]);
     state_sub.reset(new unitree::robot::ChannelSubscriber<State>("rt/sportmodestate"));
     state_sub->InitChannel([&](const void*m){std::lock_guard<std::mutex>g(mutex);state=*static_cast<const State*>(m);state_time=seconds();},1);
     remote_sub.reset(new unitree::robot::ChannelSubscriber<RemoteState>("rt/lowstate"));
     remote_sub->InitChannel([&](const void*m){std::lock_guard<std::mutex>g(mutex);remote=Remote::decode(static_cast<const RemoteState*>(m)->wireless_remote());remote_time=seconds();},1);
     client.reset(new unitree::robot::go2::SportClient);client->SetTimeout(.1f);client->Init();
     if(stop()!=0)throw std::runtime_error("initial StopMove failed");
   }
   Packet current;unsigned long long last_sequence=0;double received=0;
   while(!interrupted){
     char data[256];ssize_t n;bool malformed=false,new_command=false;
     while((n=recv(socket_fd,data,sizeof(data)-1,MSG_DONTWAIT))>0){
       data[n]=0;Packet p;if(!parse(data,p)||p.sequence<=last_sequence){malformed=true;continue;}
       last_sequence=p.sequence;current=p;received=seconds();new_command=true;
     }
     const double now=seconds();std::string rejection;
     if(malformed){latched=true;rejection="malformed_or_replayed_command";if(latch_reason.empty())latch_reason=rejection;}
     if(std::filesystem::exists(cancel)){latched=true;rejection="operator_stop";if(latch_reason.empty())latch_reason=rejection;}
     if(!current.healthy||now-current.stamp<0||now-current.stamp>.25||now-received>.25)rejection="upstream_stale_or_unhealthy";
     if(execute){
       std::lock_guard<std::mutex>g(mutex);const double state_check_now=seconds();auto rpy=state.imu_state().rpy();
       if(const char* reason=elf_nav::state_rejection(state_check_now-state_time,state.error_code(),state.mode(),rpy[0],rpy[1],moving))rejection=reason;
       if(state.error_code()==2&&now-last_warning>1){
         std::cout<<"{\"event\":\"guard_warning\",\"reason\":\"operator_allowed_code_2\",\"robot_error_code\":2,\"mode\":"<<unsigned(state.mode())<<"}"<<std::endl;
         last_warning=now;
       }
       if(const char* reason=elf_nav::remote_rejection(state_check_now-remote_time,remote.keys(),{remote.lx(),remote.ly(),remote.rx(),remote.ry()})) {
         rejection=reason;if(rejection=="remote_override")latched=true;
       }
     }
     // Yield ownership after the first stop; repeated StopMove would fight manual control.
     if(rejection=="remote_override"||rejection=="operator_stop"){stop();break;}
     if(!rejection.empty()&&moving){latched=true;if(latch_reason.empty())latch_reason=rejection;}
     // A zero Twist is a normal Nav2 command (for example while waiting for
     // the next controller cycle).  StopMove changes the firmware mode and
     // can reject the next valid command, so reserve it for guard conditions
     // and shutdown.  Send a zero Move through the same motion interface.
     if(latched||!rejection.empty()){
       if(moving||now-last_stop>.2)stop();
       if(now-last_log>1){
         std::cout<<"{\"event\":\"guard\",\"latched\":"<<(latched?"true":"false")
           <<",\"reason\":\""<<(latched&&!latch_reason.empty()?latch_reason:rejection)<<"\",\"robot_mode\":"<<unsigned(state.mode())
           <<",\"robot_error_code\":"<<state.error_code()<<",\"state_age_s\":"<<now-state_time
           <<",\"remote_age_s\":"<<now-remote_time<<"}"<<std::endl;last_log=now;
       }
    }else if(new_command&&now-last_move>=.1){
      int rc=client?client->Move(current.vx,0,current.wz):0;
       if(rc!=0){latched=true;exit_code=1;stop();}
       else{moving=true;last_move=now;std::cout<<"{\"event\":\"move\",\"mock\":"<<(mock?"true":"false")<<",\"vx\":"<<current.vx<<",\"yaw_rate\":"<<current.wz<<"}"<<std::endl;}
     }
     std::this_thread::sleep_for(std::chrono::milliseconds(20));
   }
 }catch(const std::exception&e){std::cerr<<e.what()<<std::endl;exit_code=1;}
 for(int i=0;i<3;++i){if(stop()==0)break;}
 remote_sub.reset();state_sub.reset();close(socket_fd);unlink(path.c_str());flock(lock,LOCK_UN);close(lock);return exit_code;
}

// Adapted from pingandog/src/go2_comm_tests (image, state and high-level motion).
// No ROS dependency and no joint-control or posture-changing commands.
#include "guard.hpp"
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/go2/obstacles_avoid/obstacles_avoid_client.hpp>
#include <unitree/robot/go2/video/video_client.hpp>
#include <chrono>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>
#include <fcntl.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>

using Clock = std::chrono::steady_clock;
using State = unitree_go::msg::dds_::SportModeState_;
volatile std::sig_atomic_t interrupted = 0;
void on_signal(int) { interrupted = 1; }
double elapsed(Clock::time_point t) { return std::chrono::duration<double>(Clock::now()-t).count(); }

struct Sample { State state; double age; };
class StateReader {
  std::mutex mutex_;
  State state_{};
  Clock::time_point received_{};
  unitree::robot::ChannelSubscriberPtr<State> sub_;
public:
  StateReader() {
    sub_.reset(new unitree::robot::ChannelSubscriber<State>("rt/sportmodestate"));
    sub_->InitChannel([this](const void* msg) {
      std::lock_guard<std::mutex> guard(mutex_);
      state_ = *static_cast<const State*>(msg); received_ = Clock::now();
    }, 1);
  }
  // Destroy the subscriber before the mutex/state used by its callback.
  Sample sample() { std::lock_guard<std::mutex> guard(mutex_); return {state_, elapsed(received_)}; }
  Sample wait() {
    auto begin = Clock::now();
    while (elapsed(begin) < 3 && !interrupted) {
      auto s = sample(); if (s.age <= .5) return s;
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    throw std::runtime_error("no fresh sport state within 3 seconds");
  }
};
bool check_state(const Sample& s, bool starting, const char* phase) {
  const auto& r = s.state.imu_state().rpy();
  const char* reason = elf::rejection(s.age, s.state.error_code(), s.state.mode(), r[0], r[1], starting);
  if (!reason) {
    if (s.state.error_code() == 2)
      std::cout << "{\"event\":\"guard_warning\",\"phase\":\"" << phase
        << "\",\"reason\":\"operator_allowed_code_2\",\"robot_error_code\":2,\"mode\":"
        << unsigned(s.state.mode()) << "}" << std::endl;
    return true;
  }
  std::cout << "{\"event\":\"guard_rejected\",\"phase\":\"" << phase
    << "\",\"reason\":\"" << reason << "\",\"robot_error_code\":" << s.state.error_code()
    << ",\"mode\":" << unsigned(s.state.mode()) << "}" << std::endl;
  std::cerr << "guard rejected (" << phase << "): " << reason
    << "; SportModeState.error_code=" << s.state.error_code()
    << "; mode=" << unsigned(s.state.mode()) << "; age_s=" << s.age
    << "; roll=" << r[0] << "; pitch=" << r[1] << std::endl;
  return false;
}
void print_state(const Sample& s) {
  const auto& p=s.state.position(); const auto& v=s.state.velocity();
  const auto& r=s.state.imu_state().rpy();
  // SDK state should contain finite numbers; reject malformed samples before JSON.
  for (auto a : {p[0],p[1],p[2],v[0],v[1],v[2],r[0],r[1],r[2]})
    if (!std::isfinite(a)) throw std::runtime_error("nonfinite state");
  std::cout << "{\"event\":\"state\",\"age_s\":" << s.age
    << ",\"mode\":" << unsigned(s.state.mode()) << ",\"error_code\":" << s.state.error_code()
    << ",\"position\":[" << p[0] << ',' << p[1] << ',' << p[2]
    << "],\"velocity\":[" << v[0] << ',' << v[1] << ',' << v[2]
    << "],\"rpy\":[" << r[0] << ',' << r[1] << ',' << r[2] << "]}" << std::endl;
}

// Serialize our own step processes. stop bypasses the lock and leaves a cancel
// marker that the active step checks before each velocity call.
struct MotionLock {
  int fd = -1;
  explicit MotionLock(const std::filesystem::path& p) {
    fd = open(p.c_str(), O_CREAT|O_RDWR|O_NOFOLLOW, 0600);
    if (fd < 0) throw std::runtime_error("cannot open motion lock");
    if (flock(fd, LOCK_EX|LOCK_NB) != 0) { close(fd); fd=-1; throw std::runtime_error("another ELF step is active"); }
  }
  ~MotionLock() { if (fd>=0) { flock(fd, LOCK_UN); close(fd); } }
};
int stop(unitree::robot::go2::SportClient& client) {
  int rc = -1;
  for (int n=0; n<3; ++n) {
    try { rc=client.StopMove(); } catch (...) { rc=-1; }
    std::cout << "{\"event\":\"stop\",\"code\":" << rc << "}" << std::endl;
    if (rc==0) return 0;
  }
  return rc;
}
double number(const char* arg) {
  std::size_t n=0; double v=std::stod(arg,&n);
  if (n!=std::string(arg).size()) throw std::runtime_error("invalid numeric argument");
  return v;
}
int run(int argc, char** argv) {
  if (argc<3) throw std::runtime_error("usage: elf_go2 IFACE state|image PATH|stop|step VX YAW_RATE SECONDS --execute");
  std::string iface=argv[1], action=argv[2];
  elf::Step spec{};
  if (action=="step") {
    if (argc!=7 || std::string(argv[6])!="--execute") throw std::runtime_error("step requires explicit --execute");
    spec={number(argv[3]),number(argv[4]),number(argv[5])}; elf::validate(spec);
  } else if (action=="image") { if (argc!=4) throw std::runtime_error("image requires output path"); }
  else if ((action!="state" && action!="stop") || argc!=3) throw std::runtime_error("unknown command or extra arguments");

  std::filesystem::path cancel;
  std::unique_ptr<MotionLock> lock;
  if (action=="step" || action=="stop") {
    auto dir=std::filesystem::path("/tmp")/("elf-go2-"+std::to_string(getuid()));
    if (mkdir(dir.c_str(),0700)!=0 && errno!=EEXIST) throw std::runtime_error("cannot create runtime directory");
    struct stat st{};
    if (lstat(dir.c_str(),&st)!=0 || !S_ISDIR(st.st_mode) || st.st_uid!=getuid() || (st.st_mode&077)!=0)
      throw std::runtime_error("unsafe runtime directory");
    cancel=dir/"cancel";
    if (action=="step") { lock.reset(new MotionLock(dir/"motion.lock")); std::filesystem::remove(cancel); }
    else { std::ofstream marker(cancel); marker << "stop\n"; if (!marker) throw std::runtime_error("cannot signal cancellation"); }
  }
  unitree::robot::ChannelFactory::Instance()->Init(0,iface);
  if (action=="image") {
    unitree::robot::go2::VideoClient client; client.SetTimeout(3.0f); client.Init();
    std::vector<uint8_t> data; int rc=client.GetImageSample(data);
    if (rc!=0 || data.empty()) throw std::runtime_error("image RPC failed: "+std::to_string(rc));
    auto p=std::filesystem::absolute(argv[3]); std::filesystem::create_directories(p.parent_path());
    std::ofstream file(p,std::ios::binary); file.write(reinterpret_cast<const char*>(data.data()),data.size()); file.close();
    if (!file) throw std::runtime_error("image write failed");
    std::cout << "{\"event\":\"image\",\"bytes\":" << data.size() << "}" << std::endl;
    return 0;
  }
  if (action=="state") { StateReader reader; print_state(reader.wait()); return 0; }
  unitree::robot::go2::SportClient client; client.SetTimeout(.3f); client.Init();
  if (action=="stop") return stop(client)==0 ? 0 : 1;
  unitree::robot::go2::ObstaclesAvoidClient obstacle_client;
  obstacle_client.SetTimeout(.3f); obstacle_client.Init();
  bool avoidance_enabled = false;
  int avoidance_rc = obstacle_client.SwitchGet(avoidance_enabled);
  if (avoidance_rc != 0) throw std::runtime_error("obstacle avoidance status query failed");
  if (!avoidance_enabled) {
    avoidance_rc = obstacle_client.SwitchSet(true);
    if (avoidance_rc != 0 || obstacle_client.SwitchGet(avoidance_enabled) != 0 || !avoidance_enabled)
      throw std::runtime_error("unable to enable obstacle avoidance");
  }
  if (obstacle_client.UseRemoteCommandFromApi(true) != 0)
    throw std::runtime_error("unable to acquire obstacle avoidance command source");
  StateReader reader;
  bool ok=false;
  try {
    auto initial=reader.wait(); print_state(initial);
    if (!check_state(initial,true,"before_move")) throw std::runtime_error("step blocked by state guard");
    if (stop(client)!=0) throw std::runtime_error("stop preflight failed");
    auto begin=Clock::now();
    while (elapsed(begin)<spec.seconds) {
      if (interrupted || std::filesystem::exists(cancel)) throw std::runtime_error("step cancelled");
      auto s=reader.sample(); print_state(s);
      if (!check_state(s,false,"during_move")) throw std::runtime_error("step stopped by state guard");
      int rc=obstacle_client.Move(spec.vx,0,spec.yaw_rate);
      std::cout << "{\"event\":\"move\",\"code\":" << rc << "}" << std::endl;
      if (rc!=0) throw std::runtime_error("Move RPC failed");
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    ok=true;
  } catch (const std::exception& e) { std::cerr << e.what() << std::endl; }
  int avoidance_stop = obstacle_client.Move(0, 0, 0);
  obstacle_client.UseRemoteCommandFromApi(false);
  int stopped=stop(client);
  // Observe settling after StopMove, but never describe RPC acknowledgement as
  // proof of exact displacement or stationary legs.
  std::this_thread::sleep_for(std::chrono::milliseconds(700));
  auto final=reader.sample(); print_state(final);
  bool final_healthy=check_state(final,false,"after_stop");
  return ok && avoidance_stop == 0 && stopped==0 && final_healthy ? 0 : 1;
}
int main(int argc,char** argv) {
  std::signal(SIGINT,on_signal); std::signal(SIGTERM,on_signal);
  try { return run(argc,argv); }
  catch(const std::exception& e) { std::cerr << e.what() << std::endl; return 1; }
}

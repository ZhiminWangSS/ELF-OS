#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <atomic>
#include <chrono>
#include <thread>
#include <iostream>
int main(){std::atomic<unsigned> count{0},nonzero{0};unitree::robot::ChannelFactory::Instance()->Init(0,"eth0");unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::LowState_> sub("rt/lowstate");sub.InitChannel([&](const void*p){auto&m=*static_cast<const unitree_go::msg::dds_::LowState_*>(p);++count;for(auto b:m.wireless_remote())if(b){++nonzero;break;}},1);std::this_thread::sleep_for(std::chrono::seconds(3));std::cout<<"{\"read_only\":true,\"lowstate_samples\":"<<count.load()<<",\"samples_with_remote_bytes\":"<<nonzero.load()<<"}\n";}

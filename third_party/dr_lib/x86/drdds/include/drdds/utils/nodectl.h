#ifndef NODECTL_H_
#define NODECTL_H_

#include <functional>
#include "drdds/core/drdds_core.h"
#include "log4cplus_dr/dr_logger.hpp"
#include <queue>
#include <signal.h>

enum NodeStatus { DEATH=0, LIVE};

class NodeCtl {
using ExecHandler = std::function<int(int argc, char** argv)>;
public:
  NodeCtl(std::string app_id, std::string node_name, 
          ExecHandler handler, int argc, char** argv, 
          bool autorun=false, int sig=SIGKILL);
  ~NodeCtl();
  void Run();
private:
  void StructID(std::string app_id);
  void Load();
  void Clear();
  void Start();
  void Stop();
  void Restart();
  void ChildNodeStatusUpdate(bool log_flag=true);
  void SignalCallBack(const drdds::msg::SetNode* data);
  std::string app_id_;
  std::string node_name_;
  uint16_t module_id_;
  uint16_t node_id_;
  uint16_t node_status_;
  bool autorun_;
  int sig_;
  std::queue<drdds::msg::SetNode> exe_queue_;

  ExecHandler handler_ {nullptr};
  int argc_;
  char** argv_;

  pid_t child_pid_ {-1};
  ChannelSetNode *channel_set_node_ {nullptr};
  ChannelNodeStatus *channel_node_status_ {nullptr};
  DrLogger* logger_;
};

#endif

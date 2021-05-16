#ifndef PROCESS_H
#define PROCESS_H

#include <string>
/*
Basic class for Process representation
It contains relevant attributes as shown below
*/
class Process {
 public:
  Process(int pid);
  int Pid() const;
  std::string User() const;
  std::string Command() const;
  float CpuUtilization() const;
  std::string Ram() const;
  long int UpTime() const;
  bool operator<(Process const& a) const;

  // Private members
 private:
  int pid_;
  float cpu_utilization_;
  std::string command_;
  long ram_;
  std::string user_name_;
  long int up_time_;
};

#endif

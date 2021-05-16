#include "process.h"

#include <unistd.h>

#include <cctype>
//#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "linux_parser.h"
using std::string;
using std::to_string;
using std::vector;

Process::Process(int pid)
    : pid_(pid),
      command_(LinuxParser::Command(pid)),
      ram_(std::stol(LinuxParser::Ram(pid))),
      user_name_(LinuxParser::User(pid)),
      up_time_(LinuxParser::UpTime(pid)) {
  long seconds = LinuxParser::UpTime() - up_time_;
  long total_time = LinuxParser::ActiveJiffies(pid);
  cpu_utilization_ = float(total_time) / float(seconds);
}
// Return this process's ID
int Process::Pid() const { return pid_; }

// Return this process's CPU utilization
float Process::CpuUtilization() const { return cpu_utilization_; }

// Return the command that generated this process
string Process::Command() const { return command_; }

// Return this process's memory utilization
string Process::Ram() const { return std::to_string(ram_); }

// Return the user (name) that generated this process
string Process::User() const { return user_name_; }

// Return the age of this process (in seconds)
long int Process::UpTime() const { return up_time_; }

// Overload the "less than" comparison operator for Process objects
bool Process::operator<(Process const& a) const {
  return CpuUtilization() < a.cpu_utilization_;
}

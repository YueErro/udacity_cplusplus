#include "system.h"

#include <unistd.h>

//#include <algorithm>
#include <cstddef>
#include <set>
#include <string>
#include <vector>

//#include "format.h"
#include "linux_parser.h"
#include "process.h"
#include "processor.h"

using std::set;
using std::size_t;
using std::string;
using std::vector;

// Return the system's CPU
Processor& System::Cpu() { return cpu_; }

// Return a container composed of the system's processes
vector<Process>& System::Processes() {
  processes_.clear();
  std::vector<int> pids = LinuxParser::Pids();
  for (long unsigned int i = 0; i < pids.size(); ++i) {
    if (!LinuxParser::Ram(pids[i]).empty()) {
      processes_.emplace_back(Process(pids[i]));
    }
  }
  std::sort(processes_.rbegin(), processes_.rend());
  return processes_;
}

// Return the system's kernel identifier (string)
std::string System::Kernel() const { return LinuxParser::Kernel(); }

// Return the system's memory utilization
float System::MemoryUtilization() const {
  return LinuxParser::MemoryUtilization();
}

// Return the operating system name
std::string System::OperatingSystem() const {
  return LinuxParser::OperatingSystem();
}

// Return the number of processes actively running on the system
int System::RunningProcesses() const { return LinuxParser::RunningProcesses(); }

// Return the total number of processes on the system
int System::TotalProcesses() const { return LinuxParser::TotalProcesses(); }

// Return the number of seconds since the system started running
long int System::UpTime() const { return LinuxParser::UpTime(); }

#include "linux_parser.h"

#include <dirent.h>
#include <unistd.h>

#include <filesystem>
#include <string>
#include <vector>

using std::stof;
using std::string;
using std::to_string;
using std::vector;

// DONE: An example of how to read data from the filesystem
string LinuxParser::OperatingSystem() {
  string line;
  string key;
  string value;
  std::ifstream filestream(kOSPath);
  if (filestream.is_open()) {
    while (std::getline(filestream, line)) {
      std::replace(line.begin(), line.end(), ' ', '_');
      std::replace(line.begin(), line.end(), '=', ' ');
      std::replace(line.begin(), line.end(), '"', ' ');
      std::istringstream linestream(line);
      while (linestream >> key >> value) {
        if (key == "PRETTY_NAME") {
          std::replace(value.begin(), value.end(), '_', ' ');
          return value;
        }
      }
    }
  }
  return value;
}

// DONE: An example of how to read data from the filesystem
string LinuxParser::Kernel() {
  string os, version, kernel;
  string line;
  std::ifstream stream(kProcDirectory + kVersionFilename);
  if (stream.is_open()) {
    std::getline(stream, line);
    std::istringstream linestream(line);
    linestream >> os >> version >> kernel;
  }
  return kernel;
}

// BONUS: using std::filesystem
vector<int> LinuxParser::Pids() {
  std::vector<int> pids;
  std::string filename;
  for (const auto& p : std::filesystem::directory_iterator(kProcDirectory)) {
    // Is this a directory?
    if (std::filesystem::is_directory(p)) {
      // Is every character of the name a digit?
      filename = p.path().filename();
      if (std::all_of(filename.begin(), filename.end(), isdigit)) {
        pids.emplace_back(stoi(filename));
      }
    }
  }
  return pids;
}

// Read and return the system memory utilization
float LinuxParser::MemoryUtilization() {
  std::ifstream stream(kProcDirectory + kMeminfoFilename);
  if (stream.is_open()) {
    std::string line;
    std::string key;
    std::string value;
    float mem_total = -1.0;
    float mem_free = -1.0;
    while (std::getline(stream, line) &&
           (mem_total == -1.0 || mem_free == -1.0)) {
      std::istringstream linestream(line);
      linestream >> key >> value;
      if (key == kMemTotal) {
        mem_total = std::stof(value);
      } else if (key == kMemFree) {
        mem_free = std::stof(value);
      }
    }
    // Current system memory usage
    return (mem_total - mem_free) / mem_total;
  }
  // Information not available
  return -1.0;
}

// Read and return the system uptime
long LinuxParser::UpTime() {
  std::ifstream stream(kProcDirectory + kUptimeFilename);
  if (stream.is_open()) {
    std::string line;
    std::string up_time;
    std::getline(stream, line);
    std::istringstream linestream(line);
    linestream >> up_time;
    // Current system memory usage
    return std::stol(up_time);
  }
  // Information not available
  return -1;
}

// Read and return the number of jiffies for the system
long LinuxParser::Jiffies() { return ActiveJiffies() + IdleJiffies(); }

// Read and return the number of active jiffies for a PID
long LinuxParser::ActiveJiffies(int pid) {
  std::ifstream stream(kProcDirectory + std::to_string(pid) + kStatFilename);
  long total_time = -1;
  if (stream.is_open()) {
    total_time = 0;
    std::string line;
    std::string time;
    const int time_pos = 17;
    std::getline(stream, line);
    std::istringstream linestream(line);
    for (int i = 0; i < time_pos; ++i) {
      linestream >> time;
      if (i > 12 && i < time_pos) {
        total_time += std::stol(time);
      }
    }
  }
  return total_time / sysconf(_SC_CLK_TCK);
}

// Read and return the number of active jiffies for the system
long LinuxParser::ActiveJiffies() {
  std::vector<std::string> jiffies = CpuUtilization();

  return stol(jiffies[CPUStates::kUser_]) + stol(jiffies[CPUStates::kNice_]) +
         stol(jiffies[CPUStates::kSystem_]) + stol(jiffies[CPUStates::kIRQ_]) +
         stol(jiffies[CPUStates::kSoftIRQ_]) +
         stol(jiffies[CPUStates::kSteal_]);
}

// Read and return the number of idle jiffies for the system
long LinuxParser::IdleJiffies() {
  std::vector<std::string> jiffies = CpuUtilization();
  return stol(jiffies[CPUStates::kIdle_]) + stol(jiffies[CPUStates::kIOwait_]);
}

// Read and return CPU utilization
vector<string> LinuxParser::CpuUtilization() {
  std::vector<std::string> jiffies;
  std::ifstream stream(kProcDirectory + kStatFilename);
  if (stream.is_open()) {
    std::string line;
    std::string value;
    std::getline(stream, line);
    std::istringstream linestream(line);
    linestream >> value;
    while (linestream >> value) {
      jiffies.emplace_back(value);
    }
  }
  return jiffies;
}

// Read and return the total number of processes
int LinuxParser::TotalProcesses() {
  int num_processes = -1;
  std::ifstream stream(kProcDirectory + kStatFilename);
  if (stream.is_open()) {
    std::string line;
    std::string key;
    std::string value;
    while (std::getline(stream, line)) {
      std::istringstream linestream(line);
      linestream >> key >> value;
      if (key == kStatTotalProceses) {
        num_processes = std::stoi(value);
        break;
      }
    }
  }
  return num_processes;
}

// Read and return the number of running processes
int LinuxParser::RunningProcesses() {
  int num_running_processes = -1;
  std::ifstream stream(kProcDirectory + kStatFilename);
  if (stream.is_open()) {
    std::string line;
    std::string key;
    std::string value;
    while (std::getline(stream, line)) {
      std::istringstream linestream(line);
      linestream >> key >> value;
      if (key == kStatRunningProceses) {
        num_running_processes = std::stoi(value);
        break;
      }
    }
  }
  return num_running_processes;
}

// Read and return the command associated with a process
string LinuxParser::Command(int pid) {
  std::string command = "-1";
  std::ifstream stream(kProcDirectory + std::to_string(pid) + kCmdlineFilename);
  if (stream.is_open()) {
    std::string line;
    std::getline(stream, line);
    std::istringstream linestream(line);
    linestream >> command;
  }
  // Information not available
  return command;
}

// Read and return the memory used by a process
string LinuxParser::Ram(int pid) {
  std::string memory = "-1";
  std::ifstream stream(kProcDirectory + std::to_string(pid) + kStatusFilename);
  if (stream.is_open()) {
    std::string line;
    std::string key;
    std::string value;
    while (std::getline(stream, line)) {
      std::istringstream linestream(line);
      linestream >> key >> value;
      // Using VmData instead of VmSize in order to get the exact physical
      // memory instead of the virtual memory
      if (key == kStatusVmData) {
        // Convert from KB to MB
        memory = std::to_string(std::stoi(value) / 1024);
        break;
      }
    }
  }
  return memory;
}

// Read and return the user ID associated with a process
string LinuxParser::Uid(int pid) {
  std::string u_id = "-1";
  std::ifstream stream(kProcDirectory + std::to_string(pid) + kStatusFilename);
  if (stream.is_open()) {
    std::string line;
    std::string key;
    std::string value;
    while (std::getline(stream, line)) {
      std::istringstream linestream(line);
      linestream >> key >> value;
      if (key == kStatusUid) {
        u_id = value;
        break;
      }
    }
  }
  return u_id;
}

// Read and return the user associated with a process
string LinuxParser::User(int pid) {
  std::ifstream stream(kPasswordPath);
  std::string username = "-1";
  if (stream.is_open()) {
    std::string line;
    std::string uid = Uid(pid);
    while (std::getline(stream, line)) {
      std::istringstream linestream(line);
      if (line.find(":x:" + uid + ":" + uid) != std::string::npos) {
        std::stringstream ss(line);
        std::getline(ss, username, ':');
        break;
      }
    }
  }
  return username;
}

// Read and return the uptime of a process
long LinuxParser::UpTime(int pid) {
  std::ifstream stream(kProcDirectory + std::to_string(pid) + kStatFilename);
  if (stream.is_open()) {
    std::string line;
    std::string up_time;
    std::getline(stream, line);
    std::istringstream linestream(line);
    const int up_time_pos = 22;
    for (int i = 0; i < up_time_pos; ++i) {
      linestream >> up_time;
    }
    return stol(up_time) / sysconf(_SC_CLK_TCK);
  }
  return -1;
}

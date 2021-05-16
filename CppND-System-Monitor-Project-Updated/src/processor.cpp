#include "processor.h"

#include "linux_parser.h"

// Return the aggregate CPU utilization
float Processor::Utilization() {
  long total_old, total_new, active_new, idle_old, idle_new;
  total_new = CurrentTotal();
  active_new = CurrentActive();
  idle_new = CurrentIdle();
  total_old = PrevTotal();
  idle_old = PrevIdle();
  Update(idle_new, active_new, total_new);
  float total_diff = float(total_new) - float(total_old);
  float idle_diff = float(idle_new) - float(idle_old);
  // Current CPU memory usage
  return (total_diff - idle_diff) / total_diff;
}

long Processor::CurrentTotal() const { return LinuxParser::Jiffies(); }
long Processor::CurrentActive() const { return LinuxParser::ActiveJiffies(); }
long Processor::CurrentIdle() const { return LinuxParser::IdleJiffies(); }

long Processor::PrevTotal() const { return total_; }
long Processor::PrevActive() const { return active_; }
long Processor::PrevIdle() const { return idle_; }
void Processor::Update(long idle, long active, long total) {
  idle_ = idle;
  active_ = active;
  total_ = total;
}

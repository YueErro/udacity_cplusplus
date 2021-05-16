#include "format.h"

#include <chrono>
#include <iomanip>
#include <string>

using std::string;

// Complete this helper function
// INPUT: Long int measuring seconds
// OUTPUT: HH:MM:SS
string Format::ElapsedTime(long seconds) {
  std::chrono::seconds secs{seconds};
  // return std::chrono::format("%T", seconds); // in C++20 :-)
  std::chrono::hours hours =
      std::chrono::duration_cast<std::chrono::hours>(secs);
  secs -= std::chrono::duration_cast<std::chrono::seconds>(hours);
  std::chrono::minutes minutes =
      std::chrono::duration_cast<std::chrono::minutes>(secs);
  secs -= std::chrono::duration_cast<std::chrono::seconds>(minutes);

  std::stringstream ss{};
  ss << std::setw(2) << std::setfill('0') << hours.count()     // HH
     << std::setw(1) << ":"                                    // :
     << std::setw(2) << std::setfill('0') << minutes.count()   // MM
     << std::setw(1) << ":"                                    // :
     << std::setw(2) << std::setfill('0') << secs.count();  // SS

  return ss.str();
}

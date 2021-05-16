#include "format.h"

#include <string>

using std::string;

// Complete this helper function
// INPUT: Long int measuring seconds
// OUTPUT: HH:MM:SS
string Format::ElapsedTime(long seconds) {
  int hours;
  int minutes;
  hours = seconds / 3600;
  seconds = seconds % 3600;
  minutes = seconds / 60;
  seconds = seconds % 60;
  return std::to_string(hours) + ":" + std::to_string(minutes) + ":" +
         std::to_string(seconds);
}

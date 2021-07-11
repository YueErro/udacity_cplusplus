#ifndef TRAFFICLIGHT_H
#define TRAFFICLIGHT_H

#include <mutex>
#include <deque>
#include <condition_variable>
#include "TrafficObject.h"

// forward declarations to avoid include cycle
class Vehicle;

enum TrafficLightPhase
{
  RED,
  GREEN
};

template <class T>
class MessageQueue
{
public:
  void send(T &&msg);
  T receive();

private:
  std::deque<T> _dequeue;
  std::condition_variable _condition;
  std::mutex _mutex;
};

class TrafficLight : public TrafficObject
{
  static const int MIN = 4;
  static const int MAX = 6;

public:
  // constructor / desctructor
  TrafficLight();
  ~TrafficLight();
  // getters / setters
  TrafficLightPhase getCurrentPhase();
  // typical behaviour methods
  void waitForGreen();
  void simulate();

private:
  // typical behaviour methods
  void cycleThroughPhases();

  MessageQueue<TrafficLightPhase> _queue;
  std::condition_variable _condition;
  std::mutex _mutex;
  TrafficLightPhase _currentPhase;
};

#endif

#include <iostream>
#include <random>
#include <future>
#include "TrafficLight.h"

#define MIN 4.0
#define MAX 6.0

/* Implementation of class "MessageQueue" */
template <typename T>
T MessageQueue<T>::receive()
{
  std::unique_lock<std::mutex> lock(_mutex);
  _condition.wait(lock, [this] { return !_dequeue.empty(); });

  T msg = std::move(_dequeue.front());
  _dequeue.pop_front();
  return msg;
}

template <typename T>
void MessageQueue<T>::send(T &&msg)
{
  std::lock_guard<std::mutex> lock(_mutex);
  _dequeue.clear();
  _dequeue.emplace_back(msg);
  _condition.notify_one();
}

/* Implementation of class "TrafficLight" */
TrafficLight::TrafficLight()
{
  _currentPhase = TrafficLightPhase::RED;
}

TrafficLight::~TrafficLight()
{
}

void TrafficLight::waitForGreen()
{
  while (true)
  {
    if (_queue.receive() == TrafficLightPhase::GREEN)
    {
      return;
    }
  }
}

TrafficLightPhase TrafficLight::getCurrentPhase()
{
  return _currentPhase;
}

void TrafficLight::simulate()
{
  threads.emplace_back(std::thread(&TrafficLight::cycleThroughPhases, this));
}

// virtual function which is executed in a thread
void TrafficLight::cycleThroughPhases()
{
  // Init our random generation between 4 and 6 seconds
  static std::random_device rd;
  static std::mt19937 mt(rd());
  static std::uniform_real_distribution<double> dist(MIN, MAX);

  // Initialize variables
  double cycle_duration =
      dist(mt);  // Duration of a single simulation cycle in seconds, is randomly chosen

  std::chrono::duration<double> current_time;
  std::chrono::duration<double> time_diff;
  std::chrono::duration<double> start_time =
      std::chrono::high_resolution_clock::now().time_since_epoch();
  while (true)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    current_time = std::chrono::high_resolution_clock::now().time_since_epoch();
    time_diff = current_time - start_time;
    if (time_diff.count() >= cycle_duration)
    {
      _currentPhase = _currentPhase == TrafficLightPhase::RED ? TrafficLightPhase::GREEN :
                                                                TrafficLightPhase::RED;
      _queue.send(std::move(_currentPhase));

      cycle_duration = dist(mt);
      start_time = std::chrono::high_resolution_clock::now().time_since_epoch();
    }
  }
}

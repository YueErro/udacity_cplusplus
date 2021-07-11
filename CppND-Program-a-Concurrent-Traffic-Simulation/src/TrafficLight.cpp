#include <iostream>
#include <random>
#include "TrafficLight.h"

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
  _dequeue.emplace_back(msg);
  _condition.notify_one();
}

/* Implementation of class "TrafficLight" */
TrafficLight::TrafficLight()
{
  _currentPhase = TrafficLightPhase::RED;
}

void TrafficLight::waitForGreen()
{
  while (true)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
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
  // FP.2b : Finally, the private method „cycleThroughPhases“ should be started in a thread
  // when the public method „simulate“ is called. To do this, use the thread queue in the base class.
}

// virtual function which is executed in a thread
void TrafficLight::cycleThroughPhases()
{
  // FP.2a : Implement the function with an infinite loop that measures the time between
  // two loop cycles and toggles the current phase of the traffic light between red and
  // green and sends an update method to the message queue using move semantics. The cycle
  // duration should be a random value between 4 and 6 seconds. Also, the while-loop
  // should use std::this_thread::sleep_for to wait 1ms between two cycles.
}

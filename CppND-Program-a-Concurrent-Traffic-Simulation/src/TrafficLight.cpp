#include <iostream>
#include <random>
#include <future>
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
  srand(time(NULL));
  double cycle_duration = MIN + ((double)rand() / RAND_MAX) * (MAX - MIN);

  std::chrono::duration<double> start_time =
      std::chrono::system_clock::now().time_since_epoch();
  std::chrono::duration<double> current_time;
  std::chrono::duration<double> time_diff;
  while (true)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    current_time = std::chrono::system_clock::now().time_since_epoch();
    time_diff = current_time - start_time;
    if (time_diff.count() >= cycle_duration)
    {
      _currentPhase = _currentPhase == TrafficLightPhase::RED ? TrafficLightPhase::GREEN :
                                                                TrafficLightPhase::RED;
      auto sentFuture = std::async(std::launch::async, &MessageQueue<TrafficLightPhase>::send,
                                   &_queue, std::move(_currentPhase));
      sentFuture.wait();
    }
  }
}

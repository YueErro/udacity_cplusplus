#ifndef PROCESSOR_H
#define PROCESSOR_H

class Processor {
 public:
  float Utilization();
  long CurrentTotal() const;
  long CurrentActive() const;
  long CurrentIdle() const;
  long PrevTotal() const;
  long PrevIdle() const;
  long PrevActive() const;
  void Update(long idle, long active, long total);

  // Private members
 private:
  long idle_;
  long active_;
  long total_;
};

#endif

#ifndef Aggregate_h
#define Aggregate_h

template<
  class T,        // the type to be averaged
  class TA=T      // the type for the accumulator (normally same as T, but in casees of 
                  // small integers we may want to have a larger accumulator to prevent roll-over
                  // of the sum
>
class Aggregate
{
public:
    T minimum;
    T maximum;
    T sum;
    T count;
    T last;


  Aggregate() : minimum(0), maximum(0), sum(0), count(0), last(0) {}

	void clear() {
    minimum = maximum = last = 0;
	}
 
	void add(T value) {
    if(count==0) {
      minimum = value;
      maximum = value;
    } else {
      if(value > maximum)
        maximum = value;
      if(value < minimum)
        minimum = value;
    }
    last = value;
    sum += value;
    count++;
  }
	
  T average() const {
    return (T)(sum/count);
  }
};

#endif

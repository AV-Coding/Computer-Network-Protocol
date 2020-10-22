#include <math.h>
#include "Frame.h"

Frame::Frame(){
	U = 0;
	X = 0;
	backoff = 0;
	completeTime = 0;
}
Frame::Frame(double _U, int _X, int _backoff, int _completeTime){
	U = _U;
	X = _X;
	backoff = _backoff;
	completeTime = _completeTime;
}


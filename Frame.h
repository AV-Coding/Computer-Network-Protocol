#ifndef Frame_H
#define Frame_H

#include <string>
#include <iostream>
#include <vector>
#include <string>

using namespace std;

class Frame{
private:
	double U;
	int X;
	int backoff;
	int completeTime;

public:
	Frame();

	Frame(double U, int X, int backoff, int completeTime);

	double GetU(){ return U; };
	int GetX() { return X; };
	int GetBackoff() { return backoff; };
	int GetCompleteTime(){ return completeTime; };

	void SetU(double _U) { U = _U; };
	void SetX(int _X) { X = _X; };
	void SetBackoff(int _backoff){ backoff = _backoff; };
	void SetCompleteTime(double _completeTime) { completeTime = _completeTime; };
};

#endif
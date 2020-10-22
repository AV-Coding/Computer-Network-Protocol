#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <iostream>
#include <vector>
#include <queue>
#include <assert.h>
#include "Frame.h"

using namespace std;

const int SIFS = 1;
const int ACK = 2;
const int DIFS = 2;
const int PAYLOAD = 50;
const int SIMTIME = 10;
const int RTS = 2;
const int CTS = 2;
const double SLOT = 0.00001;
const int MIN_CW = 4;
const int MAX_CW = 1024;
const int MAX_SLOT_TIME = 1e6;

int numberOfFrames= 20;
vector<Frame> NodeA;
vector<Frame> NodeC;
vector<int> Xa(numberOfFrames);
vector<int> Xc(numberOfFrames);
const double LAMDA = 500;
int counterA = 0;
int counterC = 0;
int counterCollision = 0;
int maxBackoff = MIN_CW;
int indexA = 0;
int indexC = 0;
bool sentAllA = false;
bool sentAllC = false;
int afterBackoff_A = 0;  //current A slot time upon backoff reaching 0 
int afterBackoff_C = 0; //current C slot time upon backoff reaching 0 
int currTime = 0;
int random = 0; 
bool virtCS = false;


void generatePoissonTraffic();
void resetACKTime();
void performSimulation();
void simulateCSMA(bool vcs); 
void simulateHiddenCSMA(bool vcs);
void transmitA_CSMA(bool vcs);
void transmitC_CSMA(bool vcs);
void printCounter();
void generateDummyData();
void initData();

void initData(){
	counterA = 0;
	counterC = 0;
	counterCollision = 0;
	sentAllA = false;
	sentAllC = false;
	indexA = 0;
	indexC = 0;
	currTime = 0;
}

int main(){
	srand((unsigned)time(0));  // Initialize random number generator.
	// define lamda for A and C
	//int lamda[5] = {200, 300, 500, 1000, 2000};

	numberOfFrames = LAMDA * SIMTIME;

	performSimulation();
	
	getchar();
	return 0;
}

void performSimulation(){
	cout << endl << "----------Simulating Same CSMA without VCS----------" << endl;
	//generateDummyData(); //to be replaced by generatePoissonTraffic()
	generatePoissonTraffic();
	simulateCSMA(virtCS = false);	//1A
	printCounter();
	cout << endl << "----------Simulating Same CSMA with VCS----------" << endl;
	//generateDummyData();
	resetACKTime();
	simulateCSMA(virtCS = true);	//2A
	printCounter();
	cout << endl << "----------Simulating Hidden CSMA without VCS----------" << endl;
	resetACKTime();
	simulateHiddenCSMA(virtCS = false);	//1B
	printCounter();
	cout << endl << "----------Simulating Hidden CSMA with VCS----------" << endl;
	resetACKTime();
	simulateHiddenCSMA(virtCS = true);	//2B
	printCounter();
}

void resetACKTime(){
	maxBackoff = MIN_CW;
	for (int i = 0; i < numberOfFrames; i++){
		NodeA[i].SetCompleteTime(0);
		NodeC[i].SetCompleteTime(0);
		int random = rand() % maxBackoff; // create random back off for A
		NodeA[i].SetBackoff(random);
		random = rand() % maxBackoff; // create random back off for C
		NodeC[i].SetBackoff(random);
	}
}


void generatePoissonTraffic(){
	double xa1 = 0;
	double xa2 = 0;
	int xa3 = 0;
	double xc1 = 0;
	double xc2 = 0;
	int xc3 = 0;

	NodeA.resize(0);
	NodeC.resize(0);
	srand(time(0));

	//totalFrames = LAMDA * SIMTIME;
	//Generate 2 sets for A and C, each set has totalFrames numbers
	for (int i = 0; i < numberOfFrames; i++){
		int random = rand() % maxBackoff; // create random back off for A
		NodeA.push_back(Frame(0.0, 0, random, 0));
		random = rand() % maxBackoff; // create random back off for C
		NodeC.push_back(Frame(0.0, 0, random, 0));
		NodeA[i].SetU((float)rand() / RAND_MAX);
		NodeC[i].SetU((float)rand() / RAND_MAX);
	}

	for (int i = 0; i < NodeA.size(); i++){
		//Convert Ua and Uc to Xa and Xc by Xa = -1/lamda*ln(1-Ua)
		xa1 = -(1 / LAMDA)*log(1 - NodeA[i].GetU());
		xc1 = -(1 / LAMDA)*log(1 - NodeC[i].GetU());
		//Convert Xa and Xc into of slot unit by divide each elements from Xa to SLOT value
		xa2 = xa1 / SLOT;
		xa2 = (int)xa2;
		xc2 = xc1 / SLOT;
		xc2 = (int)xc2;
		NodeA[i].SetX(xa2);
		NodeC[i].SetX(xc2);
		//Convert the packet time to actual arrival time by replace x2 = x1+x2, x3 = x1+x2+x3,...
		if (i > 0){
			xa3 = NodeA[i].GetX() + NodeA[i - 1].GetX();
			NodeA[i].SetX(xa3);
			xc3 = NodeC[i].GetX() + NodeC[i - 1].GetX();
			NodeC[i].SetX(xc3);
		}
	}

	//for (int i = 0; i < NodeA.size(); i++){
	//	cout << "NodeA[" << i << "]" << " U:" << NodeA[i].GetU() << " X:" << NodeA[i].GetX() << " backoff: " << NodeA[i].GetBackoff() << " ACK received time: " << NodeA[i].GetCompleteTime() << endl;
	//	cout << "NodeC[" << i << "]" << " U:" << NodeC[i].GetU() << " X:" << NodeC[i].GetX() << " backoff: " << NodeC[i].GetBackoff() << " ACK received time: " << NodeC[i].GetCompleteTime() << endl;
	//}
}

void printCounter(){
	std::cout << "NodeA frame counter: " << counterA << endl;
	std::cout << "NodeC frame counter: " << counterC << endl;
	std::cout << "Collision counter: " << counterCollision << endl;
	/*std::cout << "ACK time of Last Frame on node A : " << NodeA[indexA-1].GetCompleteTime() << endl;
	std::cout << "ACK time of Last Frame on node C : " << NodeC[indexC-1].GetCompleteTime() << endl;*/
}

void generateDummyData(){
	//Xa = { 100, 256, 394, 400, 550, 660, 700, 770, 890, 980 };
	//Xc = { 130, 246, 404, 420, 530, 680, 750, 850, 930, 990 };
	Xa = { 100, 256, 394, 400, 550, 660, 700, 770, 890, 980 };
	Xc = { 130, 256, 394, 400, 550, 660, 750, 850, 930, 990 };
	NodeA.resize(0);
	NodeC.resize(0);
	for (int i = 0; i < Xa.size(); i++){
		int random = rand() % maxBackoff;
		NodeA.push_back(Frame(0.0, Xa[i], random, 0));
	}

	for (int i = 0; i < Xc.size(); i++){
		int random = rand() % maxBackoff;
		NodeC.push_back(Frame(0.0, Xc[i], random, 0));
	}


	for (int i = 0; i < NodeA.size(); i++){
		cout << "NodeA[" << indexA << "] X:" << NodeA[i].GetX() << " backoff: " << NodeA[i].GetBackoff() << " ACK received time: " << NodeA[i].GetCompleteTime() << endl;
		cout << "NodeC[" << indexC << "] X:" << NodeC[i].GetX() << " backoff: " << NodeC[i].GetBackoff() << " ACK received time: " << NodeC[i].GetCompleteTime() << endl;
	}
}



void simulateCSMA(bool vcs){
	initData();

	int currXa = 0;
	int currXc = 0;
	int currBa = 0;
	int currBc = 0;
	int boDelta = 0; 
	int afterDIFa = 0; //actual time to start DIF
	int afterDIFc = 0;

	while ((!(sentAllA) || !(sentAllC)) && (currTime < MAX_SLOT_TIME)){
		/*cout << "Current Time is " << currTime << endl;*/
		if (!(sentAllA)){
			currXa = NodeA[indexA].GetX();
			currBa = NodeA[indexA].GetBackoff();

			if (NodeA[indexA].GetX() < currTime){
				afterDIFa = currTime + DIFS;
				afterBackoff_A = currTime + DIFS + NodeA[indexA].GetBackoff();
			}
			else{
				afterDIFa = NodeA[indexA].GetX() + DIFS;
				afterBackoff_A = NodeA[indexA].GetX() + DIFS + NodeA[indexA].GetBackoff(); //calculate end of backoff time (upon reaching 0)
			}

		}
		else{
			afterDIFa = INT_MAX;
			afterBackoff_A = INT_MAX;
		}
		if (!(sentAllC)){
			currXc = NodeC[indexC].GetX();
			currBc = NodeC[indexC].GetBackoff();

			if (NodeC[indexC].GetX() < currTime){
				afterDIFc = currTime + DIFS;
				afterBackoff_C = currTime + DIFS + NodeC[indexC].GetBackoff();
			}
			else{
				afterDIFc = NodeC[indexC].GetX() + DIFS;
				afterBackoff_C = NodeC[indexC].GetX() + DIFS + NodeC[indexC].GetBackoff(); //calculate end of backoff time (upon reaching 0)
			}

		}
		else{
			afterDIFc = INT_MAX;
			afterBackoff_C = INT_MAX;
		}

		boDelta = afterBackoff_C - afterBackoff_A;
		
		if (boDelta > 0){
			if (afterDIFc < afterBackoff_A){
				NodeC[indexC].SetBackoff(boDelta);
			}
			transmitA_CSMA(vcs);
			indexA++;
			if (indexA >= NodeA.size()){ sentAllA = true; }
		}
		else if (boDelta == 0){
			//cout << "Collision has occured between " << "NodeA[" << indexA << "] and " "NodeC[" << indexC << "]" <<endl;
			counterCollision++;
			maxBackoff = 2 * maxBackoff; //double maxbackoff
			if (maxBackoff > MAX_CW) { maxBackoff = MAX_CW; }
			random = rand() % maxBackoff;
			NodeA[indexA].SetBackoff(random);
			//cout << "NodeA[" << indexA << "] backoff is now " << random <<endl;
			random = rand() % maxBackoff;
			NodeC[indexC].SetBackoff(random);
			//cout << "NodeC[" << indexC << "] backoff is now " << random << endl;
			if (currTime < NodeA[indexA].GetX()){
				currTime = NodeA[indexA].GetX();//advance time forward to next slot in A 
			}
			if (vcs){
				currTime = currTime + DIFS + NodeA[indexA].GetBackoff() + RTS + SIFS + CTS;
			}
			else{
				currTime = currTime + DIFS + NodeA[indexA].GetBackoff() + PAYLOAD + SIFS + ACK;
			}

		}
		else{
			if (afterDIFa < afterBackoff_C){
				NodeA[indexA].SetBackoff(0-boDelta);
			}
			transmitC_CSMA(vcs);
			indexC++;
			if (indexC >= NodeC.size()){ sentAllC = true; }
		}
	}

	//for (int i = 0; i < NodeA.size(); i++){
	//	cout << "NodeA[" << i << "] X:" << NodeA[i].GetX() << " backoff: " << NodeA[i].GetBackoff() << " Complete Time: " << NodeA[i].GetCompleteTime() << endl;
	//	cout << "NodeC[" << i << "] X:" << NodeC[i].GetX() << " backoff: " << NodeC[i].GetBackoff() << " Complete Time: " << NodeC[i].GetCompleteTime() << endl;
	//}

	//assert(NodeA[0].GetCompleteTime() > 150 && NodeA[0].GetCompleteTime() < 180);
	//cout << "NodeA[0].GetCompleteTime() IS GOOD" << endl;
	//assert(NodeC[0].GetCompleteTime() > 250 && NodeC[0].GetCompleteTime() < 400);
	//cout << "NodeC[0].GetCompleteTime() IS GOOD" << endl;
}



void simulateHiddenCSMA(bool vcs){
	initData();

	int currXa = 0;
	int currXc = 0;
	int currBa = 0;
	int currBc = 0;
	int xmissionTimeA = 0;
	int xmissionTimeC = 0;
	int afterDIFa = 0; //actual time to start DIF
	int afterDIFc = 0;

	while ((!(sentAllA) || !(sentAllC)) && (currTime < MAX_SLOT_TIME)){
		//cout << "Current Time is " << currTime << endl;
		if (!(sentAllA)){
			currXa = NodeA[indexA].GetX();
			currBa = NodeA[indexA].GetBackoff();

			if (NodeA[indexA].GetX() < currTime){
				afterDIFa = currTime + DIFS;
				afterBackoff_A = currTime + DIFS + NodeA[indexA].GetBackoff();				
			}
			else{
				afterDIFa = NodeA[indexA].GetX() + DIFS;
				afterBackoff_A = NodeA[indexA].GetX() + DIFS + NodeA[indexA].GetBackoff(); //calculate end of backoff time (upon reaching 0)
			}
			if (vcs){
				xmissionTimeA = afterBackoff_A + RTS + SIFS;
			}
			else{
				xmissionTimeA = afterBackoff_A + PAYLOAD + SIFS;
			}			
		}
		else{
			xmissionTimeA = INT_MAX;
			afterDIFa = INT_MAX;
			afterBackoff_A = INT_MAX;
		}
		if (!(sentAllC)){
			currXc = NodeC[indexC].GetX();
			currBc = NodeC[indexC].GetBackoff();

			if (NodeC[indexC].GetX() < currTime){
				afterDIFc = currTime + DIFS;
				afterBackoff_C = currTime + DIFS + NodeC[indexC].GetBackoff();
			}
			else{
				afterDIFc = NodeC[indexC].GetX() + DIFS;
				afterBackoff_C = NodeC[indexC].GetX() + DIFS + NodeC[indexC].GetBackoff(); //calculate end of backoff time (upon reaching 0)
			}

			if (vcs){
				xmissionTimeC = afterBackoff_C + RTS + SIFS;
			}
			else{
				xmissionTimeC = afterBackoff_C + PAYLOAD + SIFS;
			}
		}
		else{
			xmissionTimeC = INT_MAX;
			afterDIFc = INT_MAX;
			afterBackoff_C = INT_MAX;
		}
		
		if (xmissionTimeA < afterBackoff_C){
			if (afterDIFc < xmissionTimeA){
				NodeC[indexC].SetBackoff(afterBackoff_C - xmissionTimeA);
			}
			transmitA_CSMA(vcs);
			indexA++;
			if (indexA >= NodeA.size()){ sentAllA = true; }
		}
		else if (xmissionTimeC < afterBackoff_A) {
			if (afterDIFa < xmissionTimeC){
				NodeA[indexA].SetBackoff(afterBackoff_A - xmissionTimeC);
			}
			transmitC_CSMA(vcs);
			indexC++;
			if (indexC >= NodeC.size()){ sentAllC = true; }
		}
		else {
			//cout << "Collision has occured between " << "NodeA[" << indexA << "] and " "NodeC[" << indexC << "]" << endl;
			counterCollision++;
			maxBackoff = 2 * maxBackoff; //double maxbackoff
			if (maxBackoff > MAX_CW) { maxBackoff = MAX_CW; }
			random = rand() % maxBackoff;
			NodeA[indexA].SetBackoff(random);
			//cout << "NodeA[" << indexA << "] backoff is now " << random << endl;
			random = rand() % maxBackoff;
			NodeC[indexC].SetBackoff(random);
			//cout << "NodeC[" << indexC << "] backoff is now " << random << endl;
			if (currTime < NodeA[indexA].GetX()){
				currTime = NodeA[indexA].GetX();//advance time forward to next slot in A
			}
			if (vcs){
				currTime = currTime + DIFS + NodeA[indexA].GetBackoff() + RTS + SIFS + CTS;
			}
			else{
				currTime = currTime + DIFS + NodeA[indexA].GetBackoff() + PAYLOAD + SIFS + ACK;
			}
		}
	}
}



void transmitA_CSMA(bool vcs){
	if (currTime < NodeA[indexA].GetX()){
		currTime = NodeA[indexA].GetX();//advance time forward to next slot in A
	}

	if (vcs){
		currTime = currTime + DIFS + NodeA[indexA].GetBackoff() + RTS + CTS + PAYLOAD + 3 * SIFS + ACK;
	}
	else{
		currTime = currTime + DIFS + NodeA[indexA].GetBackoff() + PAYLOAD + SIFS + ACK;
	}

	NodeA[indexA].SetCompleteTime(currTime);
	maxBackoff = MIN_CW;
	counterA++;
	//cout << "NodeA[" << indexA << "] X:" << NodeA[indexA].GetX() << " backoff: " << NodeA[indexA].GetBackoff() << " ACK received time: " << NodeA[indexA].GetCompleteTime() << endl;
}

void transmitC_CSMA(bool vcs){
	if (currTime < NodeC[indexC].GetX()){
		currTime = NodeC[indexC].GetX();//advance time forward to next slot in C
	}

	if (vcs){
		currTime = currTime + DIFS + NodeC[indexC].GetBackoff() + RTS + CTS + PAYLOAD + 3 * SIFS + ACK;
	}
	else{
		currTime = currTime + DIFS + NodeC[indexC].GetBackoff() + PAYLOAD + SIFS + ACK;
	}

	NodeC[indexC].SetCompleteTime(currTime);
	maxBackoff = MIN_CW;
	counterC++;
	//cout << "NodeC[" << indexC << "] X:" << NodeC[indexC].GetX() << " backoff: " << NodeC[indexC].GetBackoff() << " ACK received time: " << NodeC[indexC].GetCompleteTime() << endl;
}
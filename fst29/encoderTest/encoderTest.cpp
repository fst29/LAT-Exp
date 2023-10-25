#include <iostream>
#include <wiringPi.h>
#include <unistd.h>
using namespace std;
int main()
{
	wiringPiSetup();
	pinMode(0, INPUT);
	pinMode(2, INPUT);
	pinMode(3, INPUT);

	// See encoder datasheet
	int pinA = 0;
	int pinB = 0;

	while (1)
	{
		usleep(100000);
		pinA = digitalRead(0);
		pinB = digitalRead(2);
		cout << "P0: " << pinA << " P2: " << pinB << endl;
	}
}

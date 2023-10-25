#include <iostream>
#include <wiringPi.h>
#include <unistd.h>
using namespace std;

// State of encoder outputs
int A_state = 0;
int B_state = 0;

int position = 0;  // Measured in encoder ticks
int direction = 1; // +1 -> CCW, -1 -> CW

int encoder_resolution = 1024; // tick per rotation

int encoder_state = 0000;
// The pin numbers of the encoder outputs (using wiringPi convention)
int encoder_A_pin_number = 0;
int encoder_B_pin_number = 2;

int state_transition_matrix[16] = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0};

void callback(void)
{
    encoder_state = ((encoder_state << 2) & 0x0F) + (digitalRead(encoder_A_pin_number) << 1) + digitalRead(encoder_B_pin_number);
    position = position + state_transition_matrix[encoder_state];
}

void A_callback(void)
{

    A_state = !A_state;
    if (B_state != A_state)
        direction = -1; // CW
    else
        direction = 1; // CCW

    position += direction;
}

void B_callback(void)
{
    B_state = !B_state;
    if (A_state != B_state)
        direction = 1; // CCW
    else
        direction = -1; // CW

    position += direction;
}

int main()
{
    wiringPiSetup();

    // Initialise the encoder pins
    pinMode(encoder_A_pin_number, INPUT);
    pinMode(encoder_B_pin_number, INPUT);

    A_state = digitalRead(encoder_A_pin_number);
    B_state = digitalRead(encoder_B_pin_number);
    encoder_state = (A_state << 1) + B_state;
    // cout<<(1<<1);
    //  Attach interrupts to pin state changes
    wiringPiISR(encoder_A_pin_number, INT_EDGE_BOTH, callback);
    wiringPiISR(encoder_B_pin_number, INT_EDGE_BOTH, callback);

    int rpm = 0;
    int refresh_rate = 20;

    while (1)
    {
        usleep(1000000 / refresh_rate);
        cout << "State: " << encoder_state << " position: " << position << endl;

        rpm = position / encoder_resolution * refresh_rate * 60;
        // position = 0;

        cout << "Current speed: " << rpm << " RPM, current direction: " << direction << endl;
    }
}

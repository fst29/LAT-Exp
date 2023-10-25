#define Phoenix_No_WPI // remove WPI dependencies
#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include "ctre/phoenix/cci/Unmanaged_CCI.h"
#include <string>
#include <iostream>
#include <chrono>
#include <thread>
#include <SDL2/SDL.h>
#include <unistd.h>

using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

/* make some talons for drive train */
TalonFX talRght(0);
TalonFX talLeft(1);

void initDrive()
{
	/* both talons should blink green when driving forward */
	talRght.SetInverted(true);
}

void drive(double fwd, double turn)
{
	double left = fwd - turn;
	double rght = fwd + turn; /* positive turn means turn robot LEFT */

	talLeft.Set(ControlMode::PercentOutput, left);
	talRght.Set(ControlMode::PercentOutput, rght);
}
/** simple wrapper for code cleanup */
void sleepApp(int ms)
{
	std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

int main() {
	/* don't bother prompting, just use can0 */
	//std::cout << "Please input the name of your can interface: ";
	std::string interface;
	//std::cin >> interface;
	interface = "can0";
	ctre::phoenix::platform::can::SetCANInterface(interface.c_str());
	
	// Comment out the call if you would rather use the automatically running diag-server, note this requires uninstalling diagnostics from Tuner. 
	// c_SetPhoenixDiagnosticsStartTime(-1); // disable diag server, instead we will use the diag server stand alone application that Tuner installs

	/* setup drive */
	initDrive();

	while (true) {
		/* we are looking for gamepad (first time or after disconnect),
			neutral drive until gamepad (re)connected. */
		drive(0, 0);

		sleepApp(100);
	}
	return 0;
}

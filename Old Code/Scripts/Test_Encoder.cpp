// Encoder reading test. Working after the modification was made to display the output only ever 10ms - no more pulses are missed.

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
#include <ctime>
#include <math.h>
#include <fstream>
#include <iostream>		// Include all needed libraries here
#include <wiringPi.h>

#define PI 3.14159265
// Assume you are already in these folder - easier to reference
using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;
using namespace std;

/* make some talons for drive train */
// I  call the motor Joint1.
TalonFX Joint1(1); // My current TalonFX motor which I managed to configure with Phoenix Tuner
TalonFX Joint0(0);
/** simple wrapper for code cleanup */
// void means an empty class: the function does not return anything
void sleepApp(int ms)
{
	std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}


int main() {
	c_SetPhoenixDiagnosticsStartTime(0); // disable diag server, instead we will use the diag server stand alone application that Tuner installs
	// Configuring the motor
	Joint1.ConfigFactoryDefault();
	/* first choose the sensor */
	Joint1.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 10);
	// Joint1.SetSensorPhase(true);
	/* set closed loop gains in slot0 */
	double frequency = 0.4; // frequency of oscilation
	double A = 100; // Amplitude in sensor measurement
				//Using All Configs
				TalonFXConfiguration allConfigs;
				Joint1.GetAllConfigs(allConfigs, 100);
				//PID Config

				SlotConfiguration slot_config;
				slot_config.kP = 0.8*1023/2/A;
				slot_config.kD = 0;
				slot_config.kI = 0;
				slot_config.kF = 0;

				allConfigs.slot0 = slot_config;

				allConfigs.neutralDeadband = 0;
				allConfigs.peakOutputForward = 1;
				allConfigs.peakOutputReverse = -1;

				//Velocity measurement
				allConfigs.velocityMeasurementPeriod = SensorVelocityMeasPeriod::Period_10Ms;
				allConfigs.velocityMeasurementWindow = 8;

				// Motion magic settings
				allConfigs.motionCruiseVelocity = 100;
				allConfigs.motionAcceleration = 100;

				//Sensor
				allConfigs.primaryPID.selectedFeedbackSensor = FeedbackDevice::IntegratedSensor;

				Joint1.ConfigAllSettings(allConfigs, 100);
				
				// Sensor Collection
				TalonFXSensorCollection sensJoint1(Joint1);
				


	/* Zero the sensor */
	Joint1.SetSelectedSensorPosition(0, 0, 10);
	// Choose a slot for magic motion
	Joint1.SelectProfileSlot(0, 0);
	/* don't bother prompting, just use can0 */
	//std::cout << "Please input the name of your can interface: ";
	/*std::string interface;
	interface = "can0";
	ctre::phoenix::platform::can::SetCANInterface(interface.c_str());
	*/
	// End of configs

		wiringPiSetup();			// Setup the library
		pinMode(0, INPUT);		// Configure GPIO0 as an input
		pinMode(2, INPUT);		// Configure GPIO1 as an input
		// Main program loop
		int pin0;
		int pin2;
		int previousPin0;
		double previousDir;
		double counter;
		double rotation;
		double rot_rad;
		double joint_position;
		double direction;
		double loop_time;
		double code_time;
		double joint_position_abs;
		double write_time;
		write_time = 0;
		loop_time = 0;
		code_time = 0;
		previousPin0 = 1;
		previousDir = 1;
		counter = 0;
		
		auto start_code = std::chrono::system_clock::now();
		
		auto start = std::chrono::system_clock::now();
		auto end = std::chrono::system_clock::now();
		
		std::chrono::duration<double> elapsed_seconds = end-start;
		std::chrono::duration<double> elapsed_seconds_code = end-start_code;
		
		start = std::chrono::system_clock::now();
		while (true) {
			
			//Joint1.Set(ControlMode::Position, (target_position));
			pin0 = digitalRead(0);
			pin2 = digitalRead(2);
			if(pin0 != previousPin0){
			  if(pin2 != pin0){
			    counter++;
			    rotation = counter;
			    direction = 1;
			  }
			  else{
			    counter--;
			    rotation = counter;
			    direction = -1;
			  }
			  previousPin0 = pin0;
			  previousDir = direction;
			}
			else{
			  rotation = counter;
			  previousPin0 = pin0;
			  direction = previousDir;
			}
			rot_rad = rotation/1024*2*3.1415926535;
			
			end = std::chrono::system_clock::now();

			elapsed_seconds = end-start;
			write_time = elapsed_seconds.count();
			if (write_time > 0.01){
				joint_position = Joint1.GetSelectedSensorPosition(0);
				joint_position_abs = sensJoint1.GetIntegratedSensorAbsolutePosition();
				write_time = 0;
				elapsed_seconds_code = end - start_code;
				code_time = elapsed_seconds_code.count();
				fprintf(stdout, "Absolute Rotation Motor = %f, Rotation motor = %f, Counter = %f, Loop time = %f \r", joint_position_abs, joint_position, counter, loop_time);	
				start = std::chrono::system_clock::now();
			}
			end = std::chrono::system_clock::now();
			elapsed_seconds = end-start_code;
			loop_time = elapsed_seconds.count();
			
	}

	SDL_Quit();
	return 0;
}

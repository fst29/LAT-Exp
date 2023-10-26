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
	// Comment out the call if you would rather use the automatically running diag-server, note this requires uninstalling diagnostics from Tuner.
	c_SetPhoenixDiagnosticsStartTime(0); // disable diag server, instead we will use the diag server stand alone application that Tuner installs
	sleepApp(3000);
	// Configuring the motor
	Joint1.ConfigFactoryDefault();
	/* first choose the sensor */
	Joint1.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 100);

	Joint0.ConfigFactoryDefault();
	/* first choose the sensor */
	Joint0.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 100);


	// Joint1.SetSensorPhase(true);
	/* set closed loop gains in slot0 */
	double frequency = 0.5; // frequency of oscilation
	double A = 0.1; // Amplitude in sensor measurement
	//Joint 1 - Using All Configs
	TalonFXConfiguration allConfigs;
	Joint1.GetAllConfigs(allConfigs, 100);
	//PID Config

	SlotConfiguration slot_config;
	slot_config.kP = 0.0001;
	slot_config.kD = 0;
	slot_config.kI = 0;
	slot_config.kF = 0.0008;

	allConfigs.slot0 = slot_config;

	allConfigs.neutralDeadband = 0;
	allConfigs.peakOutputForward = 1;
	allConfigs.peakOutputReverse = -1;

	//Velocity measurement
	allConfigs.velocityMeasurementPeriod = VelocityMeasPeriod::Period_10Ms;
	allConfigs.velocityMeasurementWindow = 10;

	// Motion magic settings
	allConfigs.motionCruiseVelocity = 6*A*frequency;
	allConfigs.motionAcceleration = 6*A*frequency*frequency;

	//Sensor
	allConfigs.primaryPID.selectedFeedbackSensor = FeedbackDevice::IntegratedSensor;

	Joint1.ConfigAllSettings(allConfigs, 100);


	/* Zero the sensor */
	Joint1.SetSelectedSensorPosition(0, 0, 100);
	// Choose a slot for magic motion
	Joint1.SelectProfileSlot(0, 0);
	// End of configs

	//Joint 2 - Using All Configs
	TalonFXConfiguration allConfigs0;
	Joint0.GetAllConfigs(allConfigs0, 100);
	//PID Config

	SlotConfiguration slot_config0;
	slot_config0.kP = 0.1*1023/50;
	slot_config0.kD = 0;
	slot_config0.kI = 0;
	slot_config0.kF = 0.5;

	allConfigs0.slot0 = slot_config0;

	allConfigs0.neutralDeadband = 0;
	allConfigs0.peakOutputForward = 1;
	allConfigs0.peakOutputReverse = -1;

	//Velocity measurement
	allConfigs0.velocityMeasurementPeriod = VelocityMeasPeriod::Period_100Ms;
	allConfigs0.velocityMeasurementWindow = 10;

	// Motion magic settings
	allConfigs0.motionCruiseVelocity = 75;
	allConfigs0.motionAcceleration = 75;

	//Sensor
	allConfigs0.primaryPID.selectedFeedbackSensor = FeedbackDevice::IntegratedSensor;

	Joint0.ConfigAllSettings(allConfigs0, 100);


	/* Zero the sensor */
	Joint0.SetSelectedSensorPosition(0, 0, 100);
	// Choose a slot for magic motion
	Joint0.SelectProfileSlot(0, 0);
	// End of configs

	/* don't bother prompting, just use can0 */
	//std::cout << "Please input the name of your can interface: ";
	std::string interface;
	interface = "can0";
	ctre::phoenix::platform::can::SetCANInterface(interface.c_str());

	wiringPiSetup();			// Setup the library
	pinMode(0, INPUT);		// Configure GPIO0 as an input
	pinMode(2, INPUT);		// Configure GPIO1 as an input
	// Main program loop
	int pin0;
	int pin2;
	double pin3;
	double joint_velocity;
	double joint_position;
	double joint_position_p;
	double target_velocity;
	double target_position;
	double target_position_p;
	double loop_time;
	double motion_time;
	double adjust_time;
	double current;
	double current_p;
	double trip;
	double torque;

	double write_time;

	double run_time;

	double rotation;
	double rot_rad;
	double counter;
	double direction;
	double previousDir;
	int previousPin0;

	bool run_pinion;

	bool forward = true;

	counter = 0;
	previousPin0 = 1;
	previousDir = 1;


	loop_time = 0;
	trip = 1;
	write_time = 0;

	ofstream myfile;
	myfile.open ("data.csv");
	myfile << "Time," << "Target Position," << "Measured Position," << "Torque," << "Current," <<"Rotation raw," << "Pinion Target," << "Pinion Position," << "Pinion Current\n";

	sleepApp(1000);

	auto start = std::chrono::system_clock::now();
	auto start_code = std::chrono::system_clock::now();
	auto start_motion = std::chrono::system_clock::now();

	auto end = std::chrono::system_clock::now();

	std::chrono::duration<double> elapsed_seconds = end-start;

	auto write_inst = std::chrono::system_clock::now();
	std::chrono::duration<double> length_run = write_inst-start_code;

	start = std::chrono::system_clock::now();
	while (true) {

		//Joint1.Set(ControlMode::Position, (target_position));
		pin0 = digitalRead(0);
		pin2 = digitalRead(2);
		if(pin0 != previousPin0){
			if(pin2 != pin0){
				counter++;
				direction = 1;
			}
			else{
				counter--;
				direction = -1;
			}
			previousPin0 = pin0;
			previousDir = direction;
		}
		else{
			previousPin0 = pin0;
			direction = previousDir;
		}
		end = std::chrono::system_clock::now();
		elapsed_seconds = end-start;
		write_time = elapsed_seconds.count();

		elapsed_seconds = end-start_code;
		loop_time = elapsed_seconds.count();

		if (write_time > 0.01){
			ctre::phoenix::unmanaged::FeedEnable(100);
			// Calibration phase
			if (loop_time < 1){
				target_position_p = 0;
				target_position = A;
			}
			else if(loop_time >=1 && loop_time<3){
				target_position_p = 0;
				target_position = -A;
			}
			else if(loop_time >=3 && loop_time<4){
				target_position_p = 0;
				target_position = A;
			}
			else if(loop_time >=4 && loop_time<6){
				target_position_p = 0;
				target_position = 0;
			}
			else if (loop_time >=6 && loop_time<7){
				target_position_p = 0;
				target_position = A;
			}
			else if(loop_time >=7 && loop_time<8){
				target_position_p = 0;
				target_position = -A;
			}
			else if(loop_time >=8 && loop_time<9){
				target_position_p = 0;
				target_position = A;
			}
			else if(loop_time >=9 && loop_time<11){
				target_position_p = 0;
				target_position = 0;
			}
			Joint1.Set(ControlMode::Current, (target_position));
			Joint0.Set(ControlMode::MotionMagic, (target_position_p));
			joint_position_p = Joint0.GetSelectedSensorPosition(0);
			joint_position = Joint1.GetSelectedSensorPosition(0);

			current = Joint1.GetOutputCurrent();
			current_p = Joint0.GetOutputCurrent();
			torque = current*4.69/257;

			write_inst = std::chrono::system_clock::now();
			length_run = write_inst-start_code;
			run_time = length_run.count();
			//fprintf(stdout, "Pinion target = %f, Pinion current = %f\r", target_position_p, current_p);
			myfile << run_time << "," << target_position << "," << joint_position << "," << torque << "," << current << "," << counter << "," << target_position_p << "," << joint_position_p << "," << current_p << "\n";
			write_time = 0;
			start = std::chrono::system_clock::now();
		}
		end = std::chrono::system_clock::now();
		elapsed_seconds = end-start_code;
		loop_time = elapsed_seconds.count();
		if (loop_time > 11){
			break;}
		}

		SDL_Quit();
		return 0;
	}
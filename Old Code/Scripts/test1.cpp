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
TalonFX Joint1(1); // Carrriage motor
TalonFX Joint0(0); // Driving Motor

// Control modes
// velocity - velocity control mode
// MotionMagic - position Control
// Current - torque control
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


	/* set closed loop gains in slot0 */
	double frequency = 0.7; // frequency of oscilation
	double frequencyB = 0.2; //frequency of oscillation of carriage
	double A = 125; // Amplitude of driving shaft velocity
	double B = 200; // Amplitude in carriage motion
	//Joint 1 - Using All Configs
	TalonFXConfiguration allConfigs;
	Joint1.GetAllConfigs(allConfigs, 100);
	//PID Config

	SlotConfiguration slot_config;
	slot_config.kP = 0.7; //0.75
	slot_config.kD = 0;
	slot_config.kI = 0;
	slot_config.kF = 0.7; //0.75

	allConfigs.slot0 = slot_config;

	allConfigs.neutralDeadband = 0;
	allConfigs.peakOutputForward = 1;
	allConfigs.peakOutputReverse = -1;

	//Velocity measurement
	allConfigs.velocityMeasurementPeriod = SensorVelocityMeasPeriod::Period_10Ms;
	allConfigs.velocityMeasurementWindow = 4;

	// Motion magic settings
	allConfigs.motionCruiseVelocity = 400;
	allConfigs.motionAcceleration = 400;

	//Sensor
	allConfigs.primaryPID.selectedFeedbackSensor = FeedbackDevice::IntegratedSensor;

	Joint1.ConfigAllSettings(allConfigs, 100);

	// Joint1.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);
	//Joint1.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 10);
	Joint1.SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0 , 10, 10); // sample feedback signals at 10ms
	//Joint1.SetStatusFramePeriod(StatusFrameEnhanced::Status_Brushless_Current , 10, 10); // sample current at 10ms

	Joint1.SetNeutralMode(NeutralMode::Brake);

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
	allConfigs0.velocityMeasurementPeriod = SensorVelocityMeasPeriod::Period_10Ms;
	allConfigs0.velocityMeasurementWindow = 8;

	// Motion magic settings
	allConfigs0.motionCruiseVelocity = 300;
	allConfigs0.motionAcceleration = 300;
	allConfigs0.motionCurveStrength = 4;

	//Sensor
	allConfigs0.primaryPID.selectedFeedbackSensor = FeedbackDevice::IntegratedSensor;

	Joint0.ConfigAllSettings(allConfigs0, 100);


	/* Zero the sensor */
	Joint0.SetSelectedSensorPosition(0, 0, 100);
	// Choose a slot for magic motion
	Joint0.SelectProfileSlot(0, 0);
	// End of configs


	wiringPiSetup();			// Setup the library to use raspberry pi pins for encoder measurement
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

	double currentStat;
	double percentOutput;

	char *sysClock;

	bool forward = true;
	double tCal = 4;

	counter = 0;
	previousPin0 = 1;
	previousDir = 1;


	loop_time = 0;
	trip = 1;
	write_time = 0;
// opening the data.csv file
	ofstream myfile;
	myfile.open ("data.csv");
	myfile << "Time," << "Target Position," << "Measured Position," << "Torque," << "Current," <<"Rotation raw," << "Pinion Target," << "Pinion Position," << "Pinion Current," << "Joint Velocity," << "Stator Current," << "Percent Output," << "System Clock"<<",,,,,,,\n";

	sleepApp(1000);
// starting time counting for various intervals
	auto start = std::chrono::system_clock::now();
	auto start_code = std::chrono::system_clock::now();
	auto start_motion = std::chrono::system_clock::now();

	auto end = std::chrono::system_clock::now();

	std::chrono::duration<double> elapsed_seconds = end-start;

	auto write_inst = std::chrono::system_clock::now();
	std::chrono::duration<double> length_run = write_inst-start_code;

	start = std::chrono::system_clock::now();
	while (true) { // loop running continuously

		//processing encoder data - Start
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
		//processing encoder data - End
		end = std::chrono::system_clock::now();
		elapsed_seconds = end-start;
		write_time = elapsed_seconds.count();

		elapsed_seconds = end-start_code;
		loop_time = elapsed_seconds.count();

		if (write_time > 0.01){ //doing postprocessing every 10ms to save computational power
			ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100);
			// Calibration phase - sinusoidal motion to get the p value afterwards
			if (loop_time < tCal){
				target_position_p = 0;
				target_position = A*sin(loop_time*PI*2*frequency);
			}
			else if(loop_time >=tCal && loop_time<tCal + 10){ // Main run, simultaneous carriage and cone motion
				target_position_p = B*sin((loop_time-tCal)*PI*2*frequencyB);
				target_position = A*sin((loop_time)*PI*2*frequency);
			}
			else {
				target_position_p = 0;
				target_position = 0;
			}
			Joint1.Set(ControlMode::Velocity, (target_position));
			Joint0.Set(ControlMode::MotionMagic, (target_position_p));
			joint_position_p = Joint0.GetSelectedSensorPosition(0);
			joint_position = Joint1.GetSelectedSensorPosition(0);

			joint_velocity = Joint1.GetSelectedSensorVelocity(0);
			current = Joint1.GetOutputCurrent();
			// current_p = Joint0.GetOutputCurrent();
			current_p = 0;
			torque = current*4.69/257;
			percentOutput = Joint1.GetMotorOutputPercent();
			// currentStat = Joint1.GetStatorCurrent();
			currentStat = 0;

			char *sysClock;
			std::time_t end_time = std::chrono::system_clock::to_time_t(end);
			sysClock = strtok(std::ctime(&end_time), "\n");

			write_inst = std::chrono::system_clock::now();
			length_run = write_inst-start_code;
			run_time = length_run.count();
			//fprintf(stdout, "Pinion target = %f, Pinion current = %f\r", target_position_p, current_p);
			myfile << run_time << "," << target_position << "," << joint_position << "," << torque << "," << current << "," << counter << "," << target_position_p << "," << joint_position_p << "," << current_p << "," << joint_velocity << "," << currentStat << "," << percentOutput << "," << sysClock << ",,,,,,,\n";
			write_time = 0;
			start = std::chrono::system_clock::now();
		}
		end = std::chrono::system_clock::now();
		elapsed_seconds = end-start_code;
		loop_time = elapsed_seconds.count();
		if (loop_time > tCal + 11){//16 // breaking the loop - stopping code after time has passed
			break;}
		}

		SDL_Quit();
		return 0;
	}

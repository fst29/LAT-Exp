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
	double frequency = 0.6; // frequency of oscilation
	double A = 50; // Amplitude in sensor measurement
	//Joint 1 - Using All Configs
	TalonFXConfiguration allConfigs;
	Joint1.GetAllConfigs(allConfigs, 100);
	//PID Config

	// pid for low velocity
	SlotConfiguration slot_config;
	slot_config.kP = 1;
	slot_config.kD = 0;
	slot_config.kI = 0;
	slot_config.kF = 25;

	allConfigs.slot0 = slot_config;

	//pid for high velocity
	SlotConfiguration slot_config2;
	slot_config2.kP = 0.2; // 0.02*1023/100
	slot_config2.kD = 0;
	slot_config2.kI = 0;
	slot_config2.kF = 13; //13

	allConfigs.slot2 = slot_config2;

	//position control gains
	SlotConfiguration slot_config1;
	slot_config1.kP = 5;
	slot_config1.kD = 35;
	slot_config1.kI = 0;
	slot_config1.kF = 0.7;

	allConfigs.slot1 = slot_config1;

	allConfigs.neutralDeadband = 0;
	allConfigs.peakOutputForward = 1;
	allConfigs.peakOutputReverse = -1;

	//Velocity measurement
	allConfigs.velocityMeasurementPeriod = SensorVelocityMeasPeriod::Period_10Ms;
	allConfigs.velocityMeasurementWindow = 8;

	// Motion magic settings
	allConfigs.motionCruiseVelocity = 100;
	allConfigs.motionAcceleration = 100;
	allConfigs.motionCurveStrength = 0;

	//Sensor
	allConfigs.primaryPID.selectedFeedbackSensor = FeedbackDevice::IntegratedSensor;

	Joint1.ConfigAllSettings(allConfigs, 100);

	Joint1.ConfigAllSettings(allConfigs, 100);
	//Joint1.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);
	//Joint1.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 10);
	Joint1.SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0 , 10, 10);
	//Joint1.SetStatusFramePeriod(StatusFrameEnhanced::Status_Brushless_Current , 10, 10);


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
	slot_config0.kP = 0.1*1023/100;
	slot_config0.kD = 0;
	slot_config0.kI = 0;
	slot_config0.kF = 1.0;

	allConfigs0.slot0 = slot_config0;

	allConfigs0.neutralDeadband = 0;
	allConfigs0.peakOutputForward = 1;
	allConfigs0.peakOutputReverse = -1;

	//Velocity measurement
	allConfigs0.velocityMeasurementPeriod = SensorVelocityMeasPeriod::Period_10Ms;
	allConfigs0.velocityMeasurementWindow = 8;

	// Motion magic settings
	allConfigs0.motionCruiseVelocity = 100;
	allConfigs0.motionAcceleration = 100;

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
	// std::string interface;
	// interface = "can0";
	// ctre::phoenix::platform::can::SetCANInterface(interface.c_str());

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
	double target_position = 0;
	double target_position_p;
	double loop_time;
	double motion_time;

	double current;
	double current_p;
	double trip;
	double torque;

	double torque_demand = 0;
	double previousPos;

	double write_time;

	double run_time;

	double wait_time;

	double rotation;
	double rot_rad;
	double counter;
	double direction;
	double previousDir;
	int previousPin0;

	double adjust_time;
	double initial_time;

	double moved;
	double motion_direction;

	double currentStat;

	double staticPos;
	double percentOutput;

	int profileSelect;

	double carriagePos;

	bool run_pinion;
	bool stop = true;

	bool forward = true;

	bool posControl = false;

	bool waiting = false;
	counter = 0;
	previousPin0 = 1;
	previousDir = 1;


	loop_time = 0;
	trip = 1;
	write_time = 0;

	double Range = 150;
	double RangeC = 150;
	int carriageSteps = 5;
	double coneSteps = 5;

	ofstream myfile;
	myfile.open ("data.csv");
	myfile << "Time," << "Target Position," << "Measured Position," << "Torque," << "Current," <<"Rotation raw," << "Pinion Target," << "Pinion Position," << "Pinion Current," << "Torque demand," << "Previous Pos," << "Moved," << "Motion Direction," << "i," << "j," << "Percent Output," << "PosControl, " << "Stator Current," << "System Clock" << "," << "\n";

	sleepApp(1000);

	auto start = std::chrono::system_clock::now();
	auto start_code = std::chrono::system_clock::now();
	auto start_motion = std::chrono::system_clock::now();
	auto start_wait = std::chrono::system_clock::now();
	auto start_initial = std::chrono::system_clock::now();

	auto end = std::chrono::system_clock::now();

	std::chrono::duration<double> elapsed_seconds = end-start;

	auto write_inst = std::chrono::system_clock::now();
	std::chrono::duration<double> length_run = write_inst-start_code;


	double carriage_pos_vec[carriageSteps] = {0, RangeC, 2*RangeC, 3*RangeC, 4*RangeC};
	double nSteps[carriageSteps] = {coneSteps, coneSteps, coneSteps, coneSteps, coneSteps};
	start = std::chrono::system_clock::now();
	for (int j = 0; j < carriageSteps; j++) {
		// positive torques
		carriagePos = carriage_pos_vec[j];
		start_initial = std::chrono::system_clock::now();
		for (int i = 0; i < nSteps[j]+1; i++) {
			//fprintf(stdout, "Carriage id = %u, Rotation id = %u, previous torque demand = %f\r", j, i, torque_demand);
			adjust_time = 0;
			//target_position_p = 200*j;
			//Joint0.Set(ControlMode::MotionMagic, (target_position_p));
			//target_position = 2+0.75*i;
			target_position = 3.5+0.75*i;
			torque_demand = 0.08/4.69*257;
			start_motion = std::chrono::system_clock::now();
			stop = true;
			posControl = true;
			/* if (i < 2){
				profileSelect = 0;
			}
			else {
				profileSelect = 2;
			} */
			profileSelect = 2;
			Joint1.SelectProfileSlot(1, 0);
			Joint1.Set(ControlMode::MotionMagic, (0));
			previousPos = Joint1.GetSelectedSensorPosition(0);
			motion_direction = 1;
			while (stop) {
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

				elapsed_seconds = end-start_motion;
				adjust_time = elapsed_seconds.count();

				elapsed_seconds = end-start_code;
				loop_time = elapsed_seconds.count();

				elapsed_seconds = end-start_initial;
				initial_time = elapsed_seconds.count();

				elapsed_seconds = end - start_wait;
				wait_time = elapsed_seconds.count();

				if (write_time > 0.01){
					ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100);
					if (adjust_time < 1.5 && initial_time > 3){
						Joint1.SelectProfileSlot(1, 0);
						Joint1.Set(ControlMode::MotionMagic, (0));
						previousPos = Joint1.GetSelectedSensorPosition(0);
						moved = 0;
						staticPos = Joint1.GetSelectedSensorPosition(0);
						posControl = true;
					}
					else {
						if ((j!=0) && initial_time < 2){
							target_position_p = RangeC/2*initial_time + carriage_pos_vec[j-1];;
							Joint0.Set(ControlMode::MotionMagic, (target_position_p));
							Joint1.SelectProfileSlot(1, 0);
							//target_position = 100*sin(initial_time*PI*2*0.5);
							target_position = 0;
							Joint1.Set(ControlMode::MotionMagic, (target_position));
							start_motion = std::chrono::system_clock::now();
						}
						else if ((j==0) && initial_time < 2){
							target_position_p = 0;
							Joint0.Set(ControlMode::MotionMagic, (target_position_p));
							Joint1.SelectProfileSlot(1, 0);
							target_position = 100*sin(initial_time*PI*2*0.5);
							//target_position = 0;
							Joint1.Set(ControlMode::MotionMagic, (target_position));
							start_motion = std::chrono::system_clock::now();
						}
						else if (initial_time >= 2 && initial_time <= 3) {
							target_position_p = carriagePos;
							Joint0.Set(ControlMode::MotionMagic, (target_position_p));
							Joint1.SelectProfileSlot(1, 0);
							target_position = 0;
							Joint1.Set(ControlMode::MotionMagic, (target_position));
							start_motion = std::chrono::system_clock::now();
							previousPos = Joint1.GetSelectedSensorPosition(0);
							torque_demand = 0;
						}

					else {
						joint_position = Joint1.GetSelectedSensorPosition(0);
						if (joint_position>750 && motion_direction == 1){
							moved = 1;
							stop = true;
							waiting = true;
							motion_direction = -1;
							torque_demand = 0;
							//torque_demand = motion_direction*target_position;
							Joint1.SelectProfileSlot(1, 0);
							Joint1.Set(ControlMode::MotionMagic, (750));
							posControl = true;
							start_wait = std::chrono::system_clock::now();
						}
						else if (waiting && wait_time < 1.5){
							moved = 0;
							stop = true;
							waiting = true;
							torque_demand = 0;
							Joint1.SelectProfileSlot(1, 0);
							posControl = true;
							Joint1.Set(ControlMode::MotionMagic, (750));
						}
						else if (waiting && wait_time >=1.5){
							moved = 0;
							stop = true;
							waiting = false;
							motion_direction = -1;
							Joint1.SelectProfileSlot(profileSelect, 0);
							torque_demand = motion_direction*target_position;
							posControl = false;
							Joint1.Set(ControlMode::Velocity, (torque_demand));
						}
						else if (joint_position<0 && motion_direction == -1){
							moved = 0;
							stop = false;
							waiting = false;
							motion_direction = 1;
							torque_demand = 0;
							Joint1.SelectProfileSlot(1, 0);
							posControl = true;
							Joint1.Set(ControlMode::MotionMagic, (0));
						}
						else {
							target_position = 3.5+0.75*i;
							torque_demand = motion_direction*target_position;
							Joint1.SelectProfileSlot(profileSelect, 0);
							posControl = false;
							Joint1.Set(ControlMode::Velocity, (torque_demand));
						}
					}
					}
						joint_position_p = Joint0.GetSelectedSensorPosition(0);
						joint_position = Joint1.GetSelectedSensorPosition(0);

					current = Joint1.GetOutputCurrent();
					currentStat = Joint1.GetStatorCurrent();
					current_p = Joint0.GetOutputCurrent();
					torque = current*4.69/257;
					percentOutput = Joint1.GetMotorOutputPercent();

					write_inst = std::chrono::system_clock::now();
					length_run = write_inst-start_code;
					run_time = length_run.count();

					char *sysClock;
					std::time_t end_time = std::chrono::system_clock::to_time_t(end);
					sysClock = strtok(std::ctime(&end_time), "\n");
					//fprintf(stdout, "Pinion target = %f, Pinion current = %f\r", target_position_p, current_p);
					myfile << run_time << "," << target_position << "," << joint_position << "," << torque << "," << current << "," << counter << "," << target_position_p << "," << joint_position_p << "," << current_p << "," << torque_demand << "," << staticPos <<  "," << motion_direction << "," << i << "," << j << "," << moved << "," << percentOutput << "," << posControl << "," << currentStat << "," << sysClock << "," << 0 << ",\n";
					write_time = 0;
					start = std::chrono::system_clock::now();
					end = std::chrono::system_clock::now();
					elapsed_seconds = end-start_code;
					loop_time = elapsed_seconds.count();
				}
			}
		}
	}

	SDL_Quit();
	return 0;
}

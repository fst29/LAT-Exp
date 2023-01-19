#define Phoenix_No_WPI // remove WPI dependencies
#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include "ctre/phoenix/cci/Unmanaged_CCI.h"
#include <ctre/phoenix/motorcontrol/StatorCurrentLimitConfiguration.h>
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

	SlotConfiguration slot_config;
	slot_config.kP = 5;
	slot_config.kD = 35;
	slot_config.kI = 0;
	slot_config.kF = 0.7;

	allConfigs.slot0 = slot_config;

	SlotConfiguration slot_config1;
	slot_config1.kP = 0.0035;
	slot_config1.kD = 0;
	slot_config1.kI = 0;
	slot_config1.kF = 0.0035;

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
	//Joint1.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);
	//Joint1.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 10);
	Joint1.SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0 , 10, 10);
	//Joint1.SetStatusFramePeriod(StatusFrameEnhanced::Status_Brushless_Current , 10, 10);

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
	slot_config0.kP = 0.3*1023/100;
	slot_config0.kD = 0;
	slot_config0.kI = 0;
	slot_config0.kF = 1;

	allConfigs0.slot0 = slot_config0;

	allConfigs0.neutralDeadband = 0;
	allConfigs0.peakOutputForward = 1;
	allConfigs0.peakOutputReverse = -1;

	//Velocity measurement
	//allConfigs0.velocityMeasurementPeriod = VelocityMeasPeriod::Period_10Ms;
	//allConfigs0.velocityMeasurementWindow = 10;

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

	/* 	std::string interface;
	interface = "can0";
	ctre::phoenix::platform::can::SetCANInterface(interface.c_str());
	*/
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
	double currentStat;

	double trip;
	double torque;

	double torque_demand = 0;
	double previousPos;

	double write_time;

	double run_time;

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

	double staticPos;
	double percentOutput;

	double carriagePos;

	bool run_pinion;
	bool stop = true;

	bool forward = true;
	bool posControl = true;

	double Range = 150;
	double RangeC = 150;
	int carriageSteps = 4;
	double coneSteps = 8;
	int tol = 2;
	double dTorque = 1.5;
	double dTorqueDown = 3;


	char *sysClock;
	// double sysClock = 0;

	counter = 0;
	previousPin0 = 1;
	previousDir = 1;


	loop_time = 0;
	trip = 1;
	write_time = 0;

	ofstream myfile;
	myfile.open ("data.csv");
	myfile << "Time," << "Target Position," << "Measured Position," << "Torque," << "Current," <<"Rotation raw," << "Pinion Target," << "Pinion Position," << "Pinion Current," << "Torque demand," << "Previous Pos," << "Moved," << "Motion Direction," << "i," << "j,"<< "Percent Output, " << "Pos Control, " << "Stator Current," << "System Clock\n";

	sleepApp(1000);

	auto start = std::chrono::system_clock::now();
	auto start_code = std::chrono::system_clock::now();
	auto start_motion = std::chrono::system_clock::now();
	auto start_initial = std::chrono::system_clock::now();

	auto end = std::chrono::system_clock::now();

	std::chrono::duration<double> elapsed_seconds = end-start;

	auto write_inst = std::chrono::system_clock::now();
	std::chrono::duration<double> length_run = write_inst-start_code;

	start = std::chrono::system_clock::now();
	double carriage_pos_vec[carriageSteps] = {0, RangeC, 2*RangeC, 3*RangeC};
	double nSteps[carriageSteps] = {coneSteps, coneSteps, coneSteps, coneSteps};
	for (int j = 0; j < carriageSteps; j++) {
		carriagePos = carriage_pos_vec[j];
		start_initial = std::chrono::system_clock::now();
		// positive torques - positive cone positions
		for (int i = 0; i < nSteps[j]+1; i++) {
			// fprintf(stdout, "Carriage id = %u, Rotation id = %u, previous torque demand = %f\r", j, i, torque_demand);
			adjust_time = 0;
			// Joint0.Set(ControlMode::MotionMagic, (target_position_p));
			target_position = Range/nSteps[j]*i;
			start_motion = std::chrono::system_clock::now();
			stop = true;
			Joint1.SelectProfileSlot(0, 0);
			Joint1.Set(ControlMode::MotionMagic, (target_position));
			torque_demand = Joint1.GetOutputCurrent();
			previousPos = Joint1.GetSelectedSensorPosition(0);
			moved = 0;
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

				elapsed_seconds = end-start_initial;
				initial_time = elapsed_seconds.count();

				elapsed_seconds = end-start_code;
				loop_time = elapsed_seconds.count();

				if (write_time > 0.01){
					ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100);
					if (adjust_time < 1.5 && initial_time > 3){
						Joint1.SelectProfileSlot(0, 0);
						Joint1.Set(ControlMode::MotionMagic, (target_position));
						previousPos = Joint1.GetSelectedSensorPosition(0);
						moved = 0;
						staticPos = Joint1.GetSelectedSensorPosition(0);
						posControl = true;
						torque_demand = Joint1.GetOutputCurrent() - dTorque;
					}
					else {
						if ((j!=0) && initial_time < 2){
							target_position_p = RangeC/2*initial_time + carriage_pos_vec[j-1];;
							Joint0.Set(ControlMode::MotionMagic, (target_position_p));
							Joint1.SelectProfileSlot(0, 0);
							target_position = 0;
							Joint1.Set(ControlMode::MotionMagic, (target_position));
							start_motion = std::chrono::system_clock::now();
						}
						else if ((j==0) && initial_time < 2){
							target_position_p = 0;
							Joint0.Set(ControlMode::MotionMagic, (target_position_p));
							Joint1.SelectProfileSlot(0, 0);
							target_position = 100*sin(initial_time*PI*2*0.5);
							Joint1.Set(ControlMode::MotionMagic, (target_position));
							start_motion = std::chrono::system_clock::now();
						}
						else if (initial_time >= 2 && initial_time <= 3) {
							target_position_p = carriagePos;
							Joint0.Set(ControlMode::MotionMagic, (target_position_p));
							Joint1.SelectProfileSlot(0, 0);
							target_position = 0;
							Joint1.Set(ControlMode::MotionMagic, (target_position));
							start_motion = std::chrono::system_clock::now();
							previousPos = Joint1.GetSelectedSensorPosition(0);
							torque_demand = Joint1.GetOutputCurrent() - dTorque;
						}
						else {
							joint_position = Joint1.GetSelectedSensorPosition(0);
							posControl = false;
							motion_direction = 1;
							if (abs(staticPos-joint_position)>tol){
								stop = false;
								moved = 1;
							}
							else {
								torque_demand = torque_demand + 0.0005/4.69*257;
								Joint1.SelectProfileSlot(1, 0);
								Joint1.Set(ControlMode::Current, (torque_demand));
								previousPos = Joint1.GetSelectedSensorPosition(0);
								stop = true;
								moved = 0;
								motion_direction = 1;
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
					// sysClock = 0;

					//fprintf(stdout, "Pinion target = %f, Pinion current = %f\r", target_position_p, current_p);
					myfile << run_time << "," << target_position << "," << joint_position << "," << torque << "," << current << "," << counter << "," << target_position_p << "," << joint_position_p << "," << current_p << "," << torque_demand << "," << staticPos << "," <<moved << "," << motion_direction << "," << i << "," << j << "," << percentOutput << "," << posControl << "," << currentStat << "," << sysClock << "," << 0 << ",\n";
					write_time = 0;
					start = std::chrono::system_clock::now();
				}
				end = std::chrono::system_clock::now();
				elapsed_seconds = end-start_code;
				loop_time = elapsed_seconds.count();
			}
		}

		// Negative torques - entire range of cone positions
		for (int i = nSteps[j]; i >= -nSteps[j]; i--) {
			//fprintf(stdout, "Carriage id = %u, Rotation id = %u, previous torque demand = %f\r", j, i, torque_demand);
			// cout << "Carriage id: " << j << ", Rotation id" << i << ", Previous Torque demand: " << torque_demand << "\r";
			adjust_time = 0;
			target_position = Range/nSteps[j]*i;
			start_motion = std::chrono::system_clock::now();
			Joint1.SelectProfileSlot(0, 0);
			Joint1.Set(ControlMode::MotionMagic, (target_position));
			if (i > 0){
				torque_demand = Joint1.GetOutputCurrent();
			}
			else {
				torque_demand = -Joint1.GetOutputCurrent();
			}
			previousPos = Joint1.GetSelectedSensorPosition(0);
			moved = 0;
			stop = true;
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

				if (write_time > 0.01){
					ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100);
					if (adjust_time < 1.5){
						Joint1.SelectProfileSlot(0, 0);
						Joint1.Set(ControlMode::MotionMagic, (target_position));
						previousPos = Joint1.GetSelectedSensorPosition(0);
						moved = 0;
						staticPos = Joint1.GetSelectedSensorPosition(0);
						posControl = true;
						if (i > 0){
							torque_demand = Joint1.GetOutputCurrent()-dTorqueDown;
						}
						else {
							torque_demand = -Joint1.GetOutputCurrent()+dTorque;
						}
					}
					else {
						joint_position = Joint1.GetSelectedSensorPosition(0);
						posControl = false;
						motion_direction = -1;
						if (abs(staticPos-joint_position)>tol){
							stop = false;
							moved = 1;
						}
						else {
							torque_demand = torque_demand - 0.0005/4.69*257;
							Joint1.SelectProfileSlot(1, 0);
							Joint1.Set(ControlMode::Current, (torque_demand));
							previousPos = Joint1.GetSelectedSensorPosition(0);
							stop = true;
							moved = 0;
							motion_direction = -1;
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

					std::time_t end_time = std::chrono::system_clock::to_time_t(end);
					char *sysClock;
					sysClock = strtok(std::ctime(&end_time), "\n");
					// sysClock = 0;

					//fprintf(stdout, "Pinion target = %f, Pinion current = %f\r", target_position_p, current_p);
					myfile << run_time << "," << target_position << "," << joint_position << "," << torque << "," << current << "," << counter << "," << target_position_p << "," << joint_position_p << "," << current_p << "," << torque_demand << "," << staticPos << "," << moved << "," << motion_direction  << "," << i << "," << j << "," << percentOutput << "," << posControl << "," << currentStat << ",'"  << sysClock << "'," << 0 << ",\n";
					write_time = 0;
					start = std::chrono::system_clock::now();
					end = std::chrono::system_clock::now();
					elapsed_seconds = end-start_code;
					loop_time = elapsed_seconds.count();
				}
			}
		}

		for (int i = -nSteps[j]; i <= 0; i++) {
			//fprintf(stdout, "Carriage id = %u, Rotation id = %u, previous torque demand = %f\r", j, i, torque_demand);
			// cout << "Carriage id: " << j << ", Rotation id" << i << ", Previous Torque demand: " << torque_demand << "\r";
			adjust_time = 0;
			target_position = Range/nSteps[j]*i;
			start_motion = std::chrono::system_clock::now();
			Joint1.SelectProfileSlot(0, 0);
			Joint1.Set(ControlMode::MotionMagic, (target_position));
			torque_demand = -Joint1.GetOutputCurrent();
			previousPos = Joint1.GetSelectedSensorPosition(0);
			moved = 0;
			stop = true;
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

				if (write_time > 0.01){
					ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100);
					if (adjust_time < 1.5){
						Joint1.SelectProfileSlot(0, 0);
						Joint1.Set(ControlMode::MotionMagic, (target_position));
						previousPos = Joint1.GetSelectedSensorPosition(0);
						moved = 0;
						staticPos = Joint1.GetSelectedSensorPosition(0);
						posControl = true;
						torque_demand = -Joint1.GetOutputCurrent()+dTorqueDown;
					}
					else {
						joint_position = Joint1.GetSelectedSensorPosition(0);
						posControl = false;
						motion_direction = 1;
						if (abs(staticPos-joint_position)>tol){
							stop = false;
							moved = 1;
						}
						else {
							torque_demand = torque_demand + 0.0005/4.69*257;
							Joint1.SelectProfileSlot(1, 0);
							Joint1.Set(ControlMode::Current, (torque_demand));
							previousPos = Joint1.GetSelectedSensorPosition(0);
							stop = true;
							moved = 0;
							motion_direction = 1;
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

					std::time_t end_time = std::chrono::system_clock::to_time_t(end);
					char *sysClock;
					sysClock = strtok(std::ctime(&end_time), "\n");
					// sysClock = 0;

					//fprintf(stdout, "Pinion target = %f, Pinion current = %f\r", target_position_p, current_p);
					myfile << run_time << "," << target_position << "," << joint_position << "," << torque << "," << current << "," << counter << "," << target_position_p << "," << joint_position_p << "," << current_p << "," << torque_demand << "," << staticPos << "," << moved << "," << motion_direction  << "," << i << "," << j << "," << percentOutput << "," << posControl << "," << currentStat << ",'"  << sysClock << "'," << 0 << ",\n";
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

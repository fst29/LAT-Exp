#include <stdlib.h>
#include <fstream>
#include <string>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <thread>
#include <chrono>
#include <time.h>
#define Phoenix_No_WPI // remove WPI dependencies
#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include "ctre/phoenix/cci/Unmanaged_CCI.h"
#include "math.h"

#include <wiringPi.h>

#define PI 3.14159265

using namespace std;

string commandPipePath = "/home/pi/fst29/commands";
string measurementPipePath = "/home/pi/fst29/measurements";
string outputDataPath = "/home/pi/fst29/data";

// The pin numbers of the encoder outputs (using wiringPi convention)
int encoder_A_pin_number = 0;
int encoder_B_pin_number = 2;
// Cycles per rotation of the encoders
int motor_encoder_cpr = 2048;
int output_encoder_cpr = 1024 * 4; // encoder cpr * gear ratio

double drive_loop_frequency = 050;		 // hz
double measurement_loop_frequency = 100; // hz

double default_current = 8;		   // The drive current used during initialisation
double current_step = 0.05;		   // The step in current when measuring static friction
int output_encoder_state = 0b0000; // upper two bits represent the previous state, lower two bits represent the current state, each value corresponds to a transition
float state_transition_matrix[16] = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0};

unsigned long long current_time = 0;
unsigned long long previous_time = 0;
unsigned long long current_time_ms = 0;
double target_position = 0;

double measurement_dt = (current_time - previous_time);

string raw_command = "";
string command = "";
string state = "";

std::fstream data_file;

double command_value[4] = {0, 0, 0, 0};

string message = "";
struct motorMeasurements
{
	float position; // in degrees
	float velocity; // in degrees per second
	float current;
	float previous_position;
};

struct
{
	motorMeasurements carriage;
	motorMeasurements drive;
	motorMeasurements output;
	double p;
} measurements;

ctre::phoenix::motorcontrol::can::TalonFX carriage_motor(0); // Carrriage motor
ctre::phoenix::motorcontrol::can::TalonFX drive_motor(1);	 // drive Motor

unsigned long long get_current_time_ms()
{
	return (double)std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

std::string create_file()
{

	time_t t = time(0); // current time
	struct tm *now = localtime(&t);
	char datetime[80] = "";
	strftime(datetime, 80, "%F-%H-%M-%S", now);

	string filename = outputDataPath + "/" + datetime + ".csv";

	data_file.open(filename, std::ios::out);
	data_file << "Time,"
			  << "Command,"
			  << "State,";
	data_file << "Drive position,"
			  << "Drive velocity,"
			  << "Drive current,";
	data_file << "Carriage position,"
			  << "Carriage velocity,"
			  << "Carriage current,";
	data_file << "Output_position,"
			  << "Output_velocity" << std::endl;
	data_file.close();

	cout << "Output file created: " << filename << std::endl;
	return filename;
}

void write_to_file(string filename)
{
	data_file.open(filename, std::ios::app);

	time_t t = std::time(0);
	struct tm *now = localtime(&t);
	char time_string[80] = "";
	strftime(time_string, 80, "%T", now);

	data_file << time_string << ","
			  << raw_command << ","
			  << state << ",";

	data_file << measurements.drive.position << ",";
	data_file << measurements.drive.previous_position << ",";
	data_file << measurements.drive.velocity << ",";
	data_file << measurements.drive.current << ",";

	data_file << measurements.carriage.position << ",";
	data_file << measurements.carriage.velocity << ",";
	data_file << measurements.carriage.current << ",";

	data_file << measurements.output.position << ",";
	data_file << measurements.output.previous_position << ",";
	data_file << measurements.output.velocity << endl;

	data_file.close();
}

std::string get_command(string raw)
{
	string command = "";
	char delimiter = ' ';

	for (int i = 0; i < raw.length(); i++)
	{
		if (raw[i] != delimiter)
			command += raw[i];
		else
			break;
	}
	return command;
}

void get_single_value(string raw)
{

	string command = "";
	char delimiter = ' ';

	bool at_value = 0;
	for (int i = 0; i < raw.length(); i++)
	{
		if (raw[i] != delimiter)
		{
			if (at_value)
				command += raw[i];
		}
		else
			at_value = 1;
	}

	command_value[0] = stod(command);
	return;
}

void get_command_value(string raw)
{

	string values[4] = {"", "", "", ""};

	int current_value = -1;
	char delimiter = ' ';

	for (int i = 0; i < raw.length(); i++)
	{
		if (raw[i] == delimiter)
			current_value++;
		else if (current_value >= 0)
		{
			values[current_value] += raw[i];
		}
	}

	for (current_value; current_value >= 0; current_value--)
	{
		command_value[current_value] = stod(values[current_value]);
	}
}

void read_commands()
{
	while (1)
	{
		std::ifstream commandPipe;
		commandPipe.open(commandPipePath, ifstream::in);
		if (!commandPipe.is_open())
		{
			std::cout << " error : cannot open commandPipe" << std ::endl;
			return;
		}
		std::getline(commandPipe, raw_command);
		commandPipe.close();
		cout << "Command received: " << raw_command << endl;

		command = get_command(raw_command);

		get_command_value(raw_command);
	}
}

void setup_motors()
{

	c_SetPhoenixDiagnosticsStartTime(0);
	sleep(3);

	drive_motor.ConfigFactoryDefault();
	drive_motor.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 100);
	carriage_motor.ConfigFactoryDefault();
	carriage_motor.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 100);

	// Joint 1 - Using All Configs
	ctre::phoenix::motorcontrol::can::TalonFXConfiguration allConfigs;
	drive_motor.GetAllConfigs(allConfigs, 100);
	// PID Config

	ctre::phoenix::motorcontrol::can::SlotConfiguration slot_config;
	slot_config.kP = 0.7; // 0.75
	slot_config.kD = 0;
	slot_config.kI = 0.005;
	slot_config.kF = 0.7; // 0.75

	allConfigs.slot0 = slot_config;

	allConfigs.neutralDeadband = 0;
	allConfigs.peakOutputForward = 1;
	allConfigs.peakOutputReverse = -1;

	// Velocity measurement
	allConfigs.velocityMeasurementPeriod = SensorVelocityMeasPeriod::Period_10Ms;
	allConfigs.velocityMeasurementWindow = 4;

	// Motion magic settings
	allConfigs.motionCruiseVelocity = 400;
	allConfigs.motionAcceleration = 100;

	// Sensor
	allConfigs.primaryPID.selectedFeedbackSensor = FeedbackDevice::IntegratedSensor;

	drive_motor.ConfigAllSettings(allConfigs, 100);

	// carriage_motor.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);
	// carriage_motor.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 10);
	drive_motor.SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, 10, 10); // sample feedback signals at 10ms
	// carriage_motor.SetStatusFramePeriod(StatusFrameEnhanced::Status_Brushless_Current , 10, 10); // sample current at 10ms

	drive_motor.SetNeutralMode(NeutralMode::Brake);

	/* Zero the sensor */
	drive_motor.SetSelectedSensorPosition(0, 0, 100);
	// Choose a slot for magic motion
	drive_motor.SelectProfileSlot(0, 0);
	// End of configs

	// Joint 2 - Using All Configs
	ctre::phoenix::motorcontrol::can::TalonFXConfiguration allConfigs0;
	carriage_motor.GetAllConfigs(allConfigs0, 100);
	// PID Config

	ctre::phoenix::motorcontrol::can::SlotConfiguration slot_config0;
	slot_config0.kP = 0.1 * 1023 / 50;
	slot_config0.kD = 0;
	slot_config0.kI = 0;
	slot_config0.kF = 0.5;

	allConfigs0.slot0 = slot_config0;

	allConfigs0.neutralDeadband = 0;
	allConfigs0.peakOutputForward = 1;
	allConfigs0.peakOutputReverse = -1;

	// Velocity measurement
	allConfigs0.velocityMeasurementPeriod = SensorVelocityMeasPeriod::Period_10Ms;
	allConfigs0.velocityMeasurementWindow = 8;

	// Motion magic settings
	allConfigs0.motionCruiseVelocity = 300;
	allConfigs0.motionAcceleration = 100;
	allConfigs0.motionCurveStrength = 4;

	// Sensor
	allConfigs0.primaryPID.selectedFeedbackSensor = FeedbackDevice::IntegratedSensor;

	carriage_motor.ConfigAllSettings(allConfigs0, 100);

	/* Zero the sensor */
	carriage_motor.SetSelectedSensorPosition(0, 0, 100);
	// Choose a slot for magic motion
	carriage_motor.SelectProfileSlot(0, 0);
	// End of configs

	cout << "Motor setup done" << endl;
}

void callback(void)
{
	output_encoder_state = ((output_encoder_state << 2) & 0x0F) + (digitalRead(encoder_A_pin_number) << 1) + digitalRead(encoder_B_pin_number);
	measurements.output.position += state_transition_matrix[output_encoder_state];
}

void setup_output_encoder()
{
	for (int i = 0; i < 16; i++)
	{
		state_transition_matrix[i] = 360 * state_transition_matrix[i] / output_encoder_cpr; // convert encoder ticks to degrees
	}
	wiringPiSetup();

	// Initialise the encoder pins
	pinMode(encoder_A_pin_number, INPUT);
	pinMode(encoder_B_pin_number, INPUT);

	output_encoder_state = (digitalRead(encoder_A_pin_number) << 1) + digitalRead(encoder_B_pin_number);
	// Attach interrupts to pin state changes
	wiringPiISR(encoder_A_pin_number, INT_EDGE_BOTH, callback);
	wiringPiISR(encoder_B_pin_number, INT_EDGE_BOTH, callback);

	cout << "Output encoder setup done" << endl;
}

string create_measurement_message()
{
	return ("CARRIAGE_POSITION " + to_string(measurements.carriage.position) + " CARRIAGE_VELOCITY " + to_string(measurements.carriage.velocity) + " CARRIAGE_CURRENT " + to_string(measurements.carriage.current) + " DRIVE_POSITION " + to_string(measurements.drive.position) + " DRIVE_VELOCITY " + to_string(measurements.drive.velocity) + " DRIVE_CURRENT " + to_string(measurements.drive.current)) + " OUTPUT_POSITION " + to_string(measurements.output.position) + " OUTPUT_VELOCITY " + to_string(measurements.output.velocity) + " P_VALUE " + to_string(measurements.p);
}

void get_measurements()
{
	measurements.output.previous_position = measurements.output.position;
	measurements.drive.previous_position = measurements.drive.position;
	previous_time = current_time;

	measurements.drive.position = 360 * drive_motor.GetSelectedSensorPosition(0) / motor_encoder_cpr;
	measurements.drive.velocity = 10 * 360 * drive_motor.GetSelectedSensorVelocity(0) / motor_encoder_cpr; // getVelocity returns ticks per 100ms
	measurements.drive.current = drive_motor.GetOutputCurrent();
	measurements.carriage.position = 360 * carriage_motor.GetSelectedSensorPosition(0) / motor_encoder_cpr;
	measurements.carriage.velocity = 10 * 360 * carriage_motor.GetSelectedSensorVelocity(0) / motor_encoder_cpr; // getVelocity returns ticks per 100ms
	measurements.carriage.current = carriage_motor.GetOutputCurrent();

	current_time = get_current_time_ms() / 1000;
	measurement_dt = current_time - previous_time;

	measurements.output.velocity = (measurements.output.position - measurements.output.previous_position) / measurement_dt;

	if (measurements.output.position != measurements.output.previous_position && measurements.drive.position != measurements.drive.previous_position)
		// only measure p value if there's movement
		measurements.p = (9 * measurements.p + (measurements.output.position - measurements.output.previous_position) / (measurements.drive.position - measurements.drive.previous_position)) / 10;

	message = create_measurement_message();
	std::ofstream measurementPipe;
	measurementPipe.open(measurementPipePath, ios::out);
	measurementPipe << message;
	measurementPipe.close();
}

int main()
{

	setup_motors();

	ctre::phoenix::motorcontrol::SupplyCurrentLimitConfiguration supplyLimit(true, 20, 0, 0.001);
	drive_motor.ConfigSupplyCurrentLimit(supplyLimit);

	setup_output_encoder();

	string filename = create_file();

	std::thread command_thread(read_commands);

	double i = 0;
	unsigned long long loop_start_time_ms = get_current_time_ms();
	unsigned long long elapsed_time_ms = 0;
	unsigned long long last_drive_time_ms = 0;
	unsigned long long last_measurement_time_ms = 0;
	unsigned long long current_time_ms = get_current_time_ms();
	double positive_end_stop_position = 0;
	double negative_end_stop_position = 0;
	double start_position = 0;
	double current = 0;

	int count = 0;

	while (1)
	{

		current_time_ms = get_current_time_ms();
		elapsed_time_ms = current_time_ms - loop_start_time_ms;

		if (current_time_ms - last_drive_time_ms > 1000.0 / drive_loop_frequency)
		{

			last_drive_time_ms = current_time_ms;
			// cout<<"command: "<<command<<endl;
			if (command == "CARRIAGE_GOTO")
			{
				ctre::phoenix::unmanaged::Unmanaged::FeedEnable(1.25 * (1 / drive_loop_frequency) * 1000);
				carriage_motor.Set(ControlMode::MotionMagic, round(command_value[0] / 360 * motor_encoder_cpr));
			}
			if (command == "DRIVE_GOTO")
			{
				ctre::phoenix::unmanaged::Unmanaged::FeedEnable(1.25 * (1 / drive_loop_frequency) * 1000);

				drive_motor.Set(ControlMode::MotionMagic, round(command_value[0] / 360 * motor_encoder_cpr));
			}
			if (command == "CARRIAGE_SET_POS")
			{
				measurements.carriage.position = command_value[0];
				carriage_motor.SetSelectedSensorPosition(command_value[0] / 360 * motor_encoder_cpr, 0, 100);
				command = ""; // Clear command
			}
			if (command == "DRIVE_SET_POS")
			{
				measurements.drive.position = command_value[0];
				drive_motor.SetSelectedSensorPosition(command_value[0] / 360 * motor_encoder_cpr, 0, 100);
				command = ""; // Clear command
			}
			if (command == "OUTPUT_SET_POS")
			{
				measurements.output.position = command_value[0];
				command = ""; // Clear command
			}
			if (command == "DRIVE_SINE")
			{

				ctre::phoenix::unmanaged::Unmanaged::FeedEnable(1.25 * (1 / drive_loop_frequency) * 1000);

				target_position = command_value[2] + command_value[0] * sin(2 * PI * command_value[1] * elapsed_time_ms / 1000);

				// convert from degrees to encoder ticks
				target_position = round(target_position / 360 * motor_encoder_cpr);

				drive_motor.Set(ControlMode::Velocity, target_position);
			}
			if (command == "INITIALISE_DRIVE")
			{

				// find positive end-stop
				//  start moving
				if (state == "")
				{
					start_position = measurements.drive.position;
					ctre::phoenix::unmanaged::Unmanaged::FeedEnable(1.25 * (1 / drive_loop_frequency) * 1000);
					drive_motor.Set(ControlMode::Current, default_current);
					state = "finding_positive";
					cout << "Moving towards positive endstop" << endl;
				}

				if (state == "moving_to_midpoint")
				{

					if (abs(measurements.drive.position - target_position) > 0.25 || measurements.drive.position == negative_end_stop_position)
					{
						// Moving to the midpoint

						ctre::phoenix::unmanaged::Unmanaged::FeedEnable(1.25 * (1 / drive_loop_frequency) * 1000);

						drive_motor.Set(ControlMode::MotionMagic, round(target_position / 360 * motor_encoder_cpr));
					}
					else
					{
						// Arrived at the midpoint

						// Zero the sensors
						// drive_motor.SetSelectedSensorPosition(0, 0, 100);
						// measurements.drive.position = 0;
						measurements.output.position = 0;
						cout << "Initialised drive" << endl;
						state = "";

						// only run initialisation once
						command = "";
					}
				}

				if (state == "finding_positive")
				{
					if (count == 3)
					{
						// Stopped at the end stop
						positive_end_stop_position = measurements.drive.position;
						cout << "Positive end stop found: " << positive_end_stop_position << endl;
						ctre::phoenix::unmanaged::Unmanaged::FeedEnable(1.25 * (1 / drive_loop_frequency) * 1000);
						drive_motor.SetInverted(true);
						drive_motor.Set(ControlMode::Current, default_current);
						state = "finding_negative";
						cout << "Moving towards negative endstop" << endl;
						count = 0;
					}
					else
					{

						ctre::phoenix::unmanaged::Unmanaged::FeedEnable(1.25 * (1 / drive_loop_frequency) * 1000);

						drive_motor.Set(ControlMode::Current, default_current);
						if (measurements.drive.position <= measurements.drive.previous_position && measurements.drive.position != start_position)
						{
							count++;
						}
						else
						{
							count = 0;
							// Not there yet
						}
					}
				}

				if (state == "finding_negative")
				{
					if (count == 3)
					{
						// Stopped at the end stop
						negative_end_stop_position = -1 * measurements.drive.position;
						cout << "Negative end stop found: " << negative_end_stop_position << endl;

						drive_motor.SetInverted(false);
						drive_motor.Set(ControlMode::Current, 0);

						state = "moving_to_midpoint";

						target_position = (positive_end_stop_position + negative_end_stop_position) / 2;
						// convert degrees to encoder ticks

						cout << "Moving towards midpoint: " << target_position << "°" << endl;

						// drive_motor.Set(ControlMode::MotionMagic, target_position);

						// command = "DRIVE_GOTO";
						// command_value[0] = target_position;
						count = 0;
					}
					else
					{

						ctre::phoenix::unmanaged::Unmanaged::FeedEnable(1.25 * (1 / drive_loop_frequency) * 1000);

						drive_motor.Set(ControlMode::Current, 1 * default_current);
						if (measurements.drive.position <= measurements.drive.previous_position && measurements.drive.position != positive_end_stop_position)
						{
							count++;
						}
						else
						{
							count = 0;
							// Not there yet
						}
					}
				}
			}

			if (command == "STATIC_FRICTION")
			{
				if (state == "")
				{
					start_position = measurements.drive.position;
					current = 0;
					state = "ramp_up";
				}

				if (state == "ramp_up")
				{
					if (measurements.drive.position == start_position)
					{
						// Hold the same current for 10 loops
						if (count == 10 && current < 20)
						{

							current += current_step;
							cout << "Increasing current: " << current << endl;
							count = 0;
						}

						count++;
						ctre::phoenix::unmanaged::Unmanaged::FeedEnable(1.25 * (1 / drive_loop_frequency) * 1000);
						drive_motor.Set(ControlMode::Current, current);
					}
					else
					{
						drive_motor.Set(ControlMode::Current, 0);
						state = "moving_to_next_position";
					}
				}
				if (state == "moving_to_next_position")
				{
					target_position = start_position + 1;
					if (abs(measurements.drive.position - target_position) > 0.25)
					{
						cout << "Moving to next position" << endl;
						ctre::phoenix::unmanaged::Unmanaged::FeedEnable(1.25 * (1 / drive_loop_frequency) * 1000);
						drive_motor.Set(ControlMode::MotionMagic, round(target_position / 360 * motor_encoder_cpr));
					}
					else
					{
						if (measurements.output.position > 150)
						{
							command = "";
						}
						else
						{
							state = "";
						}
					}
				}
			}

			get_measurements();
			write_to_file(filename);
		}
		if (current_time_ms - last_measurement_time_ms > 1000 / measurement_loop_frequency)
		{
			last_measurement_time_ms = current_time_ms;
		}
	}

	cout << "Closing" << endl;
	return 0;
}
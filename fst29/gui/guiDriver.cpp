#include <chrono> // used for timing within the program
#include <fcntl.h>
#include <fstream> // writing to file/named pipe
#include <iostream>
#include <stdlib.h>
#include <string>
#include <sys/types.h>
#include <thread>
#include <time.h> // used to get the date for data export
#include <unistd.h>

#define Phoenix_No_WPI // remove WPI dependencies
#include "ctre/Phoenix.h"
#include "ctre/phoenix/cci/Unmanaged_CCI.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"

#include "math.h"

#include <wiringPi.h> // RPi GPIO library

#define PI 3.14159265

// -------------program parameters---------------

// path of the named pipes created by gui.py
std::string commandPipePath = "/home/pi/fst29/commands";
std::string measurementPipePath = "/home/pi/fst29/measurements";

// folder to store the saved data
std::string outputDataPath = "/home/pi/fst29/data";

// The frequency of the main loop (including writing to file)
double loop_frequency = 100; // hz

// initialise drive
double default_current =
	8; // The drive current used during initialisation of the output shaft

// Static friction
double current_step =
	0.05; // The step in current when measuring static friction
double max_current = 15;
double static_friction_step =
	1; // ° the distance between consecutive measurements

// initialize carriage
double init_carriage_rotation =
	50; // how far the input shaft rotates when measuring the p value
int init_carriage_ticks =
	250; // how many ticks the carriage moves between two p-value measurements

// -------------physical parameters--------------------

// The pin numbers of the encoder outputs (using wiringPi convention)
int encoder_A_pin_number = 0;
int encoder_B_pin_number = 2;
// Cycles per rotation of the encoders
int motor_encoder_cpr = 2048;
int output_encoder_cpr = 1024 * 4; // encoder cpr * gear ratio

// each tick of the carriage motor encoder changes the p-value by (initial
// estimate, run initialisation to get accurate value)
double change_in_p_per_encoder_tick = -0.000741284;

// ----------- variables used during execution ----------------------------

// upper two bits represent the previous state, lower two bits represent the
// current state, each value corresponds to a transition
int output_encoder_state = 0b0000;

// Each legal transition in ^ represents either a positive or a negative
// movement
float state_transition_matrix[16] = {0, 1, -1, 0, -1, 0, 0, 1,
									 1, 0, 0, -1, 0, -1, 1, 0};

// timing values
std::chrono::time_point<std::chrono::system_clock> last_measurement_time =
	std::chrono::system_clock::now();

std::string raw_command = "";			// incoming command form gui.py
std::string command = "";				// keyword of incomming command
double command_value[4] = {0, 0, 0, 0}; // parameters of incoming command

std::string state = "";	  // current state of multistage operations
std::string message = ""; // outgoing message sent to gui.py

std::fstream data_file; // the stream representing the csv used to store data

struct motorData
{
	// units for drive and output: degrees
	// units for carriage: p-value
	float position;			 // in degrees or p-value
	float velocity;			 // in degrees per second or deltap/s
	float current;			 // in amperes
	float target;			 // the target for the next movement, in °, °/s or p, dp/s
	float previous_position; // in degrees or p-value
};

// stores the current and data
struct
{
	motorData carriage;
	motorData drive;
	motorData output; //.current and .target are not used
} measurements;

// used in static friction test
double current = 0;


// create variables for the two motors
ctre::phoenix::motorcontrol::can::TalonFX carriage_motor(0); // Carrriage motor
ctre::phoenix::motorcontrol::can::TalonFX drive_motor(1);	 // drive Motor

// helper functions to convert between units
int deg_to_motor_tick(double deg)
{
	return std::round(deg / 360 * motor_encoder_cpr);
}

double motor_tick_to_deg(int tick) { return 360.0 * tick / motor_encoder_cpr; }

int p_value_to_tick(double p_value)
{
	return std::round(p_value / change_in_p_per_encoder_tick);
}

double tick_to_p_value(int tick)
{
	return 1.0 * change_in_p_per_encoder_tick * tick;
}

int deg_to_output_tick(double deg)
{
	return std::round(deg / 360 * output_encoder_cpr);
}

double output_tick_to_deg(int tick)
{
	return 360.0 * tick / output_encoder_cpr;
}

std::string create_file()
{
	// Get the current date and time to create a filename
	time_t t = time(0); // current time
	struct tm *now = localtime(&t);
	// Convert timestamp to string
	char datetime[80] = "";
	strftime(datetime, 80, "%F-%H-%M-%S", now);

	// create full filename including path
	std::string filename = outputDataPath + "/" + datetime + ".csv";

	data_file.open(filename, std::ios::out); // create file
	// write the header
	data_file << "Time,"
			  << "Milliseconds,"
			  << "Command,"
			  << "State,"
			  << "Target current,";
	data_file << "Drive position,"
			  << "Drive velocity,"
			  << "Drive current,";
	data_file << "Carriage position,"
			  << "Carriage velocity,"
			  << "Carriage current,";
	data_file << "Output_position,"
			  << "Output_velocity" << std::endl;

	// reopen file for appending
	data_file.close();
	data_file.open(filename, std::ios::app);

	std::cout << "Output file created: " << filename << std::endl;
	return filename;
}

void write_to_file(std::string filename)
{

	// get current time
	// time_t t = std::time(0);
	// struct tm *now = localtime(&t);
	// strftime(time_string, 80, "%T", now);

	std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();

	// This only has seconds accuracy but can be turned into a formatted string
	// easily
	time_t t = std::chrono::system_clock::to_time_t(now);
	struct tm *now_struct = localtime(&t);
	char time_string[80] = "";
	strftime(time_string, 80, "%T", now_struct);

	int current_milliseconds =
		std::chrono::time_point_cast<std::chrono::milliseconds>(now).time_since_epoch().count() % 1000;

	// write a row of data
	data_file << time_string << "," << current_milliseconds
			  << ","
			  //<< get_current_time_ms() % 1000 << ","
			  << raw_command << "," << state << ","<<current<<",";

	data_file << measurements.drive.position << ",";
	data_file << measurements.drive.velocity << ",";
	data_file << measurements.drive.current << ",";

	data_file << measurements.carriage.position << ",";
	data_file << measurements.carriage.velocity << ",";
	data_file << measurements.carriage.current << ",";

	data_file << measurements.output.position << ",";

	data_file << measurements.output.velocity << std::endl;
}

std::string get_command(std::string raw)
{
	// returns the keyword of a raw command
	std::string command = "";
	char delimiter = ' ';

	for (int i = 0; i < raw.length(); i++)
	{
		if (raw[i] != delimiter)
			// collect characters before the delimited
			command += raw[i];
		else
			break; // stop collecting when the delimiter is found
	}
	return command;
}

void get_command_values(std::string raw)
{

	std::string values[4] = {"", "", "",
							 ""}; // the four parameters represented as string

	int current_value = -1; // how manyeth value are we at, starts at -1 because
							// we start at the keyword
	char delimiter = ' ';

	for (int i = 0; i < raw.length(); i++)
	{
		if (raw[i] == delimiter)
			current_value++; // move to next value
		else if (current_value >= 0)
		{
			// collect character of the value
			values[current_value] += raw[i];
		}
	}

	// loop over backwards over found values
	for (current_value; current_value >= 0; current_value--)
	{
		// convert them to doubles and store in the global variable
		command_value[current_value] = std::stod(values[current_value]);
	}
}

void read_commands()
{
	// reads commands sent by the gui.py
	while (1)
	{
		std::ifstream commandPipe;
		commandPipe.open(commandPipePath, std::ifstream::in);
		if (!commandPipe.is_open())
		{
			std::cout << " error : cannot open commandPipe" << std ::endl;
			return;
		}

		std::getline(commandPipe, raw_command);
		commandPipe.close();
		std::cout << "Command received: " << raw_command << std::endl;

		// convert raw command values into keyword and numerical values
		command = get_command(raw_command);
		get_command_values(raw_command);
	}
}

void setup_motors()
{

	c_SetPhoenixDiagnosticsStartTime(0);
	sleep(3);

	drive_motor.ConfigFactoryDefault();
	drive_motor.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0,
											 100);
	carriage_motor.ConfigFactoryDefault();
	carriage_motor.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor,
												0, 100);

	// Joint 1 - Using All Configs
	ctre::phoenix::motorcontrol::can::TalonFXConfiguration allConfigs;
	drive_motor.GetAllConfigs(allConfigs, 100);
	// PID Config

	ctre::phoenix::motorcontrol::can::SlotConfiguration slot_config;
	slot_config.kP = 0.7; // 0.75
	slot_config.kD = 20;
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
	allConfigs.primaryPID.selectedFeedbackSensor =
		FeedbackDevice::IntegratedSensor;

	drive_motor.ConfigAllSettings(allConfigs, 100);

	// carriage_motor.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0,
	// 10, 10);
	// carriage_motor.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic,
	// 10, 10);
	drive_motor.SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, 10,
									 10); // sample feedback signals at 10ms
	// carriage_motor.SetStatusFramePeriod(StatusFrameEnhanced::Status_Brushless_Current
	// , 10, 10); // sample current at 10ms

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
	slot_config0.kD = 20;
	slot_config0.kI = 0.005;
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
	allConfigs0.primaryPID.selectedFeedbackSensor =
		FeedbackDevice::IntegratedSensor;

	carriage_motor.ConfigAllSettings(allConfigs0, 100);

	/* Zero the sensor */
	carriage_motor.SetSelectedSensorPosition(p_value_to_tick(1), 0, 100);
	// Choose a slot for magic motion
	carriage_motor.SelectProfileSlot(0, 0);
	// End of configs

	ctre::phoenix::motorcontrol::SupplyCurrentLimitConfiguration supplyLimit(
		true, 20, 0, 0.001);
	drive_motor.ConfigSupplyCurrentLimit(supplyLimit);
	carriage_motor.ConfigSupplyCurrentLimit(supplyLimit);

	std::cout << "Motor setup done" << std::endl;
}

void callback(void)
{
	// the callback function of the encoder interrupt

	// shift the current state left, store new readings in the two lower bits
	output_encoder_state = ((output_encoder_state << 2) & 0x0F) +
						   (digitalRead(encoder_A_pin_number) << 1) +
						   digitalRead(encoder_B_pin_number);
	// use the matrix to determine change in position
	measurements.output.position += state_transition_matrix[output_encoder_state];
}

void setup_output_encoder()
{
	for (int i = 0; i < 16; i++)
	{
		state_transition_matrix[i] = output_tick_to_deg(
			state_transition_matrix[i]); // convert encoder ticks to degrees
	}

	wiringPiSetup();

	// Initialise the encoder pins
	pinMode(encoder_A_pin_number, INPUT);
	pinMode(encoder_B_pin_number, INPUT);

	// Initialise the state variable
	output_encoder_state = (digitalRead(encoder_A_pin_number) << 1) +
						   digitalRead(encoder_B_pin_number);

	// Attach interrupts to pin state changes
	wiringPiISR(encoder_A_pin_number, INT_EDGE_BOTH, callback);
	wiringPiISR(encoder_B_pin_number, INT_EDGE_BOTH, callback);

	std::cout << "Output encoder setup done" << std::endl;
}

std::string create_measurement_message()
{
	return ("CARRIAGE_POSITION " + std::to_string(measurements.carriage.position) +
			" CARRIAGE_VELOCITY " + std::to_string(measurements.carriage.velocity) +
			" CARRIAGE_CURRENT " + std::to_string(measurements.carriage.current) +
			" DRIVE_POSITION " + std::to_string(measurements.drive.position) +
			" DRIVE_VELOCITY " + std::to_string(measurements.drive.velocity) +
			" DRIVE_CURRENT " + std::to_string(measurements.drive.current)) +
		   " OUTPUT_POSITION " + std::to_string(measurements.output.position) +
		   " OUTPUT_VELOCITY " + std::to_string(measurements.output.velocity);
}

void get_measurements()
{


	std::chrono::time_point<std::chrono::system_clock> current_time = std::chrono::system_clock::now();
	// time since last measurement
	double measurement_dt = std::chrono::duration<double>(
								current_time - last_measurement_time)
								.count();

	last_measurement_time = current_time; // store for next iteration



	measurements.output.previous_position =
		measurements.output.position; // used to detect movement
	measurements.drive.previous_position =
		measurements.drive.position; // used to calculate velocity

	measurements.drive.position = motor_tick_to_deg(drive_motor.GetSelectedSensorPosition(0));
	measurements.drive.velocity =
		10 * motor_tick_to_deg(drive_motor.GetSelectedSensorVelocity(
				 0)); // getVelocity returns ticks per 100ms
	measurements.drive.current = drive_motor.GetStatorCurrent();
	
	measurements.carriage.position =
		tick_to_p_value(carriage_motor.GetSelectedSensorPosition(0));
	measurements.carriage.velocity =
		10 * tick_to_p_value(carriage_motor.GetSelectedSensorVelocity(
				 0)); // getVelocity returns ticks per 100ms
	measurements.carriage.current = carriage_motor.GetStatorCurrent();

	// estimate the velocity of the output shaft
	//measurements.output.velocity =
		//(measurements.output.position - measurements.output.previous_position) /
		//measurement_dt;
	
	message = create_measurement_message();


	
	// send message to gui.py
	std::ofstream measurementPipe;
	measurementPipe.open(measurementPipePath, std::ios::out);
	measurementPipe << message;
	measurementPipe.close();

}

int main()
{

	setup_motors();

	setup_output_encoder();

	std::string filename = create_file();

	// start a new thread to read commands
	std::thread command_thread(read_commands);

	// timings
	std::chrono::time_point<std::chrono::system_clock> last_drive_time =
		std::chrono::system_clock::now(); // the last time the drive cycle was run
	std::chrono::time_point<std::chrono::system_clock> now =
		std::chrono::system_clock::now(); // used to store the current time

	std::chrono::time_point<std::chrono::system_clock> loop_start_time =
		std::chrono::system_clock::now(); // when the program was started
	double elapsed_time = 0;			  // seconds since the program was started

	// used in initialize output
	double positive_end_stop_position = 0;
	double negative_end_stop_position = 0;

	// used in static friction and initialize output
	double drive_start_position = 0;
	double output_start_position = 0;

	// used in initialize carriage
	double first_carriage_position = 0;
	double second_carriage_position = 0;
	double p_at_first_pos = 0;
	double p_at_second_pos = 0;

	// used to slow down processes
	int count = 0;
	// int i =0;
	bool enterasd = 0;
	while (1)
	{

		now = std::chrono::system_clock::now(); // current time

		elapsed_time = std::chrono::duration<double>(
						   now - loop_start_time)
						   .count(); // seconds since program start
	
		// if time since last running > 1/frequency
		
		enterasd = std::chrono::duration<double>(now - last_drive_time).count() >= 1 / loop_frequency;

		if (enterasd)
		{
			
			// last_drive_time_ms = current_time_ms;
			last_drive_time = now;
			if(command == "STOP")
			{
				state = "";
			}
			if (command == "CARRIAGE_GOTO")
			{
				ctre::phoenix::unmanaged::Unmanaged::FeedEnable(
					1.25 * (1 / loop_frequency) * 1000);

				carriage_motor.Set(ControlMode::MotionMagic,
								   p_value_to_tick(command_value[0]));
			}
			if (command == "DRIVE_GOTO")
			{
				ctre::phoenix::unmanaged::Unmanaged::FeedEnable(
					1.25 * (1 / loop_frequency) * 1000);
				drive_motor.Set(ControlMode::MotionMagic,	
								deg_to_motor_tick(command_value[0]));

			
			}
			if (command == "CARRIAGE_SET_POS")
			{
				measurements.carriage.position = command_value[0];
				carriage_motor.SetSelectedSensorPosition(
					p_value_to_tick(command_value[0]), 0, 100);
				command = ""; // Clear command
			}
			if (command == "DRIVE_SET_POS")
			{
				measurements.drive.position = command_value[0];
				drive_motor.SetSelectedSensorPosition(
					deg_to_motor_tick(command_value[0]), 0, 100);
				command = ""; // Clear command
			}
			if (command == "OUTPUT_SET_POS")
			{
				measurements.output.position = command_value[0];
				command = ""; // Clear command
			}
			if (command == "DRIVE_SINE")
			{

				ctre::phoenix::unmanaged::Unmanaged::FeedEnable(
					1.25 * (1 / loop_frequency) * 1000);

				measurements.drive.target =
					command_value[2] +
					command_value[0] * sin(2 * PI * command_value[1] * elapsed_time);

				drive_motor.Set(ControlMode::Velocity,
								deg_to_motor_tick(measurements.drive.target));
			}
			if (command == "INITIALISE_DRIVE")
			{

				// find positive end-stop
				//  start moving
				if (state == "")
				{
					drive_start_position = measurements.drive.position;
					ctre::phoenix::unmanaged::Unmanaged::FeedEnable(
						1.25 * (1 / loop_frequency) * 1000);
					drive_motor.Set(ControlMode::Current, default_current);
					state = "finding_positive";
					std::cout << "Moving towards positive endstop" << std::endl;
				}

				if (state == "moving_to_midpoint")
				{

					if (abs(measurements.drive.position - measurements.drive.target) >
							0.25 ||
						measurements.drive.position == negative_end_stop_position)
					{
						// Moving to the midpoint

						ctre::phoenix::unmanaged::Unmanaged::FeedEnable(
							1.25 * (1 / loop_frequency) * 1000);

						drive_motor.Set(ControlMode::MotionMagic,
										deg_to_motor_tick(measurements.drive.target));
					}
					else
					{
						// Arrived at the midpoint

						// Zero the output encoder
						measurements.output.position = 0;
						std::cout << "Initialised drive" << std::endl;
						state = "";

						// only run initialisation once
						command = "";
					}
				}

				if (state == "finding_positive")
				{
					if (count == 3) // wait 3 loops to make sure it is stopped
					{
						// Stopped at the end stop
						positive_end_stop_position = measurements.drive.position;
						std::cout << "Positive end stop found: "
								  << positive_end_stop_position << std::endl;
						ctre::phoenix::unmanaged::Unmanaged::FeedEnable(
							1.25 * (1 / loop_frequency) * 1000);
						// start moving in opposite direction
						drive_motor.SetInverted(true);
						drive_motor.Set(ControlMode::Current, default_current);
						state = "finding_negative";
						std::cout << "Moving towards negative endstop" << std::endl;
						count = 0;
					}
					else
					{

						ctre::phoenix::unmanaged::Unmanaged::FeedEnable(
							1.25 * (1 / loop_frequency) * 1000);

						drive_motor.Set(ControlMode::Current, default_current);
						if (measurements.drive.position <=
								measurements.drive.previous_position &&
							measurements.drive.position != drive_start_position)
						{
							// Stopped or moving in opposite direction but not at the starting
							// position
							count++;
						}
						else
						{
							// Not there yet
							count = 0;
						}
					}
				}

				if (state == "finding_negative")
				{
					if (count == 3)
					{
						// Stopped at the end stop
						negative_end_stop_position =
							-1 * measurements.drive
									 .position; // -1 because sensors are also inverted
						std::cout << "Negative end stop found: "
								  << negative_end_stop_position << std::endl;

						// undo inversion and stop
						drive_motor.SetInverted(false);
						drive_motor.Set(ControlMode::Current, 0);

						state = "moving_to_midpoint";

						measurements.drive.target =
							(positive_end_stop_position + negative_end_stop_position) / 2;
						// convert degrees to encoder ticks

						std::cout << "Moving towards midpoint: "
								  << measurements.drive.target << "°" << std::endl;

						count = 0;
					}
					else
					{

						ctre::phoenix::unmanaged::Unmanaged::FeedEnable(
							1.25 * (1 / loop_frequency) * 1000);

						drive_motor.Set(ControlMode::Current, 1 * default_current);
						if (measurements.drive.position <=
								measurements.drive.previous_position &&
							measurements.drive.position != positive_end_stop_position)
						{
							// Stopped or moving in opposite direction but not at the starting
							// position
							count++;
						}
						else
						{
							// Not there yet
							count = 0;
						}
					}
				}
			}

			if (command == "STATIC_FRICTION")
			{
				if (state == "")
				{
					// start ramping up the current
					drive_start_position = measurements.drive.position;
					current = 0;
					state = "ramp_up";
				}

				if (state == "ramp_up")
				{
					if (measurements.drive.position ==
						drive_start_position) // no movement
					{

						/* // Hold the same current for 100 loops
						if (count == 100 && current < max_current)
						{
							// increase current
							current += current_step;
							std::cout << "Increasing current: " << current << std::endl;
							count = 0;
						}

						count++;
						ctre::phoenix::unmanaged::Unmanaged::FeedEnable(
							1.25 * (1 / loop_frequency) * 1000);
						drive_motor.Set(ControlMode::Current, current); */
						
						ctre::phoenix::unmanaged::Unmanaged::FeedEnable(
							1.25 * (1 / loop_frequency) * 1000);
						drive_motor.Set(ControlMode::MotionMagic, deg_to_motor_tick(drive_start_position)+1);

					}
					else
					{
						// movement detected
						drive_motor.Set(ControlMode::Current, 0);
						std::cout << "Movement detected" << std::endl;
						state = "cooldown";
						count = 0;
						if (measurements.output.position >= 150)
						{
							// Stop before the endstops
							std::cout << "Getting close to endstops, stopping" << std::endl;
							state = "";
							command = "";
						}
					}
				}
				if (state == "cooldown")
				{
					// wait to allow the current to go down

					ctre::phoenix::unmanaged::Unmanaged::FeedEnable(
						1.25 * (1 / loop_frequency) * 1000);
					drive_motor.Set(ControlMode::Current, 0);

					if (measurements.drive.current < 0.3)
					{
						state = "";
					}
				}
			}

			if (command == "INITIALISE_CARRIAGE")
			{
				if (state == "")
				{
					drive_start_position = measurements.drive.position;
					output_start_position = measurements.output.position;
					first_carriage_position = carriage_motor.GetSelectedSensorPosition(
						0); // working with ticks here to make calculations simpler

					measurements.drive.target =
						drive_start_position + init_carriage_rotation;
					std::cout << "Moving at first pos to: "
							  << motor_tick_to_deg(
									 deg_to_motor_tick(measurements.drive.target))
							  << std::endl;
					state = "moving_at_first_pos";
				}
				if (state == "moving_at_first_pos")
				{
					if (deg_to_motor_tick(measurements.drive.position) !=
						deg_to_motor_tick(measurements.drive.target))
					{
						// Moving to position

						ctre::phoenix::unmanaged::Unmanaged::FeedEnable(
							1.25 * (1 / loop_frequency) * 1000);
						drive_motor.Set(ControlMode::MotionMagic,
										deg_to_motor_tick(measurements.drive.target));
					}
					else
					{
						// Arrived
						measurements.carriage.target =
							first_carriage_position + init_carriage_ticks;
						std::cout << "Moving the carriage to: "
								  << tick_to_p_value(measurements.carriage.target)
								  << std::endl;
						p_at_first_pos =
							(measurements.output.position - output_start_position) /
							(measurements.drive.position - drive_start_position);

						state = "moving_carriage";
					}
				}
				if (state == "moving_carriage")
				{
					if (carriage_motor.GetSelectedSensorPosition(0) !=
						measurements.carriage.target)
					{
						// Moving to position

						ctre::phoenix::unmanaged::Unmanaged::FeedEnable(
							1.25 * (1 / loop_frequency) * 1000);
						carriage_motor.Set(ControlMode::MotionMagic,
										   measurements.carriage.target);
					}
					else
					{
						// Arrived
						std::cout << "Arrived at second carriage position" << std::endl;
						second_carriage_position =
							carriage_motor.GetSelectedSensorPosition(0);

						drive_start_position = measurements.drive.position;
						output_start_position = measurements.output.position;

						measurements.drive.target =
							drive_start_position - init_carriage_rotation;
						std::cout << "Moving at second pos to: "
								  << motor_tick_to_deg(
										 deg_to_motor_tick(measurements.drive.target))
								  << std::endl;

						state = "moving_at_second_pos";
					}
				}
				if (state == "moving_at_second_pos")
				{
					if (deg_to_motor_tick(measurements.drive.position) !=
						deg_to_motor_tick(measurements.drive.target))
					{
						// Moving to position

						ctre::phoenix::unmanaged::Unmanaged::FeedEnable(
							1.25 * (1 / loop_frequency) * 1000);
						drive_motor.Set(ControlMode::MotionMagic,
										deg_to_motor_tick(measurements.drive.target));
					}
					else
					{
						// Arrived

						// p value at second point
						p_at_second_pos =
							(measurements.output.position - output_start_position) /
							(measurements.drive.position - drive_start_position);

						// assuming there is a linear relationship
						change_in_p_per_encoder_tick =
							(p_at_second_pos - p_at_first_pos) /
							(second_carriage_position - first_carriage_position);

						measurements.carriage.position = p_at_second_pos;
						carriage_motor.SetSelectedSensorPosition(
							p_value_to_tick(p_at_second_pos), 0, 100);

						std::cout << "p value initialisation complete, deltap/tick= "
								  << change_in_p_per_encoder_tick
								  << " current p-value: " << p_at_second_pos << std::endl;
						carriage_motor.Set(ControlMode::PercentOutput, 0);
						// stop running
						command = "";
						state = "";
					}
				}
			}

			get_measurements();

			write_to_file(filename);

		}

		
	}

	std::cout << "Closing" << std::endl;
	return 0;
}
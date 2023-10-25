#include <iostream>
#define Phoenix_No_WPI // remove WPI dependencies
#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include "ctre/phoenix/cci/Unmanaged_CCI.h"
#include <unistd.h>
#include <math.h>
using namespace std;

ctre::phoenix::motorcontrol::can::TalonFX carriage_motor(0); // Carrriage motor
ctre::phoenix::motorcontrol::can::TalonFX driving_motor(1);	 // Driving Motor

auto programStart = std::chrono::system_clock::now();
int main()
{
	c_SetPhoenixDiagnosticsStartTime(0);
	sleep(3);

	driving_motor.ConfigFactoryDefault();
	driving_motor.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 100);
	carriage_motor.ConfigFactoryDefault();
	carriage_motor.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 100);

	// Joint 1 - Using All Configs
	ctre::phoenix::motorcontrol::can::TalonFXConfiguration allConfigs;
	carriage_motor.GetAllConfigs(allConfigs, 100);
	// PID Config

	ctre::phoenix::motorcontrol::can::SlotConfiguration slot_config;
	slot_config.kP = 0.7; // 0.75
	slot_config.kD = 0;
	slot_config.kI = 0;
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
	allConfigs.motionAcceleration = 400;

	// Sensor
	allConfigs.primaryPID.selectedFeedbackSensor = FeedbackDevice::IntegratedSensor;

	carriage_motor.ConfigAllSettings(allConfigs, 100);

	// carriage_motor.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);
	// carriage_motor.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 10);
	carriage_motor.SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, 10, 10); // sample feedback signals at 10ms
	// carriage_motor.SetStatusFramePeriod(StatusFrameEnhanced::Status_Brushless_Current , 10, 10); // sample current at 10ms

	carriage_motor.SetNeutralMode(NeutralMode::Brake);

	/* Zero the sensor */
	carriage_motor.SetSelectedSensorPosition(0, 0, 100);
	// Choose a slot for magic motion
	carriage_motor.SelectProfileSlot(0, 0);
	// End of configs

	// Joint 2 - Using All Configs
	ctre::phoenix::motorcontrol::can::TalonFXConfiguration allConfigs0;
	driving_motor.GetAllConfigs(allConfigs0, 100);
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
	allConfigs0.motionAcceleration = 300;
	allConfigs0.motionCurveStrength = 4;

	// Sensor
	allConfigs0.primaryPID.selectedFeedbackSensor = FeedbackDevice::IntegratedSensor;

	driving_motor.ConfigAllSettings(allConfigs0, 100);

	/* Zero the sensor */
	driving_motor.SetSelectedSensorPosition(0, 0, 100);
	// Choose a slot for magic motion
	driving_motor.SelectProfileSlot(0, 0);
	// End of configs

	double frequency = 0.6;
	double A = 30;
	double target_position_p = 0;
	double target_position = 0;

	auto curTime = std::chrono::system_clock::now();
	auto previousTime = curTime;
	auto elapsed = (curTime - previousTime).count();
	int i = 0;
	while (i < 1000)
	{
		usleep(100000);
		curTime = std::chrono::system_clock::now();
		elapsed = (curTime - previousTime).count();
		// if (elapsed > 0.1){ //doing postprocessing every 10ms to save computational power

		// cout<<"test"<<endl;
		ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100);
		// Calibration phase - sinusoidal motion to get the p value afterwards
		i++;
		// target_position_p = 0;
		target_position = A * sin(i / 100.0 * 2 * 3.14);

		carriage_motor.Set(ControlMode::Velocity, (target_position));
		// driving_motor.Set(ControlMode::MotionMagic, (target_position_p));
		// joint_position_p = driving_motor.GetSelectedSensorPosition(0);
		// joint_position = carriage_motor.GetSelectedSensorPosition(0);

		// previousTime = curTime;
		// }
	}

	// int target_position = 0;
	/*while(1)
	{
	cin>>target_position;

	carriage_motor.Set(ControlMode::Velocity, target_position);

	}*/
}
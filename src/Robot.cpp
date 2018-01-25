#include "WPILib.h"
#include "I2C.h"
#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
//#include <Ultrasonic.h>
#include <CANTalon.h>

#define FILTER_TAP_NUM 11
class BuiltiInAccelerometer;
//class AnalogGyro;
class Robot: public frc::IterativeRobot {
public:
	class Joystick *Joystick1, *Joystick2;
	CANTalon *DBLeft, *DBLeftSlave1, *DBLeftSlave2, *DBRight, *DBRightSlave1, *DBRightSlave2, *Shooter, *GearIntake,
			*GearAxis, *Impeller, *RopeClimb;
	PowerDistributionPanel *PDP;
	Encoder m_encoderLeft { 6, 7, false, Encoder::k4X };
	Encoder m_encoderRight { 8, 9, false, Encoder::k4X };
	DoubleSolenoid *SolLeft;
	DoubleSolenoid *SolRight;
	I2C *i2c;
//	Ultrasonic *ultrasonic1;
	AnalogInput *ai0;
	ADXRS450_Gyro *gyro;
	Accelerometer *accel;

	const float turnSensitivity = 0.6;
	const int ramp = 50; //0V to 6V in one second, Slowest ramp is 1.173VpS
	double leftStick = 0, rightStick = 0, turn = 0, driveSpeed = 0, RPM = 0, SpeedA = 0, SpeedB = 0;
//	long long int lastTime;
	float angle = 0;bool driveMode = true, driveModeSwitch = false;

	/*


	 FIR filter designed with
	 http://t-filter.appspot.com

	 sampling frequency: 200 Hz

	 * 0 Hz - 10 Hz
	 gain = 1
	 desired ripple = 5 dB
	 actual ripple = 3.3207879107754374 dB

	 * 30 Hz - 100 Hz
	 gain = 0
	 desired attenuation = -40 dB
	 actual attenuation = -37.41216857786919 dB

	 */
	void Writeled(uint8_t* buf, int length) {
		i2c->WriteBulk(buf, length);
	}

	double filter_taps[FILTER_TAP_NUM] = { 0.021379431511483197, 0.04404756482932042, 0.07581620526448553,
			0.107874533591685, 0.13180182501697535, 0.14067998310714167, 0.13180182501697535, 0.107874533591685,
			0.07581620526448553, 0.04404756482932042, 0.021379431511483197 };

	double sensorReadings1[FILTER_TAP_NUM - 1];

	void RobotInit() {
		chooser.AddDefault(autoName50, autoName50);
		chooser.AddObject(autoNameNone, autoNameNone);
		chooser.AddObject(autoNameShootB, autoNameShootB);
		chooser.AddObject(autoNameShootR, autoNameShootR);
		frc::SmartDashboard::PutData("Auto Modes", &chooser);
		i2c = new I2C(I2C::kOnboard, 53);

		//Setting the Controllers
		Joystick1 = new Joystick(0);
		Joystick2 = new Joystick(1);

		//Setting the Talons
		DBLeft = new CANTalon(1);
		DBLeftSlave1 = new CANTalon(2);
		DBLeftSlave2 = new CANTalon(3);
		DBRight = new CANTalon(4);
		DBRightSlave1 = new CANTalon(5);
		DBRightSlave2 = new CANTalon(6);
		Shooter = new CANTalon(7);
		GearIntake = new CANTalon(8);
		GearAxis = new CANTalon(9);
		Impeller = new CANTalon(10);
		RopeClimb = new CANTalon(11);

		DBLeftSlave1->SetControlMode(CANTalon::kFollower);
		DBLeftSlave2->SetControlMode(CANTalon::kFollower);
		DBRightSlave1->SetControlMode(CANTalon::kFollower);
		DBRightSlave2->SetControlMode(CANTalon::kFollower);
		DBLeftSlave1->Set(DBLeft->GetDeviceID());
		DBLeftSlave2->Set(DBLeft->GetDeviceID());
		DBRightSlave1->Set(DBRight->GetDeviceID());
		DBRightSlave2->Set(DBRight->GetDeviceID());
		DBLeft->SetVoltageRampRate(ramp);
		DBLeftSlave1->SetVoltageRampRate(ramp);
		DBLeftSlave2->SetVoltageRampRate(ramp);
		DBRight->SetVoltageRampRate(ramp);
		DBRightSlave1->SetVoltageRampRate(ramp);
		DBRightSlave2->SetVoltageRampRate(ramp);

//		Shooter->SetVoltageRampRate(1);
//		Shooter->SetControlMode(CANTalon::kSpeed);

		PDP = new PowerDistributionPanel(1);
		ai0 = new AnalogInput(0);

		//Gyro Int and Calibration
		gyro = new ADXRS450_Gyro(SPI::kOnboardCS0);
		gyro->Calibrate();
		gyro->Reset();

		/* Defines how far the mechanism attached to the encoder moves per pulse.
		 * In this case, we assume that a 360 count encoder is directly attached
		 * to a 3 inch diameter (1.5inch radius) wheel, and that we want to
		 * measure distance in inches.
		 */
		m_encoderLeft.SetSamplesToAverage(5);
		m_encoderLeft.SetDistancePerPulse(1.0 / 360.0 * 2.0 * 3.1415 * 1.5);
		m_encoderLeft.SetMinRate(1.0);
		m_encoderRight.SetSamplesToAverage(5);
		m_encoderRight.SetDistancePerPulse(1.0 / 360.0 * 2.0 * 3.1415 * 1.5);
		m_encoderRight.SetMinRate(1.0);

		SolLeft = new DoubleSolenoid(0, 1); //Reversed is Low gear
		SolRight = new DoubleSolenoid(2, 3); //Foward is High gear

		SolLeft->Set(DoubleSolenoid::Value::kForward);	//Low gear
		SolRight->Set(DoubleSolenoid::Value::kForward);
	}

	void Drive(double left, double right) {
//		//div is a safety scaling factor to slow the robot velocity
//		double l, r, div = 2;
//		l = left / div;
//		r = right / div;
//		DBLeft->Set(l);
//		DBRight->Set(r * -1);
		DBLeft->Set(left * -1);
		DBRight->Set(right);
	}

	void WarthogDrive() {
		leftStick = (Joystick1->GetRawAxis(0));
		driveSpeed = (Joystick1->GetRawAxis(5));

		turn = ((turnSensitivity * leftStick * leftStick * leftStick) + (1 - turnSensitivity) * leftStick);
		Drive(driveSpeed - turn, driveSpeed + turn);
	}

	void TankDrive() {
		leftStick = (Joystick1->GetRawAxis(5));
		rightStick = (Joystick1->GetRawAxis(1));

		Drive(leftStick, rightStick);
	}

	double filterUltrasonicFIR(double input) {
		int i;
		double ret = 0;
		for (i = FILTER_TAP_NUM - 2; i > 1; i--) {
			sensorReadings1[i] = sensorReadings1[i - 1];
			ret += sensorReadings1[i] * filter_taps[i + 1];
		}
		sensorReadings1[0] = input;
		ret += input * filter_taps[0];

		return ret;
	}

	double filterUltrasonicIIR(double input) {
		sensorReadings1[0] = (input * 0.07) + (sensorReadings1[0] * 0.93);
		return sensorReadings1[0];
	}

	/*
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * GetString line to get the auto name from the text box below the Gyro.
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * if-else structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	void AutonomousInit() override {
		PDP->ClearStickyFaults();
		DBLeft->ClearStickyFaults();
		DBLeftSlave1->ClearStickyFaults();
		DBLeftSlave2->ClearStickyFaults();
		DBRight->ClearStickyFaults();
		DBRightSlave1->ClearStickyFaults();
		DBRightSlave2->ClearStickyFaults();
		Shooter->ClearStickyFaults();
		GearIntake->ClearStickyFaults();
		GearAxis->ClearStickyFaults();
		Impeller->ClearStickyFaults();
		RopeClimb->ClearStickyFaults();
		SolLeft->ClearAllPCMStickyFaults(0);

		autoSelected = chooser.GetSelected();
		// std::string autoSelected = SmartDashboard::GetString("Auto Selector", autoNameDefault);
		std::cout << "Auto selected: " << autoSelected << std::endl;

		//If you ever create a while loop, make sure that there is always an exit.
		//A team made a while until encoder modes 10ft, go foward
		//The robot never went into Teleop because it was stuck in auto

		if (autoSelected == autoName50) {
			// Custom Auto goes here
			//move forward
			Wait(3);
			Drive(-.5, -.5);
			Wait(6.5);
			Drive(0, 0);

		} else if (autoSelected == autoNameShootB) {
			Impeller->Set(.7);
			Shooter->Set(.7);
			Wait(10);
			Impeller->Set(0);
			Shooter->Set(0);
			Drive(.5, .5);
			Wait(.5);
			Drive(.5, 0);
			Wait(2);
			Drive(.75, .75);
			Wait(3);

		} else if (autoSelected == autoNameShootR) {
			Impeller->Set(.7);
			Shooter->Set(.7);
			Wait(10);
			Impeller->Set(0);
			Shooter->Set(0);
			Drive(.5, .5);
			Wait(.5);
			Drive(0, .5);
			Wait(1.5);
			Drive(.75, .75);
			Wait(3);

		} else {
			// No Auto goes here
			Wait(10);
		}

//		//shoot a ball out joke
//		Wait(1);
//		Impeller->Set(-.7);
//		Shooter->Set(.5);
//		Wait(1.5);
//		Impeller->Set(0);
//		Shooter->Set(0);

//		//shooting
//		ImpellerRight->Set(.75);
//		ImpellerLeft->Set(.75);
//		Shooter->Set(.75);
//		Wait(5)
//		ImpellerRight->Set(0);
//		ImpellerLeft->Set(0);
//		Shooter->Set(0);
	}

	void AutonomousPeriodic() {
		if (autoSelected == autoNameNone) {
			// Custom Auto goes here
		} else {
			// Default Auto goes here
		}

	}

	void TeleopInit() {
		//not sure what is needed but resetting the encoder position
		Shooter->SetPulseWidthPosition(0);
		Shooter->SetEncPosition(0);
		Shooter->SetPosition(0);
	}

	void TeleopPeriodic() {

		frc::SmartDashboard::PutNumber("Impeller Speed", SpeedA * 100);

		if (driveMode)
			TankDrive();
		else
			WarthogDrive();

		if (Joystick1->GetRawButton(8) && driveModeSwitch == false) {
			driveMode = !driveMode;
			driveModeSwitch = true;
		}
		if (!Joystick1->GetRawButton(8) && driveModeSwitch == true) {
			driveModeSwitch = false;
		}

		SpeedA = Joystick2->GetRawAxis(2);

		//auto currentTime = std::chrono::system_clock::now().time_since_epoch();
		//auto ccurrentTime = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime).count();
		//ultrasonic1->GetRangeMM();
		double sensorReading1 = ai0->GetValue();
		double filteredReading1 = filterUltrasonicIIR(sensorReading1);
		double distance1 = (sensorReading1 - 4.4882) / 7.5768;
		//double sensorReading = ai0->GetValue() * 20 / 10.0;
		frc::SmartDashboard::PutNumber("Range", sensorReading1);
		frc::SmartDashboard::PutNumber("Filtered Reading", filteredReading1);
		frc::SmartDashboard::PutNumber("Distance1 (cm)", distance1);
		frc::SmartDashboard::PutNumber("Impeller Speed", SpeedA * 100);

		//Accelerometer
		accel = new BuiltInAccelerometer();
		double xVal = accel->GetX();
		double yVal = accel->GetY();
		double zVal = accel->GetZ();
		frc::SmartDashboard::PutNumber("Z Axis", zVal);
		frc::SmartDashboard::PutNumber("y Axis", yVal);
		frc::SmartDashboard::PutNumber("X Axis", xVal);

		//Gyroscope
		angle = gyro->GetAngle();
		frc::SmartDashboard::PutNumber("Gyroscope", angle);
		frc::SmartDashboard::PutNumber("POV", Joystick1->GetPOV());

		frc::SmartDashboard::PutNumber("Mag Pulse Encoder Position", Shooter->GetPulseWidthPosition());
		frc::SmartDashboard::PutNumber("Mag Pulse Encoder Velocity", Shooter->GetPulseWidthVelocity());
		frc::SmartDashboard::PutNumber("Mag Encoder Position", Shooter->GetEncPosition());
		frc::SmartDashboard::PutNumber("Mag Encoder Velocity", Shooter->GetEncVel());
		frc::SmartDashboard::PutNumber("Mag Encoder Speed", Shooter->GetSpeed());
		frc::SmartDashboard::GetNumber("RPM Speed", RPM);
		frc::SmartDashboard::PutNumber("RPM Setting", RPM);
		frc::SmartDashboard::PutNumber("Encoder Distance Left", m_encoderLeft.GetDistance());
		frc::SmartDashboard::PutNumber("Encoder Rate Left", m_encoderLeft.GetRate());
		frc::SmartDashboard::PutNumber("Encoder Distance R", m_encoderRight.GetDistance());
		frc::SmartDashboard::PutNumber("Encoder Rate Right", m_encoderRight.GetRate());
		frc::SmartDashboard::PutNumber("Climer Amps", PDP->GetCurrent(15));

		//Set shooter state
//		if (Joystick2->GetRawButton(6)) {
//			SpeedB = .5;
//			//			Shooter->SetControlMode(CANTalon::kSpeed);
//			//			Shooter->Set(13400);
//		} else if (Joystick2->GetRawButton(7)) {
//			SpeedB = .6;
//			//			Shooter->SetControlMode(CANTalon::kSpeed);
//			//			Shooter->Set(13600);
//		} else if (Joystick2->GetRawButton(8)) {
//			SpeedB = .7;
//			//			Shooter->SetControlMode(CANTalon::kSpeed);
//			//			Shooter->Set(13800);
//		} else if (Joystick2->GetRawButton(9)) {
//			SpeedB = .75;
//		} else if (Joystick2->GetRawButton(10)) {
//			SpeedB = .80;
//		} else if (Joystick2->GetRawButton(11)) {
//			SpeedB = .90;
////		} else {
////
////			Shooter->SetControlMode(CANTalon::kVoltage);
////			Shooter->Set(0);
//		}
//		//Shoot
//		if (Joystick2->GetRawButton(1)) {
//			Shooter->Set(SpeedB);
//		} else {
//			Shooter->Set(0);
//		}

		if (Joystick2->GetRawButton(6)) {
//			Shooter->Set(.4);
			Shooter->SetControlMode(CANTalon::kSpeed);
			Shooter->Set(13400);
		} else if (Joystick2->GetRawButton(7)) {
//			Shooter->Set(.5);
			Shooter->SetControlMode(CANTalon::kSpeed);
			Shooter->Set(13600);
		} else if (Joystick2->GetRawButton(8)) {
//			Shooter->Set(.6);
			Shooter->SetControlMode(CANTalon::kSpeed);
			Shooter->Set(13800);
		} else if (Joystick2->GetRawButton(9)) {
//			Shooter->Set(.75);
			Shooter->SetControlMode(CANTalon::kVoltage);
			Shooter->Set(13900);
		} else if (Joystick2->GetRawButton(10)) {
//			Shooter->Set(-.75);
			Shooter->SetControlMode(CANTalon::kSpeed);
			Shooter->Set(14000);
			//} else {
			//Shooter->Set(0);
//			Shooter->Set(Joystick1->GetRawAxis(3));
		} else if (Joystick2->GetRawButton(11)) {
//			Shooter->Set(-.75);
			Shooter->SetControlMode(CANTalon::kSpeed);
			Shooter->Set(14200);
		} else {
			Shooter->SetControlMode(CANTalon::kVoltage);
			Shooter->Set(0);
			//			Shooter->Set(Joystick1->GetRawAxis(3));
		}

//		loader->Set(Joystick1->GetRawAxis(3) - Joystick1->GetRawAxis(2));

		//Impeller Movement
		if (Joystick2->GetRawButton(3)) {
			Impeller->Set(SpeedA);
		} else {
			Impeller->Set(0);
		}

		//Rope Climber
//		if ((Joystick2->GetRawAxis(1) * -1) > .25) {
//			RopeClimb->Set(.75);
//		if(Joystick2->GetRawAxis(1)<0)
		if (PDP->GetCurrent(15) < 60)
			RopeClimb->Set(Joystick2->GetRawAxis(1));	//so you can very the speed
		else
			(RopeClimb->Set(Joystick2->GetRawAxis(1) * .25));
//		else
//			RopeClimb->Set(0);

		//gear switcher
		if (Joystick1->GetRawButton(9/*left stick button*/) | Joystick1->GetRawButton(5/*left trigger button*/)) {
			SolLeft->Set(DoubleSolenoid::Value::kReverse);	//Low gear
			SolRight->Set(DoubleSolenoid::Value::kReverse);
			Joystick1->SetRumble(GenericHID::kLeftRumble, .8);
		} else if (Joystick1->GetRawButton(10/*right stick button*/)
				| Joystick1->GetRawButton(6/*right trigger button*/)) {
			SolLeft->Set(DoubleSolenoid::Value::kForward);	//high gear
			SolRight->Set(DoubleSolenoid::Value::kForward);
			Joystick1->SetRumble(GenericHID::kLeftRumble, .8);
		} else {
			Joystick1->SetRumble(GenericHID::kLeftRumble, 0);
		}

		//loader
		//code for loader
	}

	void TestPeriodic() {
		lw->Run();
	}

private:
	frc::LiveWindow* lw = LiveWindow::GetInstance();
	frc::SendableChooser<std::string> chooser;
	const std::string autoNameNone = "No Auto";
	const std::string autoName50 = "50% 6sec";
	const std::string autoNameShootB = "Shoot and cross BLUE SIDE";
	const std::string autoNameShootR = "Shoot and cross RED SIDE";
	std::string autoSelected;
};

START_ROBOT_CLASS(Robot)

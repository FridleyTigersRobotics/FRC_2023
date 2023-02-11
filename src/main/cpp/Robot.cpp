// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/PWMVictorSPX.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/Encoder.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <AHRS.h>
#include <frc/SPI.h>
#include <frc/controller/PIDController.h>
#include <frc/XboxController.h>
#include <ctre/Phoenix.h>
#include <frc/PneumaticsControlModule.h>
#include <frc/DoubleSolenoid.h>

class Robot : public frc::TimedRobot {

  // PWM Devices
  // None yet.

  // CAN Devices
  int const kFLMotorCanId = 1;
  int const kRLMotorCanId = 2;
  int const kRRMotorCanId = 3;
  int const kFRMotorCanId = 4;
  int const kPcmCanId     = 6;

  WPI_TalonSRX                 m_frontleftMotor { kFLMotorCanId };
  WPI_TalonSRX                 m_rearleftMotor  { kRLMotorCanId };
  WPI_TalonSRX                 m_rearrightMotor { kRRMotorCanId };
  WPI_TalonSRX                 m_frontrightMotor{ kFRMotorCanId };
  frc::PneumaticsControlModule m_pcm            { kPcmCanId };


  // Pneumatics
  frc::DoubleSolenoid m_clawSolenoid{ kPcmCanId, frc::PneumaticsModuleType::CTREPCM, 0, 1 };


  // Digial I/O
  frc::Encoder m_leftencoder { 0, 1 };
  frc::Encoder m_rightencoder{ 2, 3 };


  // SPI Devices
  AHRS m_imu{ frc::SPI::Port::kMXP };


  // User Input Devices
  frc::XboxController m_stick{0};


  // Drive System
  frc::MotorControllerGroup m_leftMotors { m_frontleftMotor, m_rearleftMotor };
  frc::MotorControllerGroup m_rightMotors{ m_frontrightMotor, m_rearrightMotor };
  frc::DifferentialDrive    m_robotDrive { m_leftMotors, m_rightMotors };


  // PIDs
  double const kRotateGyroP{ 0.0100 };
  double const kRotateGyroI{ 0.0000 };
  double const kRotateGyroD{ 0.0000 };
  frc2::PIDController m_rotateGyroPid{ kRotateGyroP, kRotateGyroI, kRotateGyroD };

  double kBalanceP{ 0.04 };
  double kBalanceI{ 0.00 };
  double kBalanceD{ 0.00 };
  frc::PIDController m_balancePid{ kBalanceP, kBalanceI, kBalanceD };


  // Other
  frc::Timer m_autoTimer;





 public:

  void RobotInit() override {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotors.SetInverted(true);
    m_leftencoder.SetReverseDirection(true); 
    //m_imu.Calibrate();
  }

  void TeleopInit() override {
    m_leftencoder.Reset();
    m_rightencoder.Reset();
    m_balancePid.Reset();
    m_balancePid.SetSetpoint( 0.0 );
  }



  void TeleopPeriodic() override {
    // Drive with arcade style
    frc::SmartDashboard::PutNumber("Left Encoder", m_leftencoder.Get());   
    frc::SmartDashboard::PutNumber("Right Encoder", m_rightencoder.Get());

    double pidValue = m_balancePid.Calculate( m_imu.GetRoll() );
    frc::SmartDashboard::PutNumber("pidValue", pidValue);

    if ( m_stick.GetAButton() )
    {
      frc::SmartDashboard::PutNumber("A Button", m_stick.GetAButton());
      m_robotDrive.ArcadeDrive(-pidValue, 0.0);
    }
    else
    {
      frc::SmartDashboard::PutNumber("A Button",m_stick.GetAButton());
      m_robotDrive.ArcadeDrive(-m_stick.GetLeftY(), -m_stick.GetLeftX());
    }
    
    if ( m_stick.GetBButton() )
    {
      m_clawSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
    }
    else if (m_stick.GetYButton())
    {
      m_clawSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
    }
    else
    {
      m_clawSolenoid.Set(frc::DoubleSolenoid::Value::kOff);
    }
  }


  void AutonomousInit() override {
    m_imu.Reset();
    m_autoTimer.Stop();
    m_autoTimer.Reset();
    m_autoTimer.Start();
  }


  void AutonomousPeriodic() override {
    //RotateDegrees(90);
    DriveForTime(0.5,0.5,0);
  }


  bool RotateDegrees( double angle )
  {
    double rotationSpeed = m_rotateGyroPid.Calculate( m_imu.GetAngle(), angle );

    if ( m_rotateGyroPid.AtSetpoint() )
    {
      //m_drive.DriveCartesian( 0.0, 0.0, 0.0 );
      m_robotDrive.ArcadeDrive(0,0);
      return true;
    }
    else
    {
      //m_drive.DriveCartesian( 0.0, 0.0, -rotationSpeed );
      m_robotDrive.ArcadeDrive(0,-rotationSpeed);
      return false;
    }
  }


  bool DriveForTime( double speed, double time, double initialAngle )
  {
    double rotationSpeed = m_rotateGyroPid.Calculate( m_imu.GetAngle(), initialAngle );

    if ( m_autoTimer.Get() < (units::time::second_t)time )
    {
      m_robotDrive.ArcadeDrive(speed,0);
      return false;
    }
    else
    {
      m_robotDrive.ArcadeDrive(0,0);
      return true;
    }
  }

};


#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif

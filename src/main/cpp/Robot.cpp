// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/PWMVictorSPX.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/Encoder.h>
#include <frc/AnalogInput.h>
#include <frc/AnalogEncoder.h>
#include <frc/DigitalInput.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <AHRS.h>
#include <frc/SPI.h>
#include <frc/controller/PIDController.h>
#include <frc/XboxController.h>
#include <ctre/Phoenix.h>
#include <frc/PneumaticsControlModule.h>
#include <frc/DoubleSolenoid.h>
#include <ctre/phoenix/motorcontrol/can/WPI_VictorSPX.h>


class Robot : public frc::TimedRobot {

  // PWM Devices
  // None yet.

  // CAN Devices
  int const kFLMotorCanId  = 1;
  int const kRLMotorCanId  = 2;
  int const kRRMotorCanId  = 3;
  int const kFRMotorCanId  = 4;
  int const kPcmCanId      = 6;
  int const kLinActACanId  = 8;
  int const kLinActBCanId  = 9;
  int const kLiftCanId     = 10;

  WPI_TalonSRX                 m_frontleftMotor { kFLMotorCanId };
  WPI_TalonSRX                 m_rearleftMotor  { kRLMotorCanId };
  WPI_TalonSRX                 m_rearrightMotor { kRRMotorCanId };
  WPI_TalonSRX                 m_frontrightMotor{ kFRMotorCanId };
  frc::PneumaticsControlModule m_pcm            { kPcmCanId     };
  WPI_VictorSPX                m_LinActRight    { kLinActACanId };
  WPI_VictorSPX                m_LinActLeft     { kLinActBCanId };
  WPI_VictorSPX                m_Lift           { kLiftCanId    };


  // Pneumatics
  frc::DoubleSolenoid m_clawSolenoid{ kPcmCanId, frc::PneumaticsModuleType::CTREPCM, 0, 1 };
  frc::DoubleSolenoid m_liftSolenoid{ kPcmCanId, frc::PneumaticsModuleType::CTREPCM, 2, 3 };


  // Digial I/O
  frc::Encoder      m_leftencoder     { 0, 1 };
  frc::Encoder      m_rightencoder    { 2, 3 };
  frc::DigitalInput m_bottomLimitLeft { 4 };
  frc::DigitalInput m_bottomLimitRight{ 5 };
  frc::Encoder      m_liftEncoder     { 6, 7 };


  // Analog I/O
  frc::AnalogInput    m_angleIntputA{ 0 };
  frc::AnalogInput    m_angleIntputB{ 1 };
  frc::AnalogEncoder  m_angleEncoderA{ m_angleIntputA };
  frc::AnalogEncoder  m_angleEncoderB{ m_angleIntputB };
  frc::AnalogInput    m_clawIntputA{ 2 };
  frc::AnalogInput    m_clawIntputB{ 3 };
  frc::AnalogEncoder  m_clawEncoderA{ m_clawIntputA };
  frc::AnalogEncoder  m_clawEncoderB{ m_clawIntputB };


  double kDistPerRotation = 360;  //how far the nechanism travels in 1 rotation of the encoder in angular degrees

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



/*
  enum lift_state_e
  {
    LIFT_STATE_HOLD,
    LIFT_STATE_RAISE,
    LIFT_STATE_LOWER
  } m_liftState = LIFT_STATE_HOLD;


  enum lift_state_e
  {
    ANGLE_STATE_HOLD,
    ANGLE_STATE_RAISE,
    ANGLE_STATE_LOWER
  } m_angleState = ANGLE_STATE_HOLD;

*/





 public:

  void RobotInit() override {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotors.SetInverted(true);
    m_leftencoder.SetReverseDirection(true); 
    //m_imu.Calibrate();

    m_angleEncoderA.SetDistancePerRotation( kDistPerRotation );
    m_angleEncoderB.SetDistancePerRotation( kDistPerRotation );
  }

  void RobotPeriodic() override {
    //frc::SmartDashboard::PutNumber("Left Encoder", m_leftencoder.Get());   
    //frc::SmartDashboard::PutNumber("Right Encoder", m_rightencoder.Get());

    frc::SmartDashboard::PutNumber("m_bottomLimitLeft",  m_bottomLimitLeft.Get() );
    frc::SmartDashboard::PutNumber("m_bottomLimitRight", m_bottomLimitRight.Get());

    frc::SmartDashboard::PutNumber("m_angleIntputA",  m_angleIntputA.GetValue());
    frc::SmartDashboard::PutNumber("m_angleIntputB",  m_angleIntputB.GetValue());
    frc::SmartDashboard::PutNumber("m_angleEncoderA", m_angleEncoderA.GetDistance());
    frc::SmartDashboard::PutNumber("m_angleEncoderB", m_angleEncoderB.GetDistance());

    frc::SmartDashboard::PutNumber("m_clawIntputA",  m_clawIntputA.GetValue());
    frc::SmartDashboard::PutNumber("m_clawIntputB",  m_clawIntputB.GetValue());
    frc::SmartDashboard::PutNumber("m_clawEncoderA", m_clawEncoderA.GetDistance());
    frc::SmartDashboard::PutNumber("m_clawEncoderB", m_clawEncoderB.GetDistance());

  }


  void TeleopInit() override {
    m_leftencoder.Reset();
    m_rightencoder.Reset();
    m_balancePid.Reset();
    m_balancePid.SetSetpoint( 0.0 );
  }



  void TeleopPeriodic() override {
    double pidValue = m_balancePid.Calculate( m_imu.GetRoll() );
    //frc::SmartDashboard::PutNumber("pidValue", pidValue);

    /*if ( m_stick.GetAButton() )
    {
      frc::SmartDashboard::PutNumber("A Button", m_stick.GetAButton());
      m_robotDrive.ArcadeDrive(-pidValue, 0.0);
    }
    else
    {
      frc::SmartDashboard::PutNumber("A Button",m_stick.GetAButton());
      m_robotDrive.ArcadeDrive(-m_stick.GetLeftY(), -m_stick.GetLeftX());
    }*/

    m_robotDrive.ArcadeDrive(-m_stick.GetLeftY(), -m_stick.GetLeftX());
    
    /*if ( m_stick.GetBButton() )
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
    }*/

    m_LinActRight.Set( 0.0 );
    m_LinActLeft.Set( 0.0 );






    if( m_stick.GetLeftBumper() )
    {
      if ( m_bottomLimitLeft.Get() )
      {
        m_angleEncoderA.Reset();
        m_LinActLeft.Set( 1.0 );
      }

      if ( m_bottomLimitRight.Get() )
      {
        m_angleEncoderA.Reset();
        m_LinActRight.Set( 1.0 );
      }

    }
    else if( m_stick.GetRightBumper() )  // passenger out
    {
      m_LinActRight.Set( -1.0 );
      m_LinActLeft.Set( -1.0 );
    }
    else
    {
      m_LinActRight.Set( 0.0 );
      m_LinActLeft.Set( 0.0 );
    }


    if ( m_stick.GetAButton() )
    {
      m_Lift.Set( 0.5 );
    }
    else if ( m_stick.GetBButton() )
    {
      m_Lift.Set( -0.5 );
    }
    else
    {
      m_Lift.Set( 0.0 );
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
      m_robotDrive.ArcadeDrive(0,0);
      return true;
    }
    else
    {
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

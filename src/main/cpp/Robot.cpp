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
/**
 * This is a demo program showing the use of the DifferentialDrive class.
 * Runs the motors with arcade steering.
*/
class Robot : public frc::TimedRobot {
  WPI_TalonSRX          m_frontleftMotor{1};
  WPI_TalonSRX          m_rearleftMotor{2};
  WPI_TalonSRX          m_rearrightMotor{3};
  WPI_TalonSRX          m_frontrightMotor{4};
  
  frc::MotorControllerGroup m_leftMotors{ m_frontleftMotor, m_rearleftMotor };
  frc::MotorControllerGroup m_rightMotors{ m_frontrightMotor, m_rearrightMotor };

  frc::DifferentialDrive m_robotDrive{m_leftMotors, m_rightMotors};
  frc::XboxController m_stick{0};
  frc::Encoder m_leftencoder{0,1};
  frc::Encoder m_rightencoder{2,3};

  frc::PneumaticsControlModule pcm{6};

  frc::DoubleSolenoid m_clawSolenoid{ 6, frc::PneumaticsModuleType::CTREPCM, 0, 1 };

  AHRS m_imu{ frc::SPI::Port::kMXP }; /* Alternatives:  I2C.Port.kMXP or SerialPort.Port.kUSB */

  double kBalanceP{ 0.04 };
  double kBalanceI{ 0.0 };
  double kBalanceD{ 0.0 };
  frc::PIDController m_balancePid{ kBalanceP, kBalanceI, kBalanceD };

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


 frc::SmartDashboard::PutNumber("Pitch", m_imu.GetPitch());
 frc::SmartDashboard::PutNumber("Roll", m_imu.GetRoll());
 frc::SmartDashboard::PutNumber("Yaw", m_imu.GetYaw());
 frc::SmartDashboard::PutNumber("x", m_imu.GetRawGyroX());
 frc::SmartDashboard::PutNumber("y", m_imu.GetRawGyroY());
 frc::SmartDashboard::PutNumber("z", m_imu.GetRawGyroZ());
  frc::SmartDashboard::PutNumber("angle", m_imu.GetAngle());

  }
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif

//TODO: Need to force lift to lowest position at beginning of robot enable so it does not go back to last saved lift setpoint

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <math.h>
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
#include <frc/smartdashboard/SendableChooser.h>
#include <AHRS.h>
#include <frc/SPI.h>
#include <frc/controller/PIDController.h>
#include <frc/XboxController.h>
#include <ctre/Phoenix.h>
#include <frc/PneumaticsControlModule.h>
#include <frc/DoubleSolenoid.h>
#include <ctre/phoenix/motorcontrol/can/WPI_VictorSPX.h>

class DualChannelAnalogEncoder {
  public:
    DualChannelAnalogEncoder( 
      int channelA, 
      int channelB, 
      int minValidValue = 500,
      int maxValidValue = 3000 
      ) :
      m_inputA{channelA},
      m_inputB{channelB}
    {
      m_value         = 0;
      m_prevValueA    = m_inputA.GetValue();
      m_prevValueB    = m_inputB.GetValue();
      m_minValidValue = minValidValue;
      m_maxValidValue = maxValidValue;
    }

    void Reset()
    {
      m_value = 0;
    }

    int GetValue()
    {
      return m_value;
    }

    void Update()
    {
      int valueA = m_inputA.GetValue();
      int valueB = m_inputB.GetValue();

      if ( valueA       > m_minValidValue && valueA       < m_maxValidValue &&
           m_prevValueA > m_minValidValue && m_prevValueA < m_maxValidValue )
      {
        m_value += valueA - m_prevValueA;
      }
      else if ( valueB       > m_minValidValue && valueB       < m_maxValidValue &&
                m_prevValueB > m_minValidValue && m_prevValueB < m_maxValidValue )
      {
        m_value += valueB - m_prevValueB;
      }
      else
      {
        m_errorCount++;
      }
      m_prevValueA = valueA;
      m_prevValueB = valueB;
    }

  private:
    frc::AnalogInput  m_inputA;
    frc::AnalogInput  m_inputB;
    int               m_prevValueA;
    int               m_prevValueB;
    int               m_value;
    int               m_minValidValue;
    int               m_maxValidValue;
    int               m_errorCount;
};







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
  int const kClawRotateCanId = 11;

  WPI_TalonSRX                 m_frontleftMotor { kFLMotorCanId };
  WPI_TalonSRX                 m_rearleftMotor  { kRLMotorCanId };
  WPI_TalonSRX                 m_rearrightMotor { kRRMotorCanId };
  WPI_TalonSRX                 m_frontrightMotor{ kFRMotorCanId };
  frc::PneumaticsControlModule m_pcm            { kPcmCanId     };
  WPI_VictorSPX                m_LinActRight    { kLinActACanId };
  WPI_VictorSPX                m_LinActLeft     { kLinActBCanId };
  WPI_VictorSPX                m_Lift           { kLiftCanId    };
  WPI_VictorSPX                m_ClawRotate     { kClawRotateCanId };


  // Pneumatics
  frc::DoubleSolenoid m_clawSolenoid{ kPcmCanId, frc::PneumaticsModuleType::CTREPCM, 0, 1 };
  frc::DoubleSolenoid m_liftSolenoid{ kPcmCanId, frc::PneumaticsModuleType::CTREPCM, 2, 3 };


  // Digial I/O
  frc::Encoder      m_leftencoder     { 6, 7 };
  frc::Encoder      m_rightencoder    { 2, 3 };
  frc::DigitalInput m_bottomLimitLeft { 4 };
  frc::DigitalInput m_bottomLimitRight{ 5 };
  frc::DigitalInput m_liftLimitTop    { 0 };
  frc::DigitalInput m_liftLimitBot    { 9 };
  frc::Encoder      m_liftencoder     { 1, 8 };

  // Analog I/O
  DualChannelAnalogEncoder m_angleEncoder{ 0, 1 };
  DualChannelAnalogEncoder m_clawEncoder { 2, 3 };


  // SPI Devices
  AHRS m_imu{ frc::SPI::Port::kMXP };


  // User Input Devices
  frc::XboxController m_stick{0};
  frc::XboxController m_logitechController{1};

  // Drive System
  frc::MotorControllerGroup m_leftMotors { m_frontleftMotor,  m_rearleftMotor  };
  frc::MotorControllerGroup m_rightMotors{ m_frontrightMotor, m_rearrightMotor };
  frc::DifferentialDrive    m_robotDrive { m_leftMotors,      m_rightMotors    };


  // PIDs
  double const kRotateGyroP{ 0.0100 };
  double const kRotateGyroI{ 0.0000 };
  double const kRotateGyroD{ 0.0000 };
  frc2::PIDController m_rotateGyroPid{ kRotateGyroP, kRotateGyroI, kRotateGyroD };

  double kBalanceP{ 0.04 };
  double kBalanceI{ 0.00 };
  double kBalanceD{ 0.00 };
  frc::PIDController m_balancePid{ kBalanceP, kBalanceI, kBalanceD };


  double kClawRotateP{ 2.0e-4 };
  double kClawRotateI{ 0.00 };
  double kClawRotateD{ 0.00 };
  frc::PIDController m_ClawRotatePid{ kClawRotateP, kClawRotateI, kClawRotateD };

  double kLiftHoldP{ 8.0e-5 };
  double kLiftHoldI{ 4.0e-4 };
  double kLiftHoldD{ 0.00 };
  frc::PIDController m_LiftHoldPid{ kLiftHoldP, kLiftHoldI, kLiftHoldD };


  // Other
  frc::Timer m_autoTimer;

  const std::string    kAutoNameDefault = "Default";
  const std::string    kAutoDrive       = "Drive";
  frc::SendableChooser<std::string> m_autoChooser;

  bool m_LiftLimitsSet = false;


  enum lift_state_e
  {
    LIFT_STATE_HOLD,
    LIFT_STATE_RAISE,
    LIFT_STATE_LOWER
  } m_liftState = LIFT_STATE_HOLD;
  


  enum angle_state_e
  {
    ANGLE_STATE_HOLD,
    ANGLE_STATE_RAISE,
    ANGLE_STATE_LOWER
  } m_angleState = ANGLE_STATE_HOLD;


  enum claw_state_e
  {
    CLAW_STATE_HOLD,
    CLAW_STATE_OPEN,
    CLAW_STATE_CLOSE
  } m_clawState = CLAW_STATE_HOLD;


    enum lift2_state_e
  {
    LIFT2_STATE_HOLD,
    LIFT2_STATE_OPEN,
    LIFT2_STATE_CLOSE
  } m_lift2State = LIFT2_STATE_HOLD;








 public:

  void RobotInit() override {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotors.SetInverted(true);
    m_leftencoder.SetReverseDirection(true); 
    //m_imu.Calibrate();

    // Autonomous Chooser
    m_autoChooser.SetDefaultOption( kAutoNameDefault, kAutoNameDefault );
    m_autoChooser.AddOption       ( kAutoDrive,       kAutoDrive       );

    frc::SmartDashboard::PutData("Auto Modes", &m_autoChooser);

    m_LiftHoldPid.SetIntegratorRange( -0.1, 0.1 ); //stops integrator wind-up
  }

  void RobotPeriodic() override {
    m_angleEncoder.Update();
    m_clawEncoder.Update();

    frc::SmartDashboard::PutNumber("m_leftencoder",      m_leftencoder.Get());   
    frc::SmartDashboard::PutNumber("m_rightencoder",     m_rightencoder.Get());
    frc::SmartDashboard::PutNumber("m_bottomLimitLeft",  m_bottomLimitLeft.Get());
    frc::SmartDashboard::PutNumber("m_bottomLimitRight", m_bottomLimitRight.Get());
    frc::SmartDashboard::PutNumber("m_angleEncoder",     m_angleEncoder.GetValue());
    frc::SmartDashboard::PutNumber("m_clawEncoder",      m_clawEncoder.GetValue());
    frc::SmartDashboard::PutNumber("m_liftLimitTop",     m_liftLimitTop.Get());
    frc::SmartDashboard::PutNumber("m_liftLimitBot",     m_liftLimitBot.Get());
  }


  void TestInit() override {
    m_angleEncoder.Reset();
    m_clawEncoder.Update();
    m_leftencoder.Reset();
    m_rightencoder.Reset();
    m_balancePid.Reset();
    m_balancePid.SetSetpoint( 0.0 );
    m_ClawRotatePid.Reset();
    m_liftencoder.Reset();
  }


  void TestPeriodic() override {
    bool AngleUpR    = m_stick.GetRightBumper();
    bool AngleUpL    = m_stick.GetLeftBumper();
    bool AngleDownR  = m_stick.GetYButton();
    bool AngleDownL  = m_stick.GetXButton();
    bool LiftUp      = m_stick.GetAButton();
    bool LiftDown    = m_stick.GetBButton();

    //m_robotDrive.ArcadeDrive(-m_stick.GetLeftY(), -m_stick.GetLeftX());

    if( AngleDownL )
    {
      m_LinActLeft.Set( 1.0 );
    }
    else if( AngleUpL )
    {
      m_LinActLeft.Set( -1.0 );
    }
    else
    {
      m_LinActLeft.Set( 0.0 );
    }

    if( AngleDownR )
    {
      m_LinActRight.Set( 1.0 );
    }
    else if( AngleUpR )
    {
      m_LinActRight.Set( -1.0 );
    }
    else
    {
      m_LinActRight.Set( 0.0 );
    }

    if ( LiftUp )
    {
      m_Lift.Set( 0.5 );
    }
    else if ( LiftDown )
    {
      m_Lift.Set( -0.5 );
    }
    else
    {
      m_Lift.Set( 0.0 );
    }
  }




  void TeleopInit() override {
    m_angleEncoder.Reset();
    m_clawEncoder.Update();
    m_leftencoder.Reset();
    m_rightencoder.Reset();
    m_balancePid.Reset();
    m_balancePid.SetSetpoint( 0.0 );
    m_ClawRotatePid.Reset();
    m_LiftLimitsSet = false;
    m_liftencoder.Reset();
    m_LiftHoldPid.Reset();
    m_LiftHoldPid.Reset();
  }



  void TeleopPeriodic() override {
    bool SelfBalanceEnable = false;//m_stick.GetYButton();
    bool ToggleClaw        = m_stick.GetXButtonPressed();
    //bool ToggleLift2       = m_stick.GetYButtonPressed();


    bool LiftUp            = m_logitechController.GetAButton();
    bool LiftDown          = m_logitechController.GetBButton();

    bool Lift2Up            = false; //m_stick.GetYButton();
    bool Lift2Down          = true; //m_stick.GetXButton();

    bool AngleUp           = m_logitechController.GetRightBumper();
    bool AngleDown         = m_logitechController.GetLeftBumper();


    bool ClawOpen           = false;//m_logitechController.GetAButton();
    bool ClawClose          = false;//m_logitechController.GetBButton();

    bool RotateClawCW      = m_logitechController.GetXButton();
    bool RotateClawCCW     = m_logitechController.GetYButton();



    // ------------------------------------------------------------------------
    //  DRIVE CONTROL
    // ------------------------------------------------------------------------
    if ( SelfBalanceEnable )
    {
      double pidValue = m_balancePid.Calculate( m_imu.GetRoll() );
      m_robotDrive.ArcadeDrive(-pidValue, 0.0);
    }
    else
    {
      m_robotDrive.ArcadeDrive(-m_stick.GetLeftY(), -m_stick.GetLeftX());
    }


    // ------------------------------------------------------------------------
    //  CLAW CONTROL
    // ------------------------------------------------------------------------
    if ( ToggleClaw )
    {
      if ( m_clawState == CLAW_STATE_OPEN )
      {
        m_clawState = CLAW_STATE_CLOSE;
      }
      else
      {
        m_clawState = CLAW_STATE_OPEN;
      }
    }

    if ( m_clawState == CLAW_STATE_OPEN )
    {
      m_clawSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
    }
    else if ( m_clawState == CLAW_STATE_CLOSE )
    {
      m_clawSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
    }
    else
    {
      m_clawSolenoid.Set(frc::DoubleSolenoid::Value::kOff);
    }

/*
    if ( ClawOpen )
    {
      m_clawSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
    }
    else if ( ClawClose )
    {
      m_clawSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
    }
    else
    {
      m_clawSolenoid.Set(frc::DoubleSolenoid::Value::kOff);
    }
    */

    /*
    if ( ToggleLift2 )
    {
      if ( m_lift2State == LIFT2_STATE_OPEN )
      {
        m_lift2State = LIFT2_STATE_CLOSE;
      }
      else
      {
        m_lift2State = LIFT2_STATE_OPEN;
      }
    }

    if ( m_lift2State == LIFT2_STATE_OPEN )
    {
      m_liftSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
    }
    else if ( m_lift2State == LIFT2_STATE_CLOSE )
    {
      m_liftSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
    }
    else
    {
      m_liftSolenoid.Set(frc::DoubleSolenoid::Value::kOff);
    }
    */

    if ( Lift2Up )
    {
      m_liftSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
    }
    else if ( Lift2Down )
    {
      m_liftSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
    }
    else
    {
      m_liftSolenoid.Set(frc::DoubleSolenoid::Value::kOff);
    }

    
    if ( RotateClawCW )
    {
      m_ClawRotate.Set( 0.5 );
    }
    else if ( RotateClawCCW )
    {
      m_ClawRotate.Set( -0.5 );
    }
    else
    {
      m_ClawRotate.Set( 0.0 );
    }

/*
    double CountsPerTurn = 13000.0;
    double ClawAngle = atan( m_stick.GetRightY() / m_stick.GetRightX() ) / ( 2.0 * 3.14159 );
    double mag = m_stick.GetRightY() * m_stick.GetRightY() + m_stick.GetRightX() * m_stick.GetRightX();
    frc::SmartDashboard::PutNumber("mag",    mag);
    frc::SmartDashboard::PutNumber("m_stick.GetRightY()",    m_stick.GetRightY());
    frc::SmartDashboard::PutNumber("m_stick.GetRightX()",    m_stick.GetRightX());
    if ( mag > 0.2 )
    {
      m_ClawRotatePid.SetSetpoint( ClawAngle * CountsPerTurn );
      double clawPower = m_ClawRotatePid.Calculate( m_clawEncoder.GetValue() );

      m_ClawRotate.Set( clawPower );
    }
    else
    {
      m_ClawRotate.Set( 0.0 );
    }
*/

    // ------------------------------------------------------------------------
    //  ANGLE CONTROL
    // ------------------------------------------------------------------------
    double linActRightValue = 0.0;
    double linActLeftValue  = 0.0;

    if( AngleDown )
    {
      if ( m_bottomLimitLeft.Get() )
      {
        linActLeftValue = 1.0;
      }
      else
      {
        m_angleEncoder.Reset();        
      }

      if ( m_bottomLimitRight.Get() )
      {
        linActRightValue = 1.0;
      }
      else
      {
        m_angleEncoder.Reset();        
      }

      if ( !m_bottomLimitLeft.Get() && !m_bottomLimitRight.Get() )
      {
        m_LiftLimitsSet = true;
      }
    }
    else if( AngleUp )
    {
      if ( m_LiftLimitsSet == true )
      {
        if ( m_angleEncoder.GetValue() > -1000 )
        {
          linActRightValue = -1.0;
          linActLeftValue  = -1.0;
        }
      }

    }

    frc::SmartDashboard::PutNumber("AngleDown",        AngleDown);
    frc::SmartDashboard::PutNumber("AngleUp",          AngleUp);
    frc::SmartDashboard::PutNumber("linActRightValue", linActRightValue);
    frc::SmartDashboard::PutNumber("linActLeftValue",  linActLeftValue);


    m_LinActRight.Set( linActRightValue );
    m_LinActLeft.Set( linActLeftValue );


    // ------------------------------------------------------------------------
    //  LIFT CONTROL
    // ------------------------------------------------------------------------
    double liftMotorValue = 0.0;
    int currentliftposition = m_liftencoder.Get();

    if ( LiftUp )
    {
      m_liftState = LIFT_STATE_RAISE;
    }
    else if ( LiftDown )
    {
      m_liftState = LIFT_STATE_LOWER;
    }
    else
    {
      m_liftState = LIFT_STATE_HOLD;
    }

    switch ( m_liftState )
    {
      case LIFT_STATE_RAISE:
      {
        m_LiftHoldPid.SetSetpoint( currentliftposition );
        if ( m_liftLimitTop.Get() )
        {
          m_liftState = LIFT_STATE_HOLD;
        }
        else
        {
          liftMotorValue = 0.5;
        }
        break;
      }

      case LIFT_STATE_LOWER:
      {
        m_LiftHoldPid.SetSetpoint( currentliftposition );
        Lift2Up = false;
        Lift2Down = true;
        if ( m_liftLimitBot.Get() )
        {
          m_liftState = LIFT_STATE_HOLD;
        }
        else
        {
          liftMotorValue = -0.5;
          if( currentliftposition < 40000) //slow down lift descent near bottom
          {
            liftMotorValue = -0.1;
          }          
        }
        break;
      }

      default:
      case LIFT_STATE_HOLD:
      {
        if ( !m_liftLimitBot.Get() && !m_liftLimitTop.Get() )
        {
          frc::SmartDashboard::PutNumber("liftsetpoint",  m_LiftHoldPid.GetSetpoint() );
          double motorliftcalc = m_LiftHoldPid.Calculate( currentliftposition );
          motorliftcalc = std::clamp( motorliftcalc, -0.5, 0.5);
          frc::SmartDashboard::PutNumber("calculated lift", motorliftcalc  );
          liftMotorValue = motorliftcalc;
        }
        if ( m_liftLimitBot.Get() )
        {
          m_liftencoder.Reset();
          liftMotorValue = 0;
        }
        if ( m_liftLimitTop.Get() )
        {
          m_LiftHoldPid.SetSetpoint( currentliftposition ); // prevents bouncing off the top switch
          liftMotorValue = 0;
          Lift2Up = true;
          Lift2Down = false;
        }
        // liftMotorValue = 0;
        break;
      }
    }
    m_Lift.Set( liftMotorValue );

    int liftEncValue = m_liftencoder.Get();
    frc::SmartDashboard::PutNumber("liftEncValue",  liftEncValue);

  }


  void AutonomousInit() override {
    m_imu.Reset();
    m_autoTimer.Stop();
    m_autoTimer.Reset();
    m_autoTimer.Start();
    m_liftencoder.Reset();
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
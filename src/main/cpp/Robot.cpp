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

    void Set( int value )
    {
      m_value = value;
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




class AccelerationLimiter {
  public:
    AccelerationLimiter( 
      double accelerationLimit
      ) :
      m_accelerationLimit{accelerationLimit}
    {
      m_currentValue = 0.0;
    }

    void Reset( )
    {
      m_currentValue = 0.0;
    }

    double Set( double value )
    {
      if ( ( value >= 0.0 && m_currentValue >= 0.0 && value < m_currentValue ) || 
           ( value <= 0.0 && m_currentValue <= 0.0 && value > m_currentValue ) )
      {
        // Do not apply acceleration limit for slowing down.
        m_currentValue = value;
      }
      else if ( ( value <= 0.0 && m_currentValue > 0.0 ) ||
                ( value >= 0.0 && m_currentValue < 0.0 ) )
      {
        // Allow dropping value to zero if changing directions.
        m_currentValue = 0.0;
      }
      else
      {
        double delta = std::clamp( value - m_currentValue, -m_accelerationLimit, m_accelerationLimit );
        m_currentValue += delta;
      }
      m_currentValue = std::clamp( m_currentValue, -1.0, 1.0 );
      return m_currentValue;
    }

  private:
    double m_accelerationLimit;
    double m_currentValue;
};




class WPI_VictorSPX_AccelerationLimited {
  public:
    WPI_VictorSPX_AccelerationLimited( 
      int    canId, 
      double accelerationLimit
      ) :
      m_motorController{canId},
      m_accelerationLimiter{accelerationLimit}
    { }

    void Set( double value )
    {
      m_motorController.Set( m_accelerationLimiter.Set( value ) );
    }

  private:
    WPI_VictorSPX       m_motorController;
    AccelerationLimiter m_accelerationLimiter;
};



// Function from Kauailab Website:
//   https://pdocs.kauailabs.com/navx-mxp/examples/mxp-io-expansion/
int GetDioChannelFromPin( int io_pin_number ) {
    static const int MAX_NAVX_MXP_DIGIO_PIN_NUMBER  = 9;
    static const int NUM_ROBORIO_ONBOARD_DIGIO_PINS = 10;
    int roborio_channel = 0;
    if ( io_pin_number < 0 ) {
        throw std::runtime_error("Error:  navX-MXP I/O Pin #");
    }

    if ( io_pin_number > MAX_NAVX_MXP_DIGIO_PIN_NUMBER ) {
        throw std::runtime_error("Error:  Invalid navX-MXP Digital I/O Pin #");
    }
    roborio_channel = io_pin_number + NUM_ROBORIO_ONBOARD_DIGIO_PINS +
                      (io_pin_number > 3 ? 4 : 0);

    return roborio_channel;
}



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

  WPI_TalonSRX                       m_frontleftMotor { kFLMotorCanId };
  WPI_TalonSRX                       m_rearleftMotor  { kRLMotorCanId };
  WPI_TalonSRX                       m_rearrightMotor { kRRMotorCanId };
  WPI_TalonSRX                       m_frontrightMotor{ kFRMotorCanId };
  frc::PneumaticsControlModule       m_pcm            { kPcmCanId     };
  WPI_VictorSPX_AccelerationLimited  m_LinActRight    { kLinActACanId,    0.05 };
  WPI_VictorSPX_AccelerationLimited  m_LinActLeft     { kLinActBCanId,    0.05 };
  WPI_VictorSPX_AccelerationLimited  m_Lift           { kLiftCanId,       0.05 };
  WPI_VictorSPX_AccelerationLimited  m_ClawRotate     { kClawRotateCanId, 0.05 };


  // Pneumatics
  frc::DoubleSolenoid m_clawSolenoid{ kPcmCanId, frc::PneumaticsModuleType::CTREPCM, 0, 1 };
  frc::DoubleSolenoid m_liftSolenoid{ kPcmCanId, frc::PneumaticsModuleType::CTREPCM, 2, 3 };


  // Digial I/O
  frc::Encoder      m_leftencoder     { GetDioChannelFromPin(2), GetDioChannelFromPin(3) };
  frc::Encoder      m_rightencoder    { GetDioChannelFromPin(0), GetDioChannelFromPin(1) };
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
  frc::XboxController m_DriveController{1};
  frc::XboxController m_AuxController  {0};


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

  double kAngleHoldP{ 0.01 };
  double kAngleHoldI{ 0.00 };
  double kAngleHoldD{ 0.00 };
  frc::PIDController m_AngleHoldPid{ kAngleHoldP, kAngleHoldI, kAngleHoldD };



  // Other
  frc::Timer m_autoTimer;

  const std::string    kAutoNameDefault = "Default";
  const std::string    kAutoDrive       = "Drive";
  frc::SendableChooser<std::string> m_autoChooser;

  bool m_AngleLimitsSet = false;
  bool m_WinchEncoderCalibrated = false;

  AccelerationLimiter m_DriveSpeedAccelerationLimiter   { 0.1 };
  AccelerationLimiter m_DriveRotationAccelerationLimiter{ 0.1 };

  double m_angleSetpoint     = 0;
  double m_winchLiftSetpoint = 0;

  bool m_PrevManualWinchLiftControlEnabled = false;
  bool m_PrevManualAngleControlEnabled     = false;


  enum claw_state_e
  {
    CLAW_STATE_HOLD,
    CLAW_STATE_OPEN,
    CLAW_STATE_CLOSE
  } m_clawState = CLAW_STATE_HOLD;


  enum pneumatic_lift_state_e
  {
    PNEUMATIC_LIFT_STATE_HOLD,
    PNEUMATIC_LIFT_STATE_EXTEND,
    PNEUMATIC_LIFT_STATE_CONTRACT
  } m_pneumaticLiftState = PNEUMATIC_LIFT_STATE_HOLD;





  enum lift_state_e
  {
    LIFT_STATE_HOLD,
    LIFT_STATE_STARTING_CONFIG,
    LIFT_STATE_GROUND,
    LIFT_STATE_CUBE,
    LIFT_STATE_DRIVING,
    LIFT_STATE_LOW_GOAL,
    LIFT_STATE_HIGH_GOAL
  } m_liftState = LIFT_STATE_GROUND;









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
    frc::SmartDashboard::PutNumber("m_liftencoder",      m_liftencoder.Get());
    frc::SmartDashboard::PutNumber("m_liftLimitTop",     m_liftLimitTop.Get());
    frc::SmartDashboard::PutNumber("m_liftLimitBot",     m_liftLimitBot.Get());

    frc::SmartDashboard::PutNumber("m_angleSetpoint",     m_angleSetpoint);
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
    bool AngleUpR    = m_DriveController.GetRightBumper();
    bool AngleUpL    = m_DriveController.GetLeftBumper();
    bool AngleDownR  = m_DriveController.GetYButton();
    bool AngleDownL  = m_DriveController.GetXButton();
    bool LiftUp      = m_DriveController.GetAButton();
    bool LiftDown    = m_DriveController.GetBButton();

    //m_robotDrive.ArcadeDrive(-m_DriveController.GetLeftY(), -m_DriveController.GetLeftX());

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
    m_AngleLimitsSet = false;
    m_WinchEncoderCalibrated = false;
    m_liftencoder.Reset();
    m_LiftHoldPid.Reset();
    m_LiftHoldPid.Reset();
    m_AngleHoldPid.Reset();
    m_liftState = LIFT_STATE_HOLD;
    m_angleSetpoint     = m_angleEncoder.GetValue();
    m_winchLiftSetpoint = m_liftencoder.Get();
  }



  void TeleopPeriodic() override {
    bool SelfBalanceEnable = m_DriveController.GetYButton();
    bool ToggleClaw        = m_DriveController.GetXButtonPressed();

    double RotateClawValue = m_AuxController.GetLeftX();

    // Maunal Winch Lift Control
    bool ManualWinchLiftUp              = m_DriveController.GetRightBumper() || m_AuxController.GetRightBumper();
    bool ManualWinchLiftDown            = m_DriveController.GetLeftBumper() || m_AuxController.GetLeftBumper();
    bool ManualWinchLiftControlEnabled  = ManualWinchLiftUp || ManualWinchLiftDown;
    bool ManualWinchLiftControlEnded    = m_PrevManualWinchLiftControlEnabled && !ManualWinchLiftControlEnabled;
    m_PrevManualWinchLiftControlEnabled = ManualWinchLiftControlEnabled;

    // Maunal Angle Control
    double const triggerThreshold = 0.1;
    bool ManualAngleUp   = ( m_DriveController.GetRightTriggerAxis() > triggerThreshold ) || 
                           ( m_AuxController.GetRightTriggerAxis()   > triggerThreshold );
    bool ManualAngleDown = ( m_DriveController.GetLeftTriggerAxis()  > triggerThreshold ) || 
                           ( m_AuxController.GetLeftTriggerAxis()    > triggerThreshold );
    bool ManualAngleControlEnabled  = ManualAngleUp || ManualAngleDown;
    bool ManualAngleControlEnded    = m_PrevManualAngleControlEnabled && !ManualAngleControlEnabled;
    m_PrevManualAngleControlEnabled = ManualAngleControlEnabled;


    // Lift Presets
    if ( m_AuxController.GetBButtonPressed() ) 
    {
      m_liftState = LIFT_STATE_GROUND;
    }
    else if ( m_AuxController.GetBButtonPressed() ) 
    {
      m_liftState = LIFT_STATE_DRIVING;
    }
    else if ( m_AuxController.GetXButtonPressed() ) 
    {
      m_liftState = LIFT_STATE_LOW_GOAL;
    }
    else if ( m_AuxController.GetYButtonPressed() ) 
    {
      m_liftState = LIFT_STATE_HIGH_GOAL;
    }
    else if ( m_AuxController.GetAButtonPressed() ) 
    {
      m_liftState = LIFT_STATE_CUBE;
    }
    else if ( m_AuxController.GetBackButtonPressed() ) 
    {
      m_liftState = LIFT_STATE_STARTING_CONFIG;
    }

    switch ( m_liftState )
    {
      case LIFT_STATE_STARTING_CONFIG:
      {
        m_angleSetpoint     = -1400;
        m_winchLiftSetpoint = 0;
        m_liftState          = LIFT_STATE_HOLD;
        break;
      }      

      case LIFT_STATE_DRIVING:
      {
        m_angleSetpoint     = -400;
        m_winchLiftSetpoint = 0;
        m_liftState          = LIFT_STATE_HOLD;
        break;
      }

      case LIFT_STATE_LOW_GOAL:
      {
        m_angleSetpoint     = -700;
        m_winchLiftSetpoint = 110000;
        m_liftState          = LIFT_STATE_HOLD;
        break;
      }

      case LIFT_STATE_HIGH_GOAL:
      {
        m_angleSetpoint     = -700;
        m_winchLiftSetpoint = 180000;
        m_liftState          = LIFT_STATE_HOLD;
        break;
      }

      case LIFT_STATE_CUBE:  
      {
        m_angleSetpoint     = -70;
        m_winchLiftSetpoint = 0;
        m_liftState          = LIFT_STATE_HOLD;
        break;
      }

      case LIFT_STATE_GROUND:  
      {
        m_angleSetpoint     = 2000;
        m_winchLiftSetpoint = 0;
        m_liftState          = LIFT_STATE_HOLD;
        break;
      }

      case LIFT_STATE_HOLD:  
      default:
      {
        break;
      }
    }

    // Manual Winch Lift Control
    int WinchLiftEncoderValue = m_liftencoder.Get();
    // Offset setpoint by ( 1 / P ) for max speed.
    double const WinchEncoderValueOffset = ( 1.0 / kLiftHoldP );

    if ( ManualWinchLiftUp )
    {
      if ( m_liftLimitTop.Get() )
      {
        m_winchLiftSetpoint = WinchLiftEncoderValue;
      }
      else
      {
        m_winchLiftSetpoint = WinchLiftEncoderValue + WinchEncoderValueOffset; 
      }
    }
    else if ( ManualWinchLiftDown )
    {
      m_winchLiftSetpoint = WinchLiftEncoderValue - WinchEncoderValueOffset;
    }
    else if ( ManualWinchLiftControlEnded )
    {
      // If manual control has ended stay at the current position.
      m_winchLiftSetpoint = WinchLiftEncoderValue;
    }

    if ( !ManualWinchLiftDown && m_liftLimitTop.Get() )
    {
      m_winchLiftSetpoint = WinchLiftEncoderValue;
    }

    m_LiftHoldPid.SetSetpoint( m_winchLiftSetpoint );

    // Manual Angle Control
    int AngleEncoderValue = m_angleEncoder.GetValue();
    // Offset setpoint by ( 1 / P ) for max speed.
    double const AngleEncoderValueOffset = ( 1 / kAngleHoldP );

    if ( ManualAngleUp )
    {
      m_angleSetpoint = AngleEncoderValue - AngleEncoderValueOffset;
    }
    else if ( ManualAngleDown )
    {
      m_angleSetpoint = AngleEncoderValue +  AngleEncoderValueOffset;
    }
    else if ( ManualAngleControlEnded )
    {
      // If manual control has ended stay at the current position.
      m_angleSetpoint = AngleEncoderValue;
    }
    m_AngleHoldPid.SetSetpoint( m_angleSetpoint );



    if ( m_winchLiftSetpoint   > 170000 || 
         WinchLiftEncoderValue > 170000 || 
         m_liftLimitTop.Get() )
    {
      m_pneumaticLiftState = PNEUMATIC_LIFT_STATE_EXTEND;
    }
    else
    {
      m_pneumaticLiftState = PNEUMATIC_LIFT_STATE_CONTRACT;
    }



    // ------------------------------------------------------------------------
    //  DRIVE CONTROL
    // ------------------------------------------------------------------------
    if ( SelfBalanceEnable )
    {
      double pidValue = m_balancePid.Calculate( m_imu.GetRoll() );
      m_robotDrive.ArcadeDrive(-pidValue, 0.0);
      m_DriveSpeedAccelerationLimiter.Reset();
      m_DriveRotationAccelerationLimiter.Reset();
    }
    else
    {
      double xSpeed    = m_DriveSpeedAccelerationLimiter.Set( -m_DriveController.GetLeftY() );
      double zRotation = m_DriveRotationAccelerationLimiter.Set( -m_DriveController.GetRightX() );
      m_robotDrive.ArcadeDrive( xSpeed, zRotation );
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

    if ( fabs( RotateClawValue ) > 0.1 )
    {
      m_ClawRotate.Set( RotateClawValue );
    }
    else
    {
      m_ClawRotate.Set( 0.0 );
    }


    // ------------------------------------------------------------------------
    //  ANGLE CONTROL
    // ------------------------------------------------------------------------
    double linActRightValue = 0.0;
    double linActLeftValue  = 0.0;
    int angleValue = -m_angleEncoder.GetValue();
    double motorVal = m_AngleHoldPid.Calculate( m_angleEncoder.GetValue() );

    linActLeftValue = motorVal;
    linActRightValue = motorVal;
    
    if ( !m_bottomLimitRight.Get() )
    {
      linActRightValue = std::clamp( linActRightValue, -1.0, 0.0 );
      m_angleEncoder.Reset();        
    }

    if ( !m_bottomLimitLeft.Get() )
    {
      linActLeftValue = std::clamp( linActLeftValue, -1.0, 0.0 );
      m_angleEncoder.Reset();        
    }

    if ( !m_bottomLimitLeft.Get() && !m_bottomLimitRight.Get() )
    {
      m_AngleLimitsSet = true;
    }

    if ( angleValue < -1000 || m_AngleLimitsSet == false )
    {
      linActRightValue = std::clamp( linActRightValue, 0.0, 1.0 );
      linActLeftValue  = std::clamp( linActLeftValue,  0.0, 1.0 );
    }


    frc::SmartDashboard::PutNumber("ANGLE_motorVal",    motorVal);
    frc::SmartDashboard::PutNumber("ANGLE_m_AngleHoldPid.GetSetpoint()",    m_AngleHoldPid.GetSetpoint());
    frc::SmartDashboard::PutNumber("ANGLE_m_angleEncoder.GetValue()",    m_angleEncoder.GetValue());
    frc::SmartDashboard::PutNumber("ANGLE_linActRightValue",    linActRightValue);
    frc::SmartDashboard::PutNumber("ANGLE_linActLeftValue",     linActLeftValue);


    m_LinActRight.Set( linActRightValue );
    m_LinActLeft.Set( linActLeftValue );


    // ------------------------------------------------------------------------
    //  WINCH LIFT CONTROL
    // ------------------------------------------------------------------------
    double liftMotorValue = 0.0;
    int currentliftposition = m_liftencoder.Get();

    if ( m_liftLimitBot.Get() )
    {
      m_WinchEncoderCalibrated = true;
    }

    liftMotorValue = m_LiftHoldPid.Calculate( currentliftposition );
    liftMotorValue = std::clamp( liftMotorValue, -0.7, 0.5);

    if ( m_liftLimitBot.Get() )
    {
      m_liftencoder.Reset();
      liftMotorValue = std::clamp( liftMotorValue, 0.0, 0.5);
    }
    if ( m_liftLimitTop.Get() )
    {
      //m_LiftHoldPid.SetSetpoint( currentliftposition ); // prevents bouncing off the top switch
      liftMotorValue = std::clamp( liftMotorValue, -0.5, 0.0);
    }

    if ( m_WinchEncoderCalibrated == false )
    {
      liftMotorValue = -0.2;
    }

    frc::SmartDashboard::PutNumber("LIFT_m_LiftHoldPid.GetSetpoint()",  m_LiftHoldPid.GetSetpoint());
    frc::SmartDashboard::PutNumber("LIFT_currentliftposition",          currentliftposition);
    frc::SmartDashboard::PutNumber("LIFT_liftMotorValue",               liftMotorValue  );

    m_Lift.Set( liftMotorValue );


    // ------------------------------------------------------------------------
    //  PNEUMATIC LIFT CONTROL
    // ------------------------------------------------------------------------
    if ( m_pneumaticLiftState == PNEUMATIC_LIFT_STATE_EXTEND )
    {
      m_liftSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
    }
    else if ( m_pneumaticLiftState == PNEUMATIC_LIFT_STATE_CONTRACT )
    {
      m_liftSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
    }
    else
    {
      m_liftSolenoid.Set(frc::DoubleSolenoid::Value::kOff);
    }



  }




  void Subsystem_AngleUpdate() {

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

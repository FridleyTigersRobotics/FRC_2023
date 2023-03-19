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
#include <numbers>
#include <frc/AnalogGyro.h>
#include <frc/Encoder.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/geometry/Pose2d.h>


#define ANGLE_DEBUG ( 1 )
#define LIFT_DEBUG  ( 1 )
#define DRIVE_DEBUG ( 1 )

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
  static constexpr int kFLMotorCanId    {  1 };
  static constexpr int kRLMotorCanId    {  2 };
  static constexpr int kRRMotorCanId    {  3 };
  static constexpr int kFRMotorCanId    {  4 };
  static constexpr int kPcmCanId        {  6 };
  static constexpr int kLinActACanId    {  8 };
  static constexpr int kLinActBCanId    {  9 };
  static constexpr int kLiftCanId       { 10 };
  static constexpr int kClawRotateCanId { 11 };

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
  frc::Encoder      m_leftEncoder     { GetDioChannelFromPin(0), GetDioChannelFromPin(1) };
  frc::Encoder      m_rightEncoder    { GetDioChannelFromPin(2), GetDioChannelFromPin(3) };
  frc::DigitalInput m_bottomLimitLeft { 4 };
  frc::DigitalInput m_bottomLimitRight{ 5 };
  frc::DigitalInput m_angleTopLimit   { 7 };
  frc::DigitalInput m_liftLimitTop    { 0 };
  frc::DigitalInput m_liftLimitBot    { 9 };
  frc::Encoder      m_liftencoder     { 1, 8 };


  // Analog I/O
  DualChannelAnalogEncoder m_angleEncoder{ 0, 1 };
  DualChannelAnalogEncoder m_clawEncoder { 2, 3 };


  // SPI Devices
  AHRS m_imu{ frc::SPI::Port::kMXP };


  // User Input Devices
  frc::XboxController m_DriveController{ 1 };
  frc::XboxController m_AuxController  { 0 };


  // Drive System
  frc::MotorControllerGroup m_leftMotors { m_frontleftMotor,  m_rearleftMotor  };
  frc::MotorControllerGroup m_rightMotors{ m_frontrightMotor, m_rearrightMotor };
  frc::DifferentialDrive    m_robotDrive { m_leftMotors, m_rightMotors };
  frc::SimpleMotorFeedforward<units::meters> m_feedforward{ 1_V, 2.5_V / 1_mps };
  frc2::PIDController m_leftPIDController { 2.0, 0.0, 0.0 };
  frc2::PIDController m_rightPIDController{ 2.0, 0.0, 0.0 };
  static constexpr units::meter_t kTrackWidth        { 0.5358_m };
  static constexpr double         kWheelRadius       { 0.0762 };  // meters
  static constexpr int            kEncoderResolution { 360 };     // pulses puer rev
  static constexpr double         kDistancePerPulse  { 2.0 * std::numbers::pi * kWheelRadius / kEncoderResolution };


  frc::DifferentialDriveKinematics m_kinematics{kTrackWidth};
  frc::DifferentialDriveOdometry m_odometry{
    m_imu.GetRotation2d(), 
    units::meter_t{m_leftEncoder.GetDistance()},
    units::meter_t{m_rightEncoder.GetDistance()}
  };

  static constexpr units::meters_per_second_t  kMaxSpeed        { 3.0_mps };
  static constexpr units::radians_per_second_t kMaxAngularSpeed { std::numbers::pi }; 
  frc::SlewRateLimiter<units::scalar>          m_speedLimiter   { 6 / 1_s };
  frc::SlewRateLimiter<units::scalar>          m_rotLimiter     { 6 / 1_s };


  // PIDs
  double const kRotateGyroP{ 0.0200 };
  double const kRotateGyroI{ 0.0011 };
  double const kRotateGyroD{ 0.0000 };
  frc2::PIDController m_rotateGyroPid{ kRotateGyroP, kRotateGyroI, kRotateGyroD };

  double const kRotateLimeP{ 0.3000 };
  double const kRotateLimeI{ 0.0051 };
  double const kRotateLimeD{ 0.0000 };
  frc2::PIDController m_rotateLimePid{ kRotateLimeP, kRotateLimeI, kRotateLimeD };

  double kBalanceP{ 0.050 };
  double kBalanceI{ 0.001 };
  double kBalanceD{ 0.000 };
  frc::PIDController m_balancePid{ kBalanceP, kBalanceI, kBalanceD };

  double kLiftHoldP{ 8.0e-5 };
  double kLiftHoldI{ 4.0e-4 };
  double kLiftHoldD{ 0.00 };
  frc::PIDController m_LiftHoldPid{ kLiftHoldP, kLiftHoldI, kLiftHoldD };

  double kAngleHoldP{ 0.008 };
  double kAngleHoldI{ 0.00 };
  double kAngleHoldD{ 0.00 };
  frc::PIDController m_AngleHoldPid{ kAngleHoldP, kAngleHoldI, kAngleHoldD };


  // Subsystem States
  bool   m_WinchEncoderCalibrated { false };
  bool   m_AngleLimitsSet         { false };
  double m_angleSetpoint          { 0 };
  double m_winchLiftSetpoint      { 0 };
  double m_rotateClawValue        { 0 };

  static constexpr int kAngleEncoderTopSafetyValue{ -1500 };
  static constexpr int kAngleEncoderTopValue      { -1300 };

  enum claw_state_e
  {
    CLAW_STATE_HOLD,
    CLAW_STATE_OPEN,
    CLAW_STATE_CLOSE
  } m_clawState = CLAW_STATE_HOLD;

  typedef enum lift_position_e
  {
    LIFT_POSITION_HOLD,
    LIFT_POSITION_STARTING_CONFIG,
    LIFT_POSITION_GROUND,
    LIFT_POSITION_CUBE,
    LIFT_POSITION_DRIVING,
    LIFT_POSITION_LOW_GOAL,
    LIFT_POSITION_HIGH_GOAL,
    LIFT_POSITION_HIGH_GOAL_AUTONOMOUS
  } lift_position_t;


  // Autonomous
  const std::string kAutoNameDefault          { "DO NOTHING"           };
  const std::string kAutoDrive                { "Drive"                };
  const std::string kAutoPlaceAndDrive        { "PlaceAndDrive"        };
  const std::string kAutoDriveAndBalance      { "DriveAndBalance"      };
  const std::string kAutoPlaceAndBalance      { "PlaceAndBalance"      };
  const std::string kAutoPlaceDriveAndBalance { "PlaceDriveAndBalance" };

  frc::SendableChooser<std::string> m_autoChooser;
  frc::Timer   m_autoTimer;
  std::string  m_autoSelected { kAutoNameDefault };
  int          m_autoSequence { 0 };
  bool         m_initState    { true };
  unsigned int m_autoState    { 0 }; 
  double       m_initialAngle { 0 };

  double m_prevAngle             { 0 };
  double m_currentAngle          { 0 };
  int m_atRotateSetpointCount    { 0 };
  units::meter_t m_startDistance { 0 };

  // Other
  std::shared_ptr<nt::NetworkTable> limelightNetworkTable;

  AccelerationLimiter m_DriveSpeedAccelerationLimiter   { 0.1 };
  AccelerationLimiter m_DriveRotationAccelerationLimiter{ 0.1 };

  bool m_PrevManualWinchLiftControlEnabled { false };
  bool m_PrevManualAngleControlEnabled     { false };
  bool m_reverseDrive                      { false };







 public:

  void SetLiftSetpoints(lift_position_t liftPosition);

  // Subsytem Updates
  void Subsystem_AngleUpdate();
  void Subsystem_LiftUpdate();
  void Subsystem_ClawUpdate();

  void Drivetrain_Stop();
  void Drivetrain_SetSpeeds(const frc::DifferentialDriveWheelSpeeds& speeds);
  void Drivetrain_Drive(units::meters_per_second_t xSpeed,
                        units::radians_per_second_t rot);
  void Drivetrain_UpdateOdometry();

  // Auto Sequences
  bool RunDriveAuto();
  bool RunPlaceAuto();
  bool RunBalanceAuto( bool forwards );

  // Auto functions
  bool Balance();
  bool RotateDegrees( double angle );
  bool DriveForDistance( units::meters_per_second_t speed, units::meter_t distance, units::time::second_t maxTime );


  void RobotInit() override {
    limelightNetworkTable = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotors.SetInverted( true );
    m_leftEncoder.SetReverseDirection( true ); 
    m_leftEncoder.SetDistancePerPulse( kDistancePerPulse );
    m_rightEncoder.SetDistancePerPulse( kDistancePerPulse );

    // Autonomous Chooser
    m_autoChooser.SetDefaultOption( kAutoNameDefault,     kAutoNameDefault   );
    m_autoChooser.AddOption       ( kAutoDrive,           kAutoDrive         );
    m_autoChooser.AddOption       ( kAutoPlaceAndDrive,   kAutoPlaceAndDrive );
    m_autoChooser.AddOption       ( kAutoDriveAndBalance, kAutoDriveAndBalance );
    m_autoChooser.AddOption       ( kAutoPlaceAndBalance, kAutoPlaceAndBalance );
    m_autoChooser.AddOption       ( kAutoPlaceDriveAndBalance, kAutoPlaceDriveAndBalance );

    frc::SmartDashboard::PutData("Auto Modes", &m_autoChooser);

    m_LiftHoldPid.SetIntegratorRange( -0.1, 0.1 ); //stops integrator wind-up
    m_LiftHoldPid.SetTolerance( 100 );
    m_rotateLimePid.SetIntegratorRange( -2.0, 2.0 ); //stops integrator wind-up
    m_balancePid.SetIntegratorRange( -0.4, 0.4 ); //stops integrator wind-up
  }


  void RobotPeriodic() override {
    m_angleEncoder.Update();
    m_clawEncoder.Update();
    frc::Pose2d pose = m_odometry.GetPose();

  #if DRIVE_DEBUG
    frc::SmartDashboard::PutNumber("DRIVE_leftEncoderRaw",      m_leftEncoder.Get());   
    frc::SmartDashboard::PutNumber("DRIVE_rightEncoderRaw",     m_rightEncoder.Get());
    frc::SmartDashboard::PutNumber("DRIVE_LeftDistance",      m_leftEncoder.GetDistance());   
    frc::SmartDashboard::PutNumber("DRIVE_RightDistance",     m_rightEncoder.GetDistance());
    frc::SmartDashboard::PutNumber("IMU_ROLL",     m_imu.GetRoll());
    frc::SmartDashboard::PutNumber("POSE_X",     (double)pose.X());
    frc::SmartDashboard::PutNumber("POSE_Y",     (double)pose.Y());
  #endif
  }


  void TestInit() override {
    m_angleEncoder.Reset();
    m_leftEncoder.Reset();
    m_rightEncoder.Reset();
    m_balancePid.Reset();
    m_balancePid.SetSetpoint( 0.0 );
    m_liftencoder.Reset();
  }


  void TestPeriodic() override {
    bool AngleUpR    = m_DriveController.GetRightBumper();
    bool AngleUpL    = m_DriveController.GetLeftBumper();
    bool AngleDownR  = m_DriveController.GetYButton();
    bool AngleDownL  = m_DriveController.GetXButton();
    bool LiftUp      = m_DriveController.GetAButton();
    bool LiftDown    = m_DriveController.GetBButton();

    ////m_robotDrive.ArcadeDrive(-m_DriveController.GetLeftY(), -m_DriveController.GetLeftX());

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
    m_liftencoder.Reset();
    m_leftEncoder.Reset();
    m_rightEncoder.Reset();
    m_balancePid.Reset();
    m_balancePid.SetSetpoint( 0.0 );
    m_LiftHoldPid.Reset();
    m_AngleHoldPid.Reset();
    m_rotateLimePid.Reset();
    m_angleSetpoint          = m_angleEncoder.GetValue();
    m_winchLiftSetpoint      = m_liftencoder.Get();
    m_WinchEncoderCalibrated = false;
    m_AngleLimitsSet         = false;
    m_rotateClawValue        = 0;
    m_reverseDrive           = false;
    m_prevAngle              = 0.0;
    m_currentAngle           = 0.0;
    m_odometry.ResetPosition(
      m_imu.GetRotation2d(),
      units::meter_t{m_leftEncoder.GetDistance()},
      units::meter_t{m_rightEncoder.GetDistance()}, 
      frc::Pose2d{});
  }



  void AutonomousInit() override {
    TeleopInit(); 
    m_imu.Reset();
    m_autoTimer.Stop();
    m_autoTimer.Reset();
    m_autoTimer.Start();
    m_autoSelected = m_autoChooser.GetSelected();
    fmt::print("Auto selected: {}\n", m_autoSelected);

    m_autoSequence = 0;
    m_initState = true;
    m_autoState = 0; 
    m_initialAngle = 0;
    m_rotateGyroPid.SetTolerance(3.0);
    m_atRotateSetpointCount = 0;

    // Need to assume we are in the starting configuration.
    m_angleEncoder.Set( kAngleEncoderTopValue );
  }




  void TeleopPeriodic() override {
    bool SelfBalanceEnable  = false;//m_DriveController.GetYButton();
    bool SwapDriveDirection = m_DriveController.GetYButtonPressed();
    bool ToggleClaw         = m_DriveController.GetXButtonPressed();

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

    m_rotateClawValue = m_AuxController.GetLeftX();

    if ( SwapDriveDirection )
    {
      m_reverseDrive = !m_reverseDrive;
    }

    // Lift Presets
    lift_position_t liftPosition = LIFT_POSITION_HOLD;
    if ( m_AuxController.GetBButtonPressed() ) 
    {
      liftPosition = LIFT_POSITION_DRIVING;
    }
    else if ( m_AuxController.GetXButtonPressed() ) 
    {
      liftPosition = LIFT_POSITION_LOW_GOAL;
    }
    else if ( m_AuxController.GetYButtonPressed() ) 
    {
      liftPosition = LIFT_POSITION_HIGH_GOAL;
    }
    else if ( m_AuxController.GetAButtonPressed() ) 
    {
      liftPosition = LIFT_POSITION_CUBE;
    }
    else if ( m_AuxController.GetBackButtonPressed() ) 
    {
      liftPosition = LIFT_POSITION_STARTING_CONFIG;
    }

    SetLiftSetpoints( liftPosition );


    // ------------------------------------------------------------------------
    //  DRIVE CONTROL
    // ------------------------------------------------------------------------
    Drivetrain_UpdateOdometry();

    if ( SelfBalanceEnable )
    {
      double pidValue = m_balancePid.Calculate( m_imu.GetRoll() );
      m_robotDrive.ArcadeDrive(-pidValue, 0.0);
      m_DriveSpeedAccelerationLimiter.Reset();
      m_DriveRotationAccelerationLimiter.Reset();
    }
    else
    {
      if ( false )
      {
        double yVal = m_DriveController.GetLeftY();
        double xVal = m_DriveController.GetRightX();

        if ( fabs(yVal) < 0.2 )
        {
          yVal = 0.0;
        }
        else
        {
          if ( yVal > 0.0 )
          {
            yVal = ( yVal - 0.2 ) * 1.2;
            yVal = yVal * yVal;
          }
          else
          {
            yVal = ( yVal + 0.2 ) * 1.2;
            yVal = -yVal * yVal;
          }
        }

        if ( fabs(xVal) < 0.2 )
        {
          xVal = 0.0;
        }
        else
        {
           //double rotationSlowPercent = fabs( m_DriveController.GetLeftY() );

          if ( xVal > 0.0 )
          {
            

            xVal = ( xVal - 0.2 ) * 1.8;
            xVal = xVal * xVal;
          }
          else
          {
            xVal = ( xVal + 0.2 ) * 1.8;
            xVal = -xVal * xVal;
          }
        }

        auto xSpeed = -m_speedLimiter.Calculate(yVal) * kMaxSpeed;
        auto rot    = -m_rotLimiter.Calculate(xVal) * kMaxAngularSpeed;

        frc::SmartDashboard::PutNumber("DRIVE_xSpeed", (double)xSpeed  );
        frc::SmartDashboard::PutNumber("DRIVE_rot", (double)rot  );

        Drivetrain_Drive( xSpeed, rot );
      }
      else
      {
        double yVal = -m_DriveController.GetLeftY();
        double xVal = -m_DriveController.GetRightX();
        double rotationLimiter = std::clamp( 1.6 - fabs(yVal), 0.0, 1.0 );
        xVal *= rotationLimiter;

        double xSpeed    = m_DriveSpeedAccelerationLimiter.Set( yVal );
        double zRotation = m_DriveRotationAccelerationLimiter.Set( xVal );

        frc::SmartDashboard::PutNumber("DRIVE_xSpeed", (double)xSpeed  );
        frc::SmartDashboard::PutNumber("DRIVE_rot", (double)zRotation  );

        if ( m_reverseDrive )
        {
          xSpeed *= -1.0;
        }
        m_robotDrive.ArcadeDrive( xSpeed, zRotation );
      }
    }



    // ------------------------------------------------------------------------
    // Manual Winch Lift Control
    // ------------------------------------------------------------------------
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
      if ( m_liftLimitBot.Get() )
      {
        m_winchLiftSetpoint = WinchLiftEncoderValue;
      }  
      else
      {
        m_winchLiftSetpoint = WinchLiftEncoderValue - WinchEncoderValueOffset;
      }
    }
    else if ( ManualWinchLiftControlEnded )
    {
      // If manual control has ended stay at the current position.
      m_winchLiftSetpoint = WinchLiftEncoderValue;
    }

    m_LiftHoldPid.SetSetpoint( m_winchLiftSetpoint );

    // ------------------------------------------------------------------------
    // Manual Angle Control
    // ------------------------------------------------------------------------
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


    // ------------------------------------------------------------------------
    //  Claw Control
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


    // ------------------------------------------------------------------------
    //  Subsystem Updates
    // ------------------------------------------------------------------------
    Subsystem_AngleUpdate();
    Subsystem_LiftUpdate();
    Subsystem_ClawUpdate();

  }



  void AutonomousPeriodic() override {
    Drivetrain_UpdateOdometry();

    if (m_autoSelected == kAutoDrive) 
    {
      RunDriveAuto( );
    }
    else if (m_autoSelected == kAutoPlaceAndDrive)
    {
      if ( m_autoSequence == 0 )
      {
        if ( RunPlaceAuto( ) == true )
        {
          m_autoSequence++;
          m_autoState = 0;
          m_initState = true;
        }
      }
      /*else if ( m_autoSequence == 1 )
      {
        if ( RunDriveAuto( ) == true )
        {
          m_autoSequence++;
          m_autoState = 0;
          m_initState = true;
        }
      }*/
      else
      {
        Drivetrain_Stop();
      }
    }
    else if (m_autoSelected == kAutoDriveAndBalance)
    {
      RunBalanceAuto( false );
    }
    else if (m_autoSelected == kAutoPlaceAndBalance)
    {
      if ( m_autoSequence == 0 )
      {
        if ( RunBalanceAuto( false ) == true )
        {
          m_autoSequence++;
          m_autoState = 0;
          m_initState = true;
        }

        /*if ( RunPlaceAuto( ) == true )
        {
          m_autoSequence++;
          m_autoState = 0;
          m_initState = true;
        }*/
      }
      /*else if ( m_autoSequence == 1 )
      {
        if ( RunBalanceAuto( false ) == true )
        {
          m_autoSequence++;
          m_autoState = 0;
          m_initState = true;
        }
      }*/
      else
      {
        Drivetrain_Stop();
      }
    }
    else if (m_autoSelected == kAutoPlaceDriveAndBalance)
    {
      if ( m_autoSequence == 0 )
      {
        if ( RunPlaceAuto( ) == true )
        {
          m_autoSequence++;
          m_autoState = 0;
          m_initState = true;
        }
      }
      else if ( m_autoSequence == 1 )
      {
        if ( RunDriveAuto( ) == true )
        {
          m_autoSequence++;
          m_autoState = 0;
          m_initState = true;
        }
      }
      else if ( m_autoSequence == 2 )
      {
        if ( RunBalanceAuto( true ) == true )
        {
          m_autoSequence++;
          m_autoState = 0;
          m_initState = true;
        }
      }
      else
      {
        Drivetrain_Stop();
      }
    }
    else
    {
      Drivetrain_Stop();
    }

    m_LiftHoldPid.SetSetpoint( m_winchLiftSetpoint );
    m_AngleHoldPid.SetSetpoint( m_angleSetpoint );


    Subsystem_AngleUpdate();
    Subsystem_LiftUpdate();
    Subsystem_ClawUpdate();

  }
};



void Robot::SetLiftSetpoints(
  lift_position_t liftPosition
)
{
  switch ( liftPosition )
  {
    case LIFT_POSITION_STARTING_CONFIG:
    {
      m_angleSetpoint     = -1400;
      m_winchLiftSetpoint = 0;
      break;
    }      

    case LIFT_POSITION_DRIVING:
    {
      m_angleSetpoint     = -400;
      m_winchLiftSetpoint = 0;
      break;
    }

    case LIFT_POSITION_LOW_GOAL:
    {
      m_angleSetpoint     = -900;
      m_winchLiftSetpoint = 110000;
      break;
    }

    case LIFT_POSITION_HIGH_GOAL:
    {
      m_angleSetpoint     = -620;
      m_winchLiftSetpoint = 190000;
      break;
    }

    case LIFT_POSITION_HIGH_GOAL_AUTONOMOUS:
    {
      m_angleSetpoint     = -720;
      m_winchLiftSetpoint = 190000;
      break;
    }

    case LIFT_POSITION_CUBE:  
    {
      m_angleSetpoint     = -70;
      m_winchLiftSetpoint = 0;
      break;
    }

    case LIFT_POSITION_GROUND:  
    {
      m_angleSetpoint     = 2000;
      m_winchLiftSetpoint = 0;
      break;
    }

    case LIFT_POSITION_HOLD:  
    default:
    {
      break;
    }
  }
}



void Robot::Drivetrain_SetSpeeds(const frc::DifferentialDriveWheelSpeeds& speeds) {
  const auto leftFeedforward = m_feedforward.Calculate(speeds.left);
  const auto rightFeedforward = m_feedforward.Calculate(speeds.right);
  const double leftOutput = m_leftPIDController.Calculate(
      m_leftEncoder.GetRate(), speeds.left.value());
  const double rightOutput = m_rightPIDController.Calculate(
      m_rightEncoder.GetRate(), speeds.right.value());

  frc::SmartDashboard::PutNumber("DRIVE_leftRate", m_leftEncoder.GetRate()  );
  frc::SmartDashboard::PutNumber("DRIVE_leftSpeed", speeds.left.value()  );
  frc::SmartDashboard::PutNumber("DRIVE_rightRate", m_rightEncoder.GetRate()  );
  frc::SmartDashboard::PutNumber("DRIVE_rightSpeed", speeds.right.value()  );
  frc::SmartDashboard::PutNumber("DRIVE_leftOutput", leftOutput  );
  frc::SmartDashboard::PutNumber("DRIVE_leftFeedforward", (double)leftFeedforward  );
  frc::SmartDashboard::PutNumber("DRIVE_rightOutput", rightOutput  );
  frc::SmartDashboard::PutNumber("DRIVE_rightFeedforward", (double)rightFeedforward  );

  m_leftMotors.SetVoltage(units::volt_t{leftOutput} + leftFeedforward);
  m_rightMotors.SetVoltage(units::volt_t{rightOutput} + rightFeedforward);
  m_robotDrive.FeedWatchdog();
}

void Robot::Drivetrain_Drive(units::meters_per_second_t xSpeed,
                             units::radians_per_second_t rot) {
  Drivetrain_SetSpeeds(m_kinematics.ToWheelSpeeds({xSpeed, 0_mps, rot}));
}

void Robot::Drivetrain_Stop() {
  m_robotDrive.StopMotor();
}


void Robot::Drivetrain_UpdateOdometry() {
  m_odometry.Update(m_imu.GetRotation2d(),
                    units::meter_t{m_leftEncoder.GetDistance()},
                    units::meter_t{m_rightEncoder.GetDistance()});
}



void Robot::Subsystem_ClawUpdate() {
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

  if ( fabs( m_rotateClawValue ) > 0.1 )
  {
    m_ClawRotate.Set( m_rotateClawValue );
  }
  else
  {
    m_ClawRotate.Set( 0.0 );
  }
}




void Robot::Subsystem_LiftUpdate() {
  double liftMotorValue = 0.0;
  int WinchLiftEncoderValue = m_liftencoder.Get();

  // TODO : Test this limit is adequate.
  if ( m_angleEncoder.GetValue() < -1100 )  
  {
    m_winchLiftSetpoint = 0;
    m_LiftHoldPid.SetSetpoint( m_winchLiftSetpoint );
  }

  if ( m_liftLimitBot.Get() )
  {
    m_WinchEncoderCalibrated = true;
  }

  double liftPidValue = m_LiftHoldPid.Calculate( WinchLiftEncoderValue );
  liftMotorValue = liftPidValue;
  liftMotorValue = std::clamp( liftMotorValue, -0.7, 0.9);

  if ( m_liftLimitBot.Get() )
  {
    m_liftencoder.Reset();
    liftMotorValue = std::clamp( liftMotorValue, 0.0, 0.9);
  }
  if ( m_liftLimitTop.Get() )
  {
    //m_LiftHoldPid.SetSetpoint( WinchLiftEncoderValue ); // prevents bouncing off the top switch
    liftMotorValue = std::clamp( liftMotorValue, -0.5, 0.0);
  }

  if ( m_WinchEncoderCalibrated == false )
  {
    liftMotorValue = -0.2;
  }

#if LIFT_DEBUG
  frc::SmartDashboard::PutNumber("LIFT0_m_LiftHoldPid.GetSetpoint()",  m_LiftHoldPid.GetSetpoint());
  frc::SmartDashboard::PutNumber("LIFT1_WinchLiftEncoderValue",        WinchLiftEncoderValue);
  frc::SmartDashboard::PutNumber("LIFT2_m_winchLiftSetpoint",          m_winchLiftSetpoint  );
  frc::SmartDashboard::PutNumber("LIFT3_m_liftLimitTop.Get()",         m_liftLimitTop.Get()  );
  frc::SmartDashboard::PutNumber("LIFT4_liftMotorValue",               liftMotorValue  );
#endif

  //fmt::print( "{},{},{},{},{}\n", m_LiftHoldPid.GetPositionError(), m_angleEncoder.GetValue(), m_winchLiftSetpoint, m_LiftHoldPid.GetSetpoint(), WinchLiftEncoderValue, liftMotorValue );


  m_Lift.Set( liftMotorValue );

  // TODO : Test this limit is adequate.
  if ( ( WinchLiftEncoderValue > 170000 ) &&
       ( m_angleEncoder.GetValue() < -500 ) &&
       ( m_angleEncoder.GetValue() > -1100 ) )
  {
    // Extend
    m_liftSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
  }
  else
  {
    // Contract
    m_liftSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
  }
}




void Robot::Subsystem_AngleUpdate() {
  double linActRightValue = 0.0;
  double linActLeftValue  = 0.0;
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

  if ( !m_AngleLimitsSet && !m_angleTopLimit.Get() )
  {
    m_angleEncoder.Set( kAngleEncoderTopValue );       
    m_angleSetpoint = m_angleEncoder.GetValue();
    m_AngleHoldPid.SetSetpoint( m_angleSetpoint );
  }

  if ( ( !m_bottomLimitLeft.Get() && !m_bottomLimitRight.Get() ) || 
       ( !m_angleTopLimit.Get() ) )
  {
    m_AngleLimitsSet = true;
  }

  if ( m_angleEncoder.GetValue() < kAngleEncoderTopSafetyValue || !m_angleTopLimit.Get() )
  {
    linActRightValue = std::clamp( linActRightValue, 0.0, 1.0 );
    linActLeftValue  = std::clamp( linActLeftValue,  0.0, 1.0 );
  }

  if ( fabs( m_AngleHoldPid.GetPositionError() ) < 70 )
  {
    linActRightValue = 0;
    linActLeftValue = 0;
  }

  m_LinActRight.Set( linActRightValue );
  m_LinActLeft.Set( linActLeftValue );

#if ANGLE_DEBUG
  frc::SmartDashboard::PutNumber("ANGLE_angleSetpoint",                m_angleSetpoint);
  frc::SmartDashboard::PutNumber("ANGLE_motorVal",                     motorVal);
  frc::SmartDashboard::PutNumber("ANGLE_m_AngleHoldPid.GetSetpoint()", m_AngleHoldPid.GetSetpoint());
  frc::SmartDashboard::PutNumber("ANGLE_m_angleEncoder.GetValue()",    m_angleEncoder.GetValue());
  frc::SmartDashboard::PutNumber("ANGLE_linActRightValue",             linActRightValue);
  frc::SmartDashboard::PutNumber("ANGLE_linActLeftValue",              linActLeftValue);
  frc::SmartDashboard::PutNumber("ANGLE_m_bottomLimitLeft",            m_bottomLimitLeft.Get());
  frc::SmartDashboard::PutNumber("ANGLE_m_bottomLimitRight",           m_bottomLimitRight.Get());
  frc::SmartDashboard::PutNumber("ANGLE_m_angleTopLimit",              m_angleTopLimit.Get());
#endif
}





  bool Robot::RotateDegrees( double angle )
  {
    double rotationSpeed = m_rotateGyroPid.Calculate( m_imu.GetAngle(), angle );

    if ( m_rotateGyroPid.AtSetpoint() )
    {
      m_atRotateSetpointCount++;
    }

    if ( m_atRotateSetpointCount > 5 )
    {
      Drivetrain_Stop();
      return true;
    }
    else
    {
      m_atRotateSetpointCount = 0;
      Drivetrain_Drive(0.0_mps, 0.5_rad_per_s);
      return false;
    }
  }



  bool Robot::DriveForDistance( units::meters_per_second_t speed, units::meter_t distance, units::time::second_t maxTime=5.0_s )
  {
    frc::Pose2d pose = m_odometry.GetPose();

    if ( ( ( ( speed > 0.0_mps ) && ( (pose.X() - m_startDistance) < distance ) ) ||
           ( ( speed < 0.0_mps ) && ( (pose.X() - m_startDistance) > distance ) ) ) &&
         ( m_autoTimer.Get() < maxTime ) )
    {
      Drivetrain_Drive( speed, 0.0_rad_per_s );
      return false;
    }
    else
    {
      Drivetrain_Stop();
      return true;
    }
  }



  bool Robot::RunBalanceAuto( bool forwards )
  {
    frc::Pose2d pose = m_odometry.GetPose();
    bool sequenceDone = false;
    bool stateDone = false;

    switch( m_autoState )
    {
      case 0:
      {
        units::meters_per_second_t speed = 0.0_mps;
        units::meter_t             dist  = 0.0_m;

        if ( m_initState )
        {
          m_autoTimer.Stop();
          m_autoTimer.Reset();
          m_autoTimer.Start();
          m_initialAngle = m_imu.GetAngle();
          m_startDistance = pose.X();
        }

        if ( forwards )
        {
          speed = 1.0_mps;
          dist  = 2.0_m;
        }
        else
        {
          speed = -1.0_mps;
          dist  = -1.6_m;
        }

        stateDone = DriveForDistance( speed, dist );
        break;
      }

      case 1:
      {
        if ( m_initState )
        {
          m_autoTimer.Stop();
          m_autoTimer.Reset();
          m_autoTimer.Start();
          m_initialAngle = m_imu.GetAngle();
          m_currentAngle = m_imu.GetRoll();
          m_prevAngle = m_currentAngle;
          m_startDistance = pose.X();
        }
        stateDone = Balance();
        fmt::print( "BalanceDone\n" );
        break;
      }


      case 2:
      {
        if ( m_initState )
        {
          m_autoTimer.Stop();
          m_autoTimer.Reset();
          m_autoTimer.Start();
          m_initialAngle = m_imu.GetAngle();
          m_currentAngle = m_imu.GetRoll();
          m_prevAngle = m_currentAngle;
          m_startDistance = pose.X();
        }
        stateDone = DriveForDistance( 1.5_mps, 0.15_m );
        break;
      }

      default:
      {
        Drivetrain_Stop();
        sequenceDone = true;

        // Done, do nothing
        break;
      }
    }

    if ( m_initState )
    {
      m_initState = false;
    }

    if ( stateDone )
    {
      //fmt::print( "stateDone {}\n", m_autoState );
      m_autoState++;
      m_initState = true;
    }

    return sequenceDone;
  }



  bool Robot::RunDriveAuto( )
  {
    bool sequenceDone = false;
    bool stateDone = false;

    switch( m_autoState )
    {
      case 0:
      {
        if ( m_initState )
        {
          m_autoTimer.Stop();
          m_autoTimer.Reset();
          m_autoTimer.Start();
          m_initialAngle = m_imu.GetAngle();
        }
        stateDone = DriveForDistance( -1.0_mps, -4.0_m, 8.0_s );
        break;
      }

      default:
      {
        Drivetrain_Stop();
        sequenceDone = true;

        // Done, do nothing
        break;
      }
    }

    if ( m_initState )
    {
      m_initState = false;
    }

    if ( stateDone )
    {
      //fmt::print( "stateDone {}\n", m_autoState );
      m_autoState++;
      m_initState = true;
    }

    return sequenceDone;
  }




  bool Robot::RunPlaceAuto( )
  {
    bool sequenceDone = false;
    bool stateDone = false;

    switch( m_autoState )
    {
      case 0:
      {
        if ( m_initState )
        {
          m_autoTimer.Stop();
          m_autoTimer.Reset();
          m_autoTimer.Start();
          m_initialAngle = m_imu.GetAngle();
        }
        SetLiftSetpoints( LIFT_POSITION_HIGH_GOAL_AUTONOMOUS );
        stateDone = ( m_autoTimer.Get() > (units::time::second_t)3.0 ) || m_liftLimitTop.Get() || (m_LiftHoldPid.AtSetpoint() && m_LiftHoldPid.GetSetpoint() > 1000);
        Drivetrain_Stop();
        break;
      }


      case 1:
      {
        if ( m_initState )
        {
          m_autoTimer.Stop();
          m_autoTimer.Reset();
          m_autoTimer.Start();
          m_initialAngle = m_imu.GetAngle();
        }

        double rotationSpeed = m_rotateLimePid.Calculate( limelightNetworkTable->GetNumber( "tx", 0.0 ) );
        SetLiftSetpoints( LIFT_POSITION_HIGH_GOAL_AUTONOMOUS );
        stateDone = m_autoTimer.Get() > (units::time::second_t)1.0;
        Drivetrain_Drive(0.0_mps, (units::radians_per_second_t)rotationSpeed);
        break;
      }

      case 2:
      {
        if ( m_initState )
        {
          m_autoTimer.Stop();
          m_autoTimer.Reset();
          m_autoTimer.Start();
          m_initialAngle = m_imu.GetAngle();
        }

        SetLiftSetpoints( LIFT_POSITION_HIGH_GOAL );
        stateDone = m_autoTimer.Get() > (units::time::second_t)0.4;
        Drivetrain_Stop();
        break;
      }

      case 3:
      {
        if ( m_initState )
        {
          m_autoTimer.Stop();
          m_autoTimer.Reset();
          m_autoTimer.Start();
          m_initialAngle = m_imu.GetAngle();
        }
        m_clawState = CLAW_STATE_OPEN;
        stateDone = m_autoTimer.Get() > (units::time::second_t)0.2;
        Drivetrain_Stop();
        break;
      }

      case 4:
      {
        if ( m_initState )
        {
          m_autoTimer.Stop();
          m_autoTimer.Reset();
          m_autoTimer.Start();
          m_initialAngle = m_imu.GetAngle();
        }
        SetLiftSetpoints( LIFT_POSITION_LOW_GOAL );
        stateDone = m_autoTimer.Get() > (units::time::second_t)0.5;
        Drivetrain_Stop();
        break;
      }

      case 5:
      {
        if ( m_initState )
        {
          m_autoTimer.Stop();
          m_autoTimer.Reset();
          m_autoTimer.Start();
          m_initialAngle = m_imu.GetAngle();
        }
        SetLiftSetpoints( LIFT_POSITION_DRIVING );
        stateDone = m_autoTimer.Get() > (units::time::second_t)0.5;
        Drivetrain_Stop();
        break;
      }

      default:
      {
        Drivetrain_Stop();
        sequenceDone = true;

        // Done, do nothing
        break;
      }
    }

    if ( m_initState )
    {
      m_initState = false;
    }

    if ( stateDone )
    {
      //fmt::print( "stateDone {}\n", m_autoState );
      m_autoState++;
      m_initState = true;
    }

    return sequenceDone; 
  }



bool Robot::Balance()
{
  bool   balanced = false;
  double angleChangeThreshold = 0.5;
  m_currentAngle = m_imu.GetRoll();


  if ( fabs(m_currentAngle) < 3.0  )
  {
    Drivetrain_Stop();
    //balanced = true;
  }
  else
  {
    if ( m_currentAngle > 0.0 )
    {
      if ( ( m_prevAngle - m_currentAngle ) > angleChangeThreshold )
      {
        Drivetrain_Stop();
        //balanced = true;
      }
      else
      {
        Drivetrain_Drive(1.0_mps, 0.0_rad_per_s);
      }
    }
    else
    {
      if ( ( m_prevAngle - m_currentAngle ) < -angleChangeThreshold )
      {
        Drivetrain_Stop();
        //balanced = true;
      }
      else
      {
        Drivetrain_Drive(-1.0_mps, 0.0_rad_per_s);
      }
    }
  }
  frc::SmartDashboard::PutNumber("BALANCE_currentAngle", m_currentAngle);
  frc::SmartDashboard::PutNumber("BALANCE_delta",    m_prevAngle - m_currentAngle);

  m_prevAngle = m_currentAngle;
  return balanced;
}



#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif

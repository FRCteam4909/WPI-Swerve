// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
  private static final double kWheelRadius = 0.0508;
  private static final int kEncoderResolution = 4096;

  // private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
  // private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared

  // private final SpeedController m_driveMotor;
  // private final SpeedController m_turningMotor;
  
  // private final Encoder m_driveEncoder = new Encoder(0, 1);
  // private final Encoder m_turningEncoder = new Encoder(2, 3);

  // private final PIDController m_drivePIDController = new PIDController(1, 0, 0);

  // private final ProfiledPIDController m_turningPIDController =
  //     new ProfiledPIDController(
  //         1,
  //         0,
  //         0,
  //         new TrapezoidProfile.Constraints(
  //             kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

  // Gains are for example purposes only - must be determined for your own robot!
  // private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);
  // private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

  private CANSparkMax m_driveMotorA;
  private CANSparkMax m_driveMotorB;
  private CANPIDController m_aPID;
  private CANPIDController m_bPID;

  private CANEncoder m_driveEncoderA;
  private CANEncoder m_driveEncoderB;
  private Encoder m_yawEncoder;

  // Gear ratio of first two pairs of gears
  // Yaw does not depend on the third pair
  final double GEAR_RATIO_12  =  (10.0/80.0) * (34.0/90.0);

  // Gear ratio of all three pairs of gears
  // Wheel speed depends on all three
  final double GEAR_RATIO_123 =  (10.0/80.0) * (34.0/90.0) * (82.0/21.0);

  final double kMotorP = 0.000090;
  final double kMotorI = 0.000001;
  final double kMotorD = 0.000090;
  final double kMotorIz = 100.0;
  final double kMotorFf = 0.000090;

  // To keep the motor closer to peek power, we limit the max output.
  // See torque/speed curves on https://motors.vex.com/
  final double kMotorMax = .7;
  final double kMotorMin = -.7;

  private PIDController pid_yaw;

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel ID for the drive motor.
   * @param turningMotorChannel ID for the turning motor.
   */
  public SwerveModule(
    int driveMotorChannelA,
    int driveMotorChannelB,
    int dioEncoderChanA,
    int dioEncoderChanB
  ) {
      
    // m_driveMotor = new PWMVictorSPX(driveMotorChannel);
    // m_turningMotor = new PWMVictorSPX(turningMotorChannel);

    m_driveMotorA = new CANSparkMax(driveMotorChannelA, MotorType.kBrushless);
    m_driveEncoderA = m_driveMotorA.getEncoder();
    m_driveMotorA.restoreFactoryDefaults();

    // @todo current limits on the motor
    // m_driveMotorA.setSmartCurrentLimit(limit)

    
    m_aPID = m_driveMotorA.getPIDController();
    if (driveMotorChannelA == 1) {
      m_aPID.setP(kMotorP);
      m_aPID.setI(kMotorI);
      m_aPID.setD(kMotorD);
      m_aPID.setIZone(kMotorIz);
      m_aPID.setFF(kMotorFf);
      m_aPID.setOutputRange(kMotorMin, kMotorMax);
    }
      
    m_driveMotorB = new CANSparkMax(driveMotorChannelB, MotorType.kBrushless);
    m_driveEncoderB = m_driveMotorB.getEncoder();
    m_driveMotorB.restoreFactoryDefaults();
    m_bPID = m_driveMotorB.getPIDController();
    if (driveMotorChannelA == 1) {
      m_bPID.setP(kMotorP);
      m_bPID.setI(kMotorI);
      m_bPID.setD(kMotorD);
      m_bPID.setIZone(kMotorIz);
      m_bPID.setFF(kMotorFf);
      m_bPID.setOutputRange(kMotorMin, kMotorMax);
    }

    m_yawEncoder = new Encoder(dioEncoderChanA, dioEncoderChanB, true, EncodingType.k4X);
    // final double kRad2RPM = 2 * Math.PI;
    final double kEncoderTicksPerRev = 128.0; //grayhill encoder
    final double kEncoderDegPerPulse = 360 / kEncoderTicksPerRev;
    m_yawEncoder.setDistancePerPulse(kEncoderDegPerPulse);

    // Create a PID to enforce specified module direction
    // See pid_yaw.calculate for an explanation of how the PID value is used
    // We attempted to set the kp near use but found it broke the shuffleboard
    // pid calibarion tooling.
    pid_yaw = new PIDController(2, 0, 0);

    // When values outside this range are fed into the setpoint of the PID
    // the requested value is mapped onto this range.
    pid_yaw.enableContinuousInput(-180.0, 180.0);

    // // Set the distance per pulse for the drive encoder. We can simply use the
    // // distance traveled for one rotation of the wheel divided by the encoder
    // // resolution.
    // m_driveEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);

    // // Set the distance (in this case, angle) per pulse for the turning encoder.
    // // This is the the angle through an entire rotation (2 * wpi::math::pi)
    // // divided by the encoder resolution.
    // m_turningEncoder.setDistancePerPulse(2 * Math.PI / kEncoderResolution);

    // // Limit the PID Controller's input range between -pi and pi and set the input
    // // to be continuous.
    // m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    double aRPM = m_driveEncoderA.getVelocity();
    double bRPM = m_driveEncoderB.getVelocity();
    return new SwerveModuleState(getWheelVelocityMPS(aRPM, bRPM), new Rotation2d(m_yawEncoder.get()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, new Rotation2d(m_yawEncoder.get()));

    SmartDashboard.putNumber("Desired Speed", state.speedMetersPerSecond);
    SmartDashboard.putNumber("Desired Angle", state.angle.getDegrees());



    // PID is being used to convert the yaw error into the RPM to get there. For example,
    // if the encoder is currently at 90 degrees and desired is 180 degrees, the error
    // is 90 degrees initially. With a kP value of 2, this would yield an RPM of 180. As
    // the error decreases, the RPM decreases until eventually the error is zero and thus
    // the RPM is zero. See the constructor for the PID values.
    double pidCalculatedYawRPM = pid_yaw.calculate(m_yawEncoder.getDistance(), state.angle.getDegrees());

    // Given the desired wheel and yaw RPM, calculate the motor speeds
    // necessary to achieve them
    var desiredMotorSpeedsRPM = getMotorSpeedsRPM(state.speedMetersPerSecond, pidCalculatedYawRPM);

    // Set the reference speeds (setpoints) in the two PIDs
    m_aPID.setReference(desiredMotorSpeedsRPM.a, ControlType.kVelocity);
    m_bPID.setReference(desiredMotorSpeedsRPM.b, ControlType.kVelocity);

    // // Calculate the drive output from the drive PID controller.
    // final double driveOutput =
    //     m_drivePIDController.calculate(m_driveEncoder.getRate(), state.speedMetersPerSecond);

    // final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    // // Calculate the turning motor output from the turning PID controller.
    // final double turnOutput =
    //     m_turningPIDController.calculate(m_turningEncoder.get(), state.angle.getRadians());

    // final double turnFeedforward =
    //     m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    // m_driveMotor.setVoltage(driveOutput + driveFeedforward);
    // m_turningMotor.setVoltage(turnOutput + turnFeedforward);
  }
  // get wheel translation in RPM
  public double motorRPMsToWheelRPM(double aMotorRPM, double bMotorRPM) {
    // Translation is calculated as half the difference of a and b,
    // adjusted by gear ratio. Translation is dependent on all three
    // pairs of gears. The differential pinion generates translation 
    // as a function of the speed of the top and bottom differential gears.
    double wheelRPM = ((aMotorRPM - bMotorRPM) / 2) * GEAR_RATIO_123 ;
    return wheelRPM;
  }

  // get module yaw in RPM
  public double motorRPMsToModuleYawRPM(double aMotorRPM, double bMotorRPM) {
    // Yaw is calculated as the average of a and b, adjusted by gear ratio
    // Yaw does not depend on the third pair of gears, thus GEAR_RATIO_12
    // only includes the first two gear reductions.
    double moduleYawRPM = ((aMotorRPM + bMotorRPM) / 2) * GEAR_RATIO_12;
    return moduleYawRPM;
  }

  // convert desired translation and yaw RPMs to motor RPMs
  public MotorRPMs getMotorSpeedsRPM(double wheelRPM, double yawRPM) {
    double a = (yawRPM / GEAR_RATIO_12) + (wheelRPM / GEAR_RATIO_123);
    double b = (yawRPM / GEAR_RATIO_12) - (wheelRPM / GEAR_RATIO_123);
    return new MotorRPMs(a, b);
  }

  public double getWheelVelocityMPS(double aMotorRPM, double bMotorRPM) {

    double wheelRPM = motorRPMsToWheelRPM(aMotorRPM, bMotorRPM);

    final double kInchesToMeters = .0254;
    final double kWheelDiameterInches = 3.0;
    final double kSecsPerMinute = 60;
    final double kRPMtoMPS = (kWheelDiameterInches * Math.PI * kInchesToMeters) / kSecsPerMinute;

    return wheelRPM * kRPMtoMPS;
  }
}

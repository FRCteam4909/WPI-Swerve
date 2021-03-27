// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain {
  public static final double kMaxSpeed = 3.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

  final double kInchesToMeters = .0254;
  final double kHalfWheelBaseWidthInches  = 15.0;
  final double kHalfWheelBaseWidthMeters = kHalfWheelBaseWidthInches * kInchesToMeters;

  final double kHalfWheelBaseLengthInches = 15.0;
  final double kHalfWheelBaseLengthMeters = kHalfWheelBaseLengthInches * kInchesToMeters;

  private final Translation2d m_frontLeftLocation  = new Translation2d( kHalfWheelBaseWidthMeters,  kHalfWheelBaseLengthMeters);
  private final Translation2d m_frontRightLocation = new Translation2d( kHalfWheelBaseWidthMeters, -kHalfWheelBaseLengthMeters);
  private final Translation2d m_backLeftLocation   = new Translation2d(-kHalfWheelBaseWidthMeters,  kHalfWheelBaseLengthMeters);
  private final Translation2d m_backRightLocation  = new Translation2d(-kHalfWheelBaseWidthMeters, -kHalfWheelBaseLengthMeters);

  // @todo make theese constants somewhere
  public final SwerveModule m_frontLeft  = new SwerveModule(1, 2, 0, 1);
  // private final SwerveModule m_frontRight = new SwerveModule(3, 4, 2, 3);
  // private final SwerveModule m_backLeft   = new SwerveModule(5, 6, 4, 5);
  // private final SwerveModule m_backRight  = new SwerveModule(7, 8, 6, 7);

  // private final AnalogGyro m_gyro = new AnalogGyro(0);

  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  // private final SwerveDriveOdometry m_odometry =
  //     new SwerveDriveOdometry(m_kinematics, m_gyro.getRotation2d());

  public Drivetrain() {
    // m_gyro.reset();
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   */
  public void drive(double xSpeed, double ySpeed, double rot) {
    var swerveModuleStates = m_kinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    // m_frontRight.setDesiredState(swerveModuleStates[1]);
    // m_backLeft.setDesiredState(swerveModuleStates[2]);
    // m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  // /** Updates the field relative position of the robot. */
  // public void updateOdometry() {
  //   m_odometry.update(
  //       m_gyro.getRotation2d(),
  //       m_frontLeft.getState(),
  //       m_frontRight.getState(),
  //       m_backLeft.getState(),
  //       m_backRight.getState());
  // }
}

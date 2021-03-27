// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private final Joystick m_controller = new Joystick(0);
  private final Drivetrain m_swerve = new Drivetrain();

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  @Override
  public void robotInit() {
    // NetworkTable table = NetworkTableInstance.getDefault().getTable("MyTable");
    // table.getEntry("")
    SmartDashboard.putBoolean("Override", false);
    SmartDashboard.putBoolean("ZERO", false);
    
  }
  @Override
  public void teleopInit() {
    m_swerve.m_frontLeft.m_yawEncoder.reset(); // for debugging testing as the encoder is not absolute
  }

  // @Override
  // public void autonomousPeriodic() {
  //   driveWithJoystick(false);
  //   m_swerve.updateOdometry();
  // }

  @Override
  public void teleopPeriodic() {
    driveWithJoystick(true);
  }

  private void driveWithJoystick(boolean fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    double xSpeed = -m_controller.getY();
        // -m_xspeedLimiter.calculate(m_controller.getY(GenericHID.Hand.kLeft))
        //     * frc.robot.Drivetrain.kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    double ySpeed = -m_controller.getX();
        // -m_yspeedLimiter.calculate(m_controller.getX(GenericHID.Hand.kLeft))
        //     * frc.robot.Drivetrain.kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    double rot = -m_controller.getZ();
        // -m_rotLimiter.calculate()
        //     * frc.robot.Drivetrain.kMaxAngularSpeed;

    if (Math.abs(rot) < .1) {
      rot = 0;
    }
    if (Math.abs(xSpeed) < .1) {
      xSpeed = 0;
    }
    if (Math.abs(ySpeed) < .1) {
      ySpeed = 0;
    }

    

    if (SmartDashboard.getBoolean("Override", false)) {
      xSpeed = SmartDashboard.getNumber("xSpeed", 0);
      ySpeed = SmartDashboard.getNumber("ySpeed", 0);
      rot    = SmartDashboard.getNumber("rot",    0);
    }

    if (SmartDashboard.getBoolean("ZERO", false)) {
      SmartDashboard.putBoolean("ZERO", false);
      xSpeed = 0;
      ySpeed = 0;
      rot    = 0;
    }

    SmartDashboard.putNumber("xSpeed", xSpeed);
    SmartDashboard.putNumber("ySpeed", ySpeed);
    SmartDashboard.putNumber("rot",    rot);

    SmartDashboard.putNumber("Encoder Angle", m_swerve.m_frontLeft.m_yawEncoder.getDistance());

    
    m_swerve.drive(xSpeed, ySpeed, rot);
  }
}

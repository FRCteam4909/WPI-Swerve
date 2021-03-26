// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class Robot extends TimedRobot {
  private final Joystick m_controller = new Joystick(0);
  private final Drivetrain m_swerve = new Drivetrain();

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  private ShuffleboardTab sb_tab_drive;
  private NetworkTableEntry sb_drive_js_x, sb_drive_js_y, sb_drive_js_z, sb_drive_enable;

  @Override
  public void robotInit() {
    sb_tab_drive = Shuffleboard.getTab("Drive");
    sb_drive_js_x = sb_tab_drive.add("js x", 0).withSize(128, 128).getEntry();
    sb_drive_js_y = sb_tab_drive.add("js y", 0).getEntry();
    sb_drive_js_z = sb_tab_drive.add("js z", 0).getEntry();
    sb_drive_enable = sb_tab_drive.add("enable", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();
  }

  @Override
  public void autonomousPeriodic() {
    driveWithJoystick(false);
    m_swerve.updateOdometry();
  }

  @Override
  public void teleopPeriodic() {
    driveWithJoystick(true);
  }

  private void driveWithJoystick(boolean fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    final var xSpeed =
        -m_xspeedLimiter.calculate(m_controller.getX())
            * frc.robot.Drivetrain.kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var ySpeed =
        -m_yspeedLimiter.calculate(m_controller.getY())
            * frc.robot.Drivetrain.kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var rot =
        -m_rotLimiter.calculate(m_controller.getZ())
            * frc.robot.Drivetrain.kMaxAngularSpeed;
    
    sb_drive_js_x.setDouble(xSpeed);
    sb_drive_js_y.setDouble(ySpeed);
    sb_drive_js_z.setDouble(rot);

    if (sb_drive_enable.getBoolean(false)) {
      m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative); 
    }
  }
}

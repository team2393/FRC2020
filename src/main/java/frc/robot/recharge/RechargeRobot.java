/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.BasicRobot;
import frc.robot.recharge.ctrlpanel.ColorSensor;
import frc.robot.recharge.ctrlpanel.ControlWheel;
import frc.robot.recharge.ctrlpanel.ManualWheelSpeed;
import frc.robot.recharge.ctrlpanel.RotateToColor;
import frc.robot.recharge.ctrlpanel.RotateWheel;
import frc.robot.recharge.drivetrain.DriveByJoystick;
import frc.robot.recharge.drivetrain.DriveToPosition;
import frc.robot.recharge.drivetrain.DriveTrain;
import frc.robot.recharge.drivetrain.TurnToHeading;
import frc.robot.recharge.led.LEDStrip;

/** Robot for 'Infinite Recharge' - R!$E2geTHeR#2020
 */
public class RechargeRobot extends BasicRobot
{  
  private final DriveTrain drive_train = new DriveTrain();
  
  private final CommandBase drive_by_joystick = new DriveByJoystick(drive_train);
  private final DriveToPosition drive_to_position = new DriveToPosition(drive_train);
  private final TurnToHeading turn_to_heading = new TurnToHeading(drive_train);
  private final CommandBase reset_drivetrain = new InstantCommand(drive_train::reset);
  private final CommandBase shift_low = new InstantCommand(() -> drive_train.setGear(false));
  private final CommandBase shift_high = new InstantCommand(() -> drive_train.setGear(true));

  // TODO Add encoders to DriveTrain
  // TODO Command to drive to distance and heading (PID)
  // TODO Trajectory: Create
  // TODO Use simple position and heading PID to follow trajectory
  // TODO Kinematics to track current 'pose',  https://docs.wpilib.org/en/latest/docs/software/kinematics-and-odometry/differential-drive-odometry.html
  // TODO RamseteCommand to follow trajectory

  // TODO Command to drive left/right based on vision info (in network tables, set by pi)
  // TODO Grabber, shooter, ...

  private final ColorSensor color_sensor = new ColorSensor();

  private final ControlWheel fortune = new ControlWheel();
  private final Command manual_wheel = new ManualWheelSpeed(fortune);
  private final Command rotate_wheel = new RotateWheel(fortune, 3);
  private final Command rotate_to_color = new RotateToColor(fortune);
  
  private final LEDStrip led_strip = new LEDStrip();
  
  @Override
  public void robotInit()
  {
    // pcm.clearAllPCMStickyFaults();
    
    // Bind buttons to actions (only active in teleop)
    // Pressing 'A' enables manual wheel control (and stops auto rotation)
    OI.enable_wheel.whenPressed(manual_wheel);
    // Pressing 'B' turns wheel automatically, then re-enables manual control
    OI.autorotate_wheel.whenPressed(rotate_wheel.andThen(() -> manual_wheel.schedule()));
    // Pressing 'X' turns wheel to the desired color
    OI.rotate_to_color.whenPressed(rotate_to_color.andThen(() -> manual_wheel.schedule()));

    // Manual shifting
    // Note that DriveByJoystick will automatically shift,
    // so while we can invoke 'shift_high' via button,
    // we would automatically shift back low when standing still.
    OI.shift_low.whenActive(shift_low);
    OI.shift_high.whenPressed(shift_high);

    // Place some commands on dashboard
    SmartDashboard.putData("Reset Drive", reset_drivetrain);
  }
  
  @Override
  public void disabledInit()
  {
    super.disabledInit();
    led_strip.idle();
  }

  @Override
  public void teleopInit()
  {
    super.teleopInit();
    manual_wheel.schedule();
    drive_by_joystick.schedule();
  }
  
  @Override
  public void teleopPeriodic()
  {
    //final double direction = OI.getDirection();
    final double direction = SmartDashboard.getNumber("Direction", 0) / 160;
    led_strip.indicateDirection(direction);    
  }
  
  @Override
  public void autonomousInit()
  {
    super.autonomousInit();
    drive_to_position.schedule();
    // turn_to_heading.schedule();
  }
  @Override
  public void autonomousPeriodic()
  {
    led_strip.rainbow();
  
    // Every 3 seconds, toggle between two positions
    long test_index = (System.currentTimeMillis() / 3000) % 2;
    double test_pos_meters = 0.5;
    drive_to_position.setDesiredPosition(test_index * test_pos_meters);

    // double test_degrees = 45;
    // turn_to_heading.setDesiredHeading(test_index * test_degrees);
  }
}

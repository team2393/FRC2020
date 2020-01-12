/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.BasicRobot;
import frc.robot.recharge.ctrlpanel.ColorSensor;
import frc.robot.recharge.ctrlpanel.ControlWheel;
import frc.robot.recharge.ctrlpanel.ManualWheelSpeed;
import frc.robot.recharge.ctrlpanel.RotateToColor;
import frc.robot.recharge.ctrlpanel.RotateWheel;
import frc.robot.recharge.drivetrain.DriveTrain;
import frc.robot.recharge.led.LEDStrip;

/** Robot for 'Infinite Recharge' - R!$E2geTHeR#2020
 */
public class RechargeRobot extends BasicRobot
{  
  private final DriveTrain drive_train = new DriveTrain();
  private final Command shift_low = new InstantCommand(() -> drive_train.setGear(false));
  private final Command shift_high = new InstantCommand(() -> drive_train.setGear(true));

  // TODO Command to drive by joystick
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
    // Bind buttons to actions (only active in teleop)
    // Pressing 'A' enables manual wheel control (and stops auto rotation)
    OI.enable_wheel.whenPressed(manual_wheel);
    // Pressing 'B' turns wheel automatically, then re-enables manual control
    OI.autorotate_wheel.whenPressed(rotate_wheel.andThen(() -> manual_wheel.schedule()));
    // Pressing 'X' turns wheel to the desired color
    OI.rotate_to_color.whenPressed(rotate_to_color.andThen(() -> manual_wheel.schedule()));

    OI.shift_low.whenActive(shift_low);
    OI.shift_high.whenPressed(shift_high);
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
  }
  
  @Override
  public void autonomousPeriodic()
  {
    led_strip.rainbow();
  }

  @Override
  public void teleopPeriodic()
  {
    //final double direction = OI.getDirection();
   final double direction = SmartDashboard.getNumber("Direction", 0) / 160;
    led_strip.indicateDirection(direction);
    drive_train.drive(OI.getSpeed(), OI.getDirection());
    
    // pcm.clearAllPCMStickyFaults();

    if (Math.abs(OI.getSpeed()) < 0.1)
      shift_low.schedule();
  }
}

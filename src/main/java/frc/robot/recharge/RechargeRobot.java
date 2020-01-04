/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.BasicRobot;
import frc.robot.recharge.ctrlpanel.ControlWheel;
import frc.robot.recharge.ctrlpanel.ManualWheelSpeed;
import frc.robot.recharge.ctrlpanel.RotateWheel;

/** Robot for 'Infinite Recharge' - R!$E2geTHeR#2020
 */
public class RechargeRobot extends BasicRobot
{
  // TODO Basic Drivetrain class for the motors
  // TODO Command to drive by joystick
  // TODO Add encoders to DriveTrain
  // TODO Command to drive to distance and heading (PID)
  // TODO Trajectory: Create, follow
  // TODO Camera (on pi)
  // TODO Vision processing (on pi)
  // TODO Command to drive left/right based on vision info (in network tables, set by pi)
  // TODO Lift, grabber, pusher, climber, ...

  private final ControlWheel fortune = new ControlWheel(RobotMap.CONTROL_PANEL_WHEEL);
  private final Command manual_wheel = new ManualWheelSpeed(fortune);
  private final Command rotate_wheel = new RotateWheel(fortune, 3);

  @Override
  public void robotInit()
  {
    // Bind buttons to actions (only active in teleop)
    OI.enable_wheel.whenPressed(manual_wheel);
    OI.autorotate_wheel.whenPressed(rotate_wheel);
  }

  @Override
  public void teleopInit()
  {
    super.teleopInit();
  }
}

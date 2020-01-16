/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.drivetrain;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.recharge.OI;

/** Manually control speed and rotation via joystick */
public class DriveByJoystick extends CommandBase 
{
  // Results of basic drive test:
  // Minimum voltage to move: 0.3 V
  //
  // Voltage  Speed [m/s]
  //  2.8      0.7
  //  7.75     2
  // 10        2.5
  // 11.2      3
  private final SimpleMotorFeedforward feed_forward = new SimpleMotorFeedforward(0.3, 3.8);
  private final DriveTrain drive_train;

  public DriveByJoystick(DriveTrain drive_train) 
  {
    this.drive_train = drive_train;
    addRequirements(drive_train);
  }

  @Override
  public void execute()
  {
    // Test feed forward:
    // Use speed stick to request +-1 Volt.
    // Tweak a little with rotation stick to allow turning
    double voltage = feed_forward.calculate(OI.getSpeed());
    double left = voltage   + OI.getDirection();
    double right = -voltage + OI.getDirection();
    // Issue: Reports motor watchdog errors, briefly stops motors
    // TODO Check how often motors are set().
    // TODO Disable motor safety for testing.
    // TODO Set both main and follower motors
    drive_train.driveVoltage(left, right);

    // Normal joystick usage
    // drive_train.drive(OI.getSpeed(), OI.getDirection());
  }

  @Override
  public void end(final boolean interrupted)
  {
    drive_train.drive(0, 0);
  }
}

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
  //min0.3-------2.8v---0.7m/s....10v---2.5m/s...11.2---3m/s  7.75v  2m/s
  private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0.3, 3.8);
  private final DriveTrain drive_train;

  public DriveByJoystick(DriveTrain drive_train) 
  {
    this.drive_train = drive_train;
    addRequirements(drive_train);
  }

  @Override
  public void execute()
  {
    double voltage = ff.calculate(OI.getSpeed());
    drive_train.driveVoltage(voltage, -voltage);
  }

  @Override
  public void end(final boolean interrupted)
  {
    drive_train.drive(0, 0);
  }
}

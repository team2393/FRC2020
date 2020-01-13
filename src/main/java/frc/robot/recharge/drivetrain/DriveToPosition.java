/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpiutil.math.MathUtil;

/** Control position via PID */
public class DriveToPosition extends CommandBase 
{
  private final DriveTrain drive_train;
  private double desired_position = 0.0;

  public DriveToPosition(final DriveTrain drive_train) 
  {
    this.drive_train = drive_train;
  }

  public void setDesiredPosition(final double meters)
  {
    desired_position = meters;
  }

  public double getDesired_position()
  {
    System.out.println(desired_position);
    return desired_position;
  }

  @Override
  public void initialize()
  {
    super.initialize();
    drive_train.setGear(false);
    drive_train.getPositionPID().reset();
  }

  @Override
  public void execute()
  {
    final double speed = drive_train.getPositionPID().calculate(drive_train.getPositionMeters(), desired_position);
    drive_train.drive(MathUtil.clamp(speed, -0.8, 0.8), 0);
  }

  @Override
  public void end(boolean interrupted)
  {
    drive_train.drive(0, 0);
  }
}

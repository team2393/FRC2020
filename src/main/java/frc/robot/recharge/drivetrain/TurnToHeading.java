/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.drivetrain;

import org.opencv.core.Mat;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;

/** Control heading via PID */
public class TurnToHeading extends CommandBase 
{
  private final DriveTrain drive_train;

  public TurnToHeading(final DriveTrain drive_train) 
  {
    this.drive_train = drive_train;
  }

  public void setDesiredHeading(final double degrees)
  {
    drive_train.getHeadingPID().setSetpoint(degrees);
  }

  @Override
  public void initialize()
  {
    drive_train.getHeadingPID().reset();
  }

  @Override
  public void execute()
  {
    final double rotation = drive_train.getHeadingPID().calculate(drive_train.getHeadingDegrees());
    // Avoid 'jittering' in place
    if(Math.abs(rotation) < 0.05)
      drive_train.drive(0, 0);
    else
      drive_train.drive(0, MathUtil.clamp(rotation, -0.5, 0.5)); 
    }

  // @Override
  // public boolean isFinished()
  // {
  //   return drive_train.getHeadingPID().atSetpoint();
  // }
  
  @Override
  public void end(boolean interrupted)
  {
    drive_train.drive(0, 0);
  }
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.drivetrain;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;

/** Control heading via PID */
public class TurnToHeading extends CommandBase 
{
  private final DriveTrain drive_train;
  private final double desired_heading;
  private final Timer timer = new Timer();

  public TurnToHeading(final DriveTrain drive_train, final double heading) 
  {
    this.drive_train = drive_train;
    desired_heading = heading;
    // Use heading PID settings, but limit profile to 90 deg/sec rotational speed
    // Good enough: Within 1 degree, slower than 1 deg/sec
    drive_train.getHeadingPID().setTolerance(1.0);
    addRequirements(drive_train);
  }

  @Override
  public void initialize()
  {
    drive_train.getHeadingPID().reset();
    drive_train.getHeadingPID().setSetpoint(desired_heading);
    timer.start();
  }

  @Override
  public void execute()
  {
    double rotation = - drive_train.getHeadingPID().calculate(drive_train.getHeadingDegrees(), desired_heading);

    rotation = MathUtil.clamp(rotation, -0.4, 0.4);

    // TODO Avoid 'jittering' in place
    // if (Math.abs(rotation) < 0.05)
    //   drive_train.drive(0, 0);
    // else
    //   drive_train.drive(0, MathUtil.clamp(rotation, -0.5, 0.5)); 
    drive_train.drive(0, rotation); 
  }

  @Override
  public boolean isFinished()
  {
    if (timer.get() > 5.0)
    {
      System.err.println("TurnToHeading gives up (timeout)");
      return true;
    }
    return drive_train.getHeadingPID().atSetpoint();
  }
  
  @Override
  public void end(boolean interrupted)
  {
    drive_train.drive(0, 0);
  }
}

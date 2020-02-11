/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.drivetrain;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** Control heading via PID */
public class TurnToHeading extends CommandBase 
{
  private final DriveTrain drive_train;
  private final ProfiledPIDController pid;
  private final Timer timer = new Timer();

  public TurnToHeading(final DriveTrain drive_train, final double heading) 
  {
    this.drive_train = drive_train;
    // Use heading PID settings, but limit profile to 90 deg/sec rotational speed
    pid = new ProfiledPIDController(drive_train.getHeadingPID().getP(),
                                    drive_train.getHeadingPID().getI(),
                                    drive_train.getHeadingPID().getD(),
                                    new TrapezoidProfile.Constraints(90.0, 45.0));
    // Good enough: Within 1 degree, slower than 1 deg/sec
    pid.setTolerance(1.0, 1.0);
    pid.setGoal(heading);
    addRequirements(drive_train);
  }

  public void setDesiredHeading(final double degrees)
  {
    pid.reset(drive_train.getHeadingDegrees());
    pid.setGoal(degrees);
  }

  @Override
  public void initialize()
  {
    pid.reset(drive_train.getHeadingDegrees());
    timer.start();
  }

  @Override
  public void execute()
  {
    final double rotation = pid.calculate(drive_train.getHeadingDegrees());

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
    return pid.atGoal();
  }
  
  @Override
  public void end(boolean interrupted)
  {
    drive_train.drive(0, 0);
  }
}

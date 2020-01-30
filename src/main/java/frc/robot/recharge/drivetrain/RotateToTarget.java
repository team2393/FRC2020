/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.drivetrain;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;

/** Rotate to target based on camera info */
public class RotateToTarget extends CommandBase 
{
  private final Timer timer = new Timer();
  private final DriveTrain drive_train;
 
  /** Last pipeline calls that we saw */
  private int last_calls = -1;

  private int skipped = 0;

  private boolean on_target;
  
  public RotateToTarget(final DriveTrain drive_train) 
  {
    this.drive_train = drive_train;
    addRequirements(drive_train);

    SmartDashboard.setDefaultNumber("TargetRotGain", 0.05);
    SmartDashboard.setDefaultNumber("TargetRotMax", 0.33);
    // "Full screen" range would be +- 160
    SmartDashboard.setDefaultNumber("TargetRotThres", 100);
  }

  @Override
  public void initialize()
  {
    on_target = false;
    timer.start();
  }

  /** @return Direction to target, positive for 'right',
   *          0 for 'on target' or 'don't know'
   */
  public double getTargetDirection()
  {
    // Do we have new image information?
    int calls = (int) SmartDashboard.getNumber("PipelineCalls", -1);
    if (calls != last_calls)
      skipped = 0;
    else
      if (++skipped > 1)
          return 0;

    last_calls = calls;
    return SmartDashboard.getNumber("Direction", 0);
  }

  @Override
  public void execute()
  {
    final double direction = getTargetDirection();
    final double rotation;
    // Don't react to target that's too far off to the side
    if (Math.abs(direction) > SmartDashboard.getNumber("TargetRotThres", 30))
      rotation = 0.0;
    else // Proportial gain controller
      rotation = direction * SmartDashboard.getNumber("TargetRotGain", 0.0);

    final double max = SmartDashboard.getNumber("TargetRotMax", 0.4);
    drive_train.drive(0, MathUtil.clamp(rotation, -max, max));

    // if (Math.abs(direction) < 2)
    //   on_target = true;
  }
  
  @Override
  public boolean isFinished()
  {
    return false;
    // return on_target  ||  timer.get() > 5.0;
  }

  @Override
  public void end(boolean interrupted)
  {
    drive_train.drive(0, 0);
  }
}

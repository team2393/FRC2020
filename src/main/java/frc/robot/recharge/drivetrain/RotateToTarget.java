/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.drivetrain;

import java.time.LocalTime;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.recharge.udp.UDPReceiverThread;

/** Rotate to target based on camera info */
public class RotateToTarget extends CommandBase 
{
  private final Timer timer = new Timer();
  private final DriveTrain drive_train;

  private int skip = 1;
  private int direction = 0;

  public UDPReceiverThread udp;

  private boolean on_target;
  
  public RotateToTarget(final DriveTrain drive_train) throws Exception
  {
    udp = new UDPReceiverThread(5801);

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
    if (++skip > 1)
    {
      direction = udp.get();
      System.out.println("Read");
      if (direction == UDPReceiverThread.STALE)
      {
        direction = 0;
        System.out.println(LocalTime.now() + " Stale");  
      }
      else
        skip  = 0;
    }
    else
      System.out.println("Re-use");

    return direction; 
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

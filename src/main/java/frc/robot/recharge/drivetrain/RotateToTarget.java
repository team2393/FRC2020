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
import frc.robot.recharge.udp.CameraData;
import frc.robot.recharge.udp.UDPReceiverThread;

/** Rotate to target based on camera info */
public class RotateToTarget extends CommandBase 
{
  private final Timer timer = new Timer();
  private final DriveTrain drive_train;

  private int skip = 1;
  private CameraData last = new CameraData(0, 0);

  public UDPReceiverThread udp;

  private boolean on_target;
  
  public RotateToTarget(final DriveTrain drive_train)
  {
    try
    {
      udp = new UDPReceiverThread(5801);
    }
    catch (Exception ex)
    {
      throw new RuntimeException(ex);
    }

    this.drive_train = drive_train;
    addRequirements(drive_train);

    SmartDashboard.setDefaultNumber("TargetRotGain", 0.02);
    // "Full screen" range would be +- 160
    SmartDashboard.setDefaultNumber("TargetRotThres", 150);
    SmartDashboard.setDefaultNumber("Desired Distance", -500);
    SmartDashboard.setDefaultNumber("Desired Direction", 0);
  }

  @Override
  public void initialize()
  {
    on_target = false;
    timer.start();
  }

  public void updateData()
  {
    if (++skip > 1)
    {
      CameraData data = udp.get();
      // System.out.println("Read");
      if (data == null)
      {
        last.direction = last.distance = 0;
        // System.out.println(LocalTime.now() + " Stale");  
      }
      else
      {
        skip  = 0;
        last.direction = data.direction;
        last.distance = data.distance;
      }
    }
    // else
    //   System.out.println("Re-use");
  }

  @Override
  public void execute()
  {
    updateData();
    final double direction = last.direction;

    double rotation;
    // Don't react when direction is 0/unknown,
    // or when detected target is too far off to the side
    if (direction == 0.0  ||
        Math.abs(direction) > SmartDashboard.getNumber("TargetRotThres", 150))
    {
      rotation = 0.0;
    }
    else
    {
      final double direction_error = direction - SmartDashboard.getNumber("Desired Direction", 0);

      // Proportial gain controller with some minimum 
      rotation = direction_error * SmartDashboard.getNumber("TargetRotGain", 0.02);
      if (direction > 1)
        rotation += 0.1;
      else if (direction < 1)
        rotation -= 0.1;
    }

    double desired_distance = SmartDashboard.getNumber("Desired Distance", -500); 
    double speed = 0;
    double position_error = 0;
    
    // Make sure desired distance is within screen
    if (desired_distance > -120 && desired_distance < 120)
    {
      position_error = (last.distance == 0)  ?  0  :  (desired_distance - last.distance);
      speed = position_error * 10 * SmartDashboard.getNumber("TargetRotGain", 0.02);
    }
    
    final double max = 0.35;   
    drive_train.drive(MathUtil.clamp(speed, -max, max), MathUtil.clamp(rotation, -max, max));

    if (Math.abs(direction) < 2 && Math.abs(position_error) < 2)
      on_target = true;
  }
  
  @Override
  public boolean isFinished()
  {
    if (timer.hasElapsed(5.0))
    {
      System.err.println("RotateToTarget gives up (timeout)");
      return true;
    }
    return on_target;
  }

  @Override
  public void end(boolean interrupted)
  {
    drive_train.drive(0, 0);
  }
}

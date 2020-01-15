/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.drivetrain;

import java.sql.Date;
import java.time.ZonedDateTime;
import javax.xml.crypto.Data;

import org.opencv.core.Mat;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.recharge.OI;

/** Control heading via PID */
public class HeadingHold extends CommandBase 
{
  private final DriveTrain drive_train;

  public HeadingHold(final DriveTrain drive_train) 
  {
    this.drive_train = drive_train;
    SmartDashboard.setDefaultNumber("Right Stick", 0.07);
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

  /** 'delay' used to not resume heading hold until we took the finger off the steering stick for some time */
  long resume_heading_hold = 0;

  @Override
  public void execute()
  {
    final long now = System.currentTimeMillis();
    
    final boolean steering = Math.abs(OI.getDirection()) > SmartDashboard.getNumber("Right Stick", 0);
    SmartDashboard.putBoolean("Steering", steering);
    /*
    It seems that even though the left stick is released the robot 
    usually has a little inertia left and continues to spin and then the PID attempts
    to return to the position at the exact time the stick was released...
    Maybe set a timer before new rotation is set?
    TODO fix that ^
    */
    if (steering)
    {
      drive_train.drive(OI.getSpeed(), OI.getDirection());
      setDesiredHeading(drive_train.getHeadingDegrees());
      resume_heading_hold = now + 1000;
    }
    else if (now < resume_heading_hold)
    {
      drive_train.drive(OI.getSpeed(), 0);
      setDesiredHeading(drive_train.getHeadingDegrees());
    }
    else
    {
      final double rotation = drive_train.getHeadingPID().calculate(drive_train.getHeadingDegrees());
      drive_train.drive(OI.getSpeed(), rotation);
    }
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

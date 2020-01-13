/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.drivetrain;

import edu.wpi.first.wpilibj2.command.PIDCommand;

/** Control heading via PID */
public class TurnToHeading extends PIDCommand 
{
  private double desired_heading = 0.0;

  public TurnToHeading(final DriveTrain drive_train) 
  {
    super(drive_train.getHeadingPID(),
          drive_train::getHeadingDegrees,
          0,
          output -> drive_train.drive(0, output),
          drive_train);
    this.m_measurement = this::getDesiredHeading;
  }

  public void setDesiredHeading(final double degrees)
  {
    desired_heading = degrees;
  }

  public double getDesiredHeading()
  {
    return desired_heading;
  }
}

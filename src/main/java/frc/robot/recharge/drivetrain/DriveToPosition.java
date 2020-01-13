/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.drivetrain;

import edu.wpi.first.wpilibj2.command.PIDCommand;

/** Control position via PID */
public class DriveToPosition extends PIDCommand 
{
  private final DriveTrain drive_train;
  private double desired_position = 0.0;

  public DriveToPosition(final DriveTrain drive_train) 
  {
    super(drive_train.getPositionPID(),
          drive_train::getSpeedMetersPerSecond,
          0,
          output -> drive_train.drive(output, 0),
          drive_train);
    this.m_measurement = this::getDesired_position;
    this.drive_train = drive_train;
  }

  public void setDesiredPosition(final double position)
  {
    desired_position = position;
  }

  public double getDesired_position()
  {
    return desired_position;
  }

  @Override
  public void initialize()
  {
    super.initialize();
    drive_train.setGear(false);
  }
}

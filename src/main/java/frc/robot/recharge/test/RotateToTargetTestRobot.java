/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.test;

import edu.wpi.first.wpilibj.Servo;
import frc.robot.BasicRobot;
import frc.robot.recharge.drivetrain.DriveTrain;
import frc.robot.recharge.drivetrain.RotateToTarget2;

/** Robot code for testing camera and rotate-to-target */
public class RotateToTargetTestRobot extends BasicRobot
{
  // Real robot uses drivetrain, demo setup uses servo
  private final DriveTrain drive_train = new DriveTrain();

  private final RotateToTarget2 rotate_to_target = new RotateToTarget2( (speed, rotation) ->
  {
    // 'rotation' is in speed controller terms of -1...1
    // Servo uses 0..1  
    drive_train.drive(speed, rotation);
  });
  
  @Override
  public void robotInit()
  {
    super.robotInit();
  }

  @Override
  public void autonomousPeriodic()
  {
    rotate_to_target.schedule();
  }
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.recharge.OI;

public class HoodAdjust extends CommandBase 
{

  private final Hood hood;

  public HoodAdjust(Hood hood) 
  {
    this.hood = hood;
    addRequirements(hood);
  }

  // Called when the command is initially scheduled.
  // TODO maybe do something here 
  //@Override
  // public void initialize() 
  // {

  // }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    System.out.println("Hood angle: " + hood.getHoodAngle());
    hood.setAngleMotor(OI.getHoodSpeed());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    hood.setAngleMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return false;
  }
}

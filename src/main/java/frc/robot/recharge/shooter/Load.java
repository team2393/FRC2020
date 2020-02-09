/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** Load power cell(s) */
public class Load extends CommandBase
{
  private final PowerCellAccelerator pca;

  public Load(final PowerCellAccelerator pca)
  {
    this.pca = pca;
    addRequirements(pca);
  }

  @Override
  public void initialize()
  {
    // Don't shoot, yet
    pca.eject(false);
  }

  @Override
  public void execute()
  {
    if (pca.powerCellReady())
    {
      // Stop when the first ball reaches the end of the conveyor.
      pca.moveConveyor(0);
      // If there is a separate sensor at end of horizontal conveyor,
      // keep horiz. belt moving until ball is in there.
      // if (pca.powerCellAtEndOfHorizontal())
      //   pca.moveHorizontalConveyor(0);
      // else
      //   pca.moveHorizontalConveyor(PowerCellAccelerator.CONVEYOR_VOLTAGE);
    }
    else
    {
      // Move conveyor so that one or more balls get loaded
      pca.moveConveyor(PowerCellAccelerator.CONVEYOR_VOLTAGE);
      // pca.moveHorizontalConveyor(PowerCellAccelerator.CONVEYOR_VOLTAGE);
    }
  }

  // isFinished(): false, keep turning on/off based on ball state

  @Override
  public void end(final boolean interrupted)
  {
    pca.moveConveyor(0);
    //   pca.moveHorizontalConveyor(0);
  }
}

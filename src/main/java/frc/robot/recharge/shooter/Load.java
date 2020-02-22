/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** Load power cell(s),
 *  Finishes when one power cell is 'ready',
 *  but can restarted.
 */
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
    if (!pca.powerCellReady())
    {
      pca.moveTop(PowerCellAccelerator.CONVEYOR_VOLTAGE);
      pca.moveBottom(PowerCellAccelerator.CONVEYOR_VOLTAGE);
    }
    else
    {
      pca.moveTop(0);
    
      if (!pca.lowConveyorFull())
        pca.moveBottom(PowerCellAccelerator.CONVEYOR_VOLTAGE);
      else
        pca.moveBottom(0);
    }
  }

  @Override
  public boolean isFinished()
  {
    return pca.powerCellReady() && pca.lowConveyorFull();
  }

  @Override
  public void end(final boolean interrupted)
  {
    pca.moveBottom(0);
    pca.moveTop(0);
  }
}

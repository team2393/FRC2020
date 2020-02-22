/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** Turn conveyors etc. off */
public class ShooterIdle extends CommandBase
{
  private final PowerCellAccelerator pca;

  public ShooterIdle(final PowerCellAccelerator pca)
  {
    this.pca = pca;
    addRequirements(pca);
  }

  @Override
  public void initialize()
  {
    pca.eject(false);
  }

  @Override
  public void execute()
  {
    // Looks like we're doing nothing, but need to actively set
    // speeds to 0 for each period to avoid motor safety timeouts.
    pca.moveBottom(0);
    pca.moveTop(0);
  }
}

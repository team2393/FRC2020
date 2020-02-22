/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** Eject one power cell */
public class Eject extends CommandBase
{
  private final PowerCellAccelerator pca;
  private enum State
  {
    SPINUP,
    EJECT
  };
  private State state;
  private final Timer timer = new Timer();

  public Eject(final PowerCellAccelerator pca)
  {
    this.pca = pca;
    addRequirements(pca);
  }

  @Override
  public void initialize()
  {
    // Turn on the ejector
    pca.eject(true);
    state = State.SPINUP;
    System.out.println(state);
  }

  @Override
  public void execute()
  {
    // Once it's fast enough, SHOOT!!
    if (pca.getShooterRPM() >= PowerCellAccelerator.MINIMUM_SHOOTER_RPM)
    {
      state = State.EJECT;
      timer.start();
      System.out.println(state);
    }
    
    // Are we shooting? If so, move a ball out
    if (state == State.EJECT)
      pca.moveConveyor(PowerCellAccelerator.CONVEYOR_VOLTAGE);
    // else
    // {
    //   // Prepare for next shot by loading another ball
    //   if (pca.powerCellReady())
    //     pca.moveConveyor(0);
    //   else
    //     pca.moveConveyor(PowerCellAccelerator.CONVEYOR_VOLTAGE);
    // }

    // If there is a separate sensor at end of horizontal conveyor,
    // keep horiz. belt moving until ball is in there.
    // if (pca.powerCellAtEndOfHorizontal())
    //   pca.moveHorizontalConveyor(0);
    // else
    //   pca.moveHorizontalConveyor(PowerCellAccelerator.CONVEYOR_VOLTAGE);
  }

  @Override
  public boolean isFinished()
  {
    // Stop when a ball is seen coming out, or we've tried for e few seconds
    return pca.powerCellFired(); //   ||  timer.get() > 2.0;
  }

  @Override
  public void end(final boolean interrupted)
  {
    pca.eject(false);
    pca.moveConveyor(0);
    //   pca.moveHorizontalConveyor(0);
  }
}

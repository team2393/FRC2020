/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** Eject one power cell.
 *  Since ejector keeps running a little longer,
 *  rescheduling this command right away keeps
 *  the cells flowing.
 */
public class Eject extends CommandBase
{
  private final PowerCellAccelerator pca;
  private enum State
  {
    SPINUP,    // Spinning up, waiting for correct RPM
    EJECT,     // Trying feed a ball to ejector
    SUCCESS,   // Saw a ball shoot out
    TIMEOUT    // Give up, no ball seen flying out
  };
  private State state = State.SPINUP;
  private final Timer timer = new Timer();
  private final Timer wait = new Timer();

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
    wait.start();
  }

  @Override
  public void execute()
  {
    if (state == State.SPINUP)
    {
      // Once it's fast enough, SHOOT!!
      if (pca.getShooterRPM() >= PowerCellAccelerator.MINIMUM_RPM_FRACTION * PowerCellAccelerator.SHOOTER_RPM)
      {
        state = State.EJECT;
        timer.start();
        wait.stop();
      }
      else
      {
        // Not fast enough. If there's a ball ready, keep it there
        if (pca.powerCellReady())
          pca.moveConveyor(0);
        else // Otherwise load a ball
          pca.moveConveyor(PowerCellAccelerator.CONVEYOR_VOLTAGE);
      }
    }

    // Are we shooting? If so, move a ball out
    if (state == State.EJECT)
    {
      pca.moveConveyor(PowerCellAccelerator.CONVEYOR_VOLTAGE);

      // Ideally, we soon detect a ball flying out
      if (pca.powerCellFired())
        state = State.SUCCESS;
      // In reality, we might not, so stop after a few seconds
      else if (timer.hasElapsed(10.0))
        state = State.TIMEOUT;
    }

    // If there is a separate sensor at end of horizontal conveyor,
    // keep horiz. belt moving until ball is in there,
    // regardless of what the vertical conveyor and ejector are doing.
    // if (pca.powerCellAtEndOfHorizontal())
    //   pca.moveHorizontalConveyor(0);
    // else
    //   pca.moveHorizontalConveyor(PowerCellAccelerator.CONVEYOR_VOLTAGE);
  }

  @Override
  public boolean isFinished()
  {
    return state == State.SUCCESS  ||  state == State.TIMEOUT;
  }

  @Override
  public void end(final boolean interrupted)
  {
    // Turn ejector off
    // (but it keeps running for a while in case we want to shoot again, soon)
    pca.eject(false);
    
    // Turn conveyor off to prevent another ball from shooting out.
    // To shoot another ball, run this command again,
    // so it'll check the RPM, then feed another ball. 
    pca.moveConveyor(0);
    //   pca.moveHorizontalConveyor(0);

    System.out.println("Spinup delay: " + wait.get() + " seconds");
  }
}

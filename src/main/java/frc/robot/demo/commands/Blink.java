/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.demo.commands;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** Command that blinks (or beeps)
 * 
 *  Periodically turns an output on/off
 */
public class Blink extends CommandBase
{
  private final DigitalOutput output;
  private final long ms;

  /** @param output Output to use
   *  @param ms On/off period in milliseconds
   */
  public Blink(final DigitalOutput output, final long ms)
  {
    this.output = output;
    this.ms = ms;
  }

  @Override
  public void execute()
  {
    // Determine if output should be on or off
    final boolean on_off = (System.currentTimeMillis() / ms) % 2 == 1;
    output.set(on_off);
  }

  @Override
  public void end(boolean interrupted)
  {
    // When command ends, ensure that output is off
    output.set(false);
  }
}

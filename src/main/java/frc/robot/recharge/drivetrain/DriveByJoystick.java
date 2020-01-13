/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.drivetrain;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.recharge.OI;

/** Manually control speed and rotation via joystick
 * 
 *  Automatically shifts gear
 */
public class DriveByJoystick extends InstantCommand 
{
  private final DriveTrain drive_train;

  public DriveByJoystick(DriveTrain drive_train) 
  {
    this.drive_train = drive_train;
    addRequirements(drive_train);
  }

  @Override
  public void initialize() 
  {
  }

  @Override
  public void execute()
  {
    drive_train.drive(OI.getSpeed(), OI.getDirection());

    // Shift down when joystick near 'idle'
    if (Math.abs(OI.getSpeed()) < 0.1)
      drive_train.setGear(false);

    // TODO Automatically shift into fast gear
    // if we are in low gear,
    // joystick pedal to the metal
    // and encoder indicates we reached the max. speed
    // that we can get in low gear
    // if (!drive_train.isHighSpeed() &&
    //     Math.abs(OI.getSpeed()) > 0.85 &&
    //     drive_train.getSpeed() > xxx m/sec)
    //     drive_train.setGear(true);
  }

  @Override
  public void end(final boolean interrupted)
  {
    drive_train.drive(0, 0);
  }
}

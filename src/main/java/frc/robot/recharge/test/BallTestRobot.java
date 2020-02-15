/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.test;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.BasicRobot;
import frc.robot.recharge.OI;
import frc.robot.recharge.shooter.PowerCellAccelerator;

/** Robot code for testing ball handling
 *  Use _after_ SpinnerTestRobot has tuned spinner
 */
public class BallTestRobot extends BasicRobot
{
  private final PowerCellAccelerator pca = new PowerCellAccelerator();

  @Override
  public void robotPeriodic()
  {
    super.robotPeriodic();
    SmartDashboard.putBoolean("Ready", pca.powerCellReady());
    SmartDashboard.putBoolean("Fired", pca.powerCellFired());
    SmartDashboard.putNumber("Eject RPM", pca.getShooterRPM());
  }

  @Override
  public void teleopPeriodic()
  {
    // Run ejector when A is held (plus a little longer)
    pca.eject(OI.joystick.getAButton());

    // Hold Y to run conveyor at its 'normal' speed,
    // or use forward/back to directly control speed
    if (OI.joystick.getYButton())
      pca.moveConveyor(PowerCellAccelerator.CONVEYOR_VOLTAGE);
    else
      pca.moveConveyor(12*OI.getSpeed());
  }
}

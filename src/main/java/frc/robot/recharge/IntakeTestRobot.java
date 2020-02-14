/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.BasicRobot;
import frc.robot.recharge.shooter.Intake;

/** Robot code for testing intake */
public class IntakeTestRobot extends BasicRobot
{
  private final Intake intake = new Intake();
  
  @Override
  public void robotInit()
  {
    super.robotInit();
    SmartDashboard.setDefaultNumber("kCos", 0.0);
    SmartDashboard.setDefaultNumber("P", 0.0);
  }

  @Override
  public void robotPeriodic()
  {
    super.robotPeriodic();
    SmartDashboard.putNumber("Intake Angle", intake.getAngle());   
  }

  @Override
  public void teleopPeriodic()
  {
    // Hold A button to 'home'
    if (OI.joystick.getAButton())
      intake.homeIntake();

    // Hold B button to run spinner
    intake.enableSpinner(OI.joystick.getBButton());

    // 'left/right' axis to directly run rotator angle motor
    intake.setRotatorMotor(OI.getDirection());
  }

  @Override
  public void autonomousPeriodic()
  {
    intake.configure(SmartDashboard.getNumber("kCos", 0),
                     SmartDashboard.getNumber("P", 0));
    // Every 3 seconds toggle between two angles
    final boolean high = (System.currentTimeMillis() / 3000) % 2 == 0;
    intake.setIntakeAngle(high ? 60 : 30);
  }
}

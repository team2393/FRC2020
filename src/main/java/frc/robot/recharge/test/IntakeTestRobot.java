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
    // 2) Verify that angle indicates 0 (all out, 'horizontal')
    //    to ~90 degrees (all up, 'vertical').
    //    If not, fix getAngle()
    SmartDashboard.putNumber("Intake Angle", intake.getAngle());   

    intake.configure(SmartDashboard.getNumber("kCos", 0),
                     SmartDashboard.getNumber("P", 0));
  }

  @Override
  public void teleopPeriodic()
  {
    // 1) Hold A button to run spinner (already adjusted via SpinnerTestRobot)
    intake.enableSpinner(OI.joystick.getAButton());

    if (! OI.joystick.getYButton())
    {
      // 3) Check that 'forward' moves intake 'up'
      //    If not, invert motor and start over at step 1)
      intake.setRotatorMotor(OI.getSpeed());
    }
    else
    {
      // 4) Manually move arm to ~45 degrees.
      //    Hold Y button.
      //    Adjust kCos to have motor hold it there.
      //    Adjust P to have motor keep it there.
      intake.setIntakeAngle(45.0);
    }
  }

  @Override
  public void autonomousPeriodic()
  {
    // 5) Tweak settings to move between two angles
    final boolean high = (System.currentTimeMillis() / 3000) % 2 == 0;
    intake.setIntakeAngle(high ? 60 : 30);

    // 6) Enable the code in Intake.periodic() that
    //    has motor settle at <10 degrees after 5 seconds.
    //    Change the two test angles to 0 & 45,
    //    period to 10 seconds,
    //    see how it works.
  }
}

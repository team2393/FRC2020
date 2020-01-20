/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.climb;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Motors related to climb mechanism */
public class Climber extends SubsystemBase
{
  // TODO Motor for telescoping pole
  // private final WPI_TalonSRX telescope = new WPI_TalonSRX(RobotMap.XXXX);
  // private final WPI_TalonSRX puller = new WPI_TalonSRX(RobotMap.XXXX);

  // public Climber()
  // {
  //   // Are there encoders to initialize?
  // }

  /** Move telescoping arm up/down
   *  @param direction 1: Full speed up, -1: Full speed down
   */
  public void moveRelescope(double direction)
  {
    // TODO Instruct telescope motor, check direction, maybe limit speed.
    // Is  there an encoder to check so we don't rip the cord?
  }

  /** Pull up
   *  @param speed How fast to pull up, 0..1
   */
  public void pullUp(double speed)
  {
    // TODO Instruct puller. Note that we must only  pull 'up',
    // Ratched prevents us from going back down.
  }

  // TODO Is there a special way to go back down?
  // I.e. back in the pits, with some special button pressed on the drive station,
  // may we run the 'pull' motor to feed the rope back out while the rachet is
  // mechanically disabled?
}

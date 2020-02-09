/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.shooter;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.recharge.RobotMap;

/** Power cell handling: Intake
 * 
 *  Intake picks 'fuel cell' balls from floor into hopper.
 */
public class Intake extends SubsystemBase 
{
  // Motors
  private final WPI_VictorSPX intake_motor = new WPI_VictorSPX(RobotMap.INTAKE_MOTOR);
  // Must have encoder (angle) and limit switch (home)
  private final WPI_TalonFX intake_position = new WPI_TalonFX(RobotMap.INTAKE_POSITION);

  // FF & PID
  // https://trickingrockstothink.com/blog_posts/2019/10/26/controls_supp_arm.html
  // TODO Tune, then turn into ProfiledPIDController
  private final PIDController intake_position_pid = new PIDController(0, 0, 0);

  public Intake()
  {
    PowerCellAccelerator.commonSettings(intake_motor, NeutralMode.Coast);
    PowerCellAccelerator.commonSettings(intake_position, NeutralMode.Brake);

    // Encoder for position (angle)
    intake_position.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);    

    // Limit switch to 'home'
    intake_position.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
  }
    
  /** Move intake up towards 'home' switch.
   *  @return true if at 'home' position
   */
  public boolean homeIntake()
  {
    // TODO Find voltage for slow movement towards home switch
    // TODO Is home switch the forward or reverse limit switch?
    intake_position.setVoltage(-0.1);
    final boolean homed = intake_position.isFwdLimitSwitchClosed() == 1;
    if (homed)
     intake_position.setSelectedSensorPosition(0);
    return homed;
  }

  private double getIntakeAngle()
  {
    // TODO Calibrate conversion from encoder counts to angle

    // Try frc-characterization for 'arm'.
    // An angle of zero (degrees/radians) must be 'horizontal'
    // because  ArmFeedforward  uses cos(angle) to determine impact of gravity,
    // which is at maximum for angle 0 (cos(0)=1) and vanishes at 90 deg (cos(90)=0)

    return intake_position.getSelectedSensorPosition();
  }

  public void setIntakeAngle(final double angle)
  {
    // TODO First connect joystick axis to 'angle' to simply move motor
    intake_position.setVoltage(angle);

    // Calibrate angle
    // Then change this to
    //    intake_position_pid.setSetpoint(angle);
    // enable PID in periodic() below and tune PID.
    // Then change to ProfiledPIDController
  }
  
  // TODO Command to lower intake and turn rollers on,
  //      or raise intake with rollers off.
  public void setIntakeSpeed(final double volt)
  {
    intake_motor.setVoltage(-Math.abs(volt)); // TODO find out which way motor spins and only allow intake to spin one direction
  }

  @Override
  public void periodic()
  {
    SmartDashboard.putNumber("Intake Angle", getIntakeAngle());
   
    // TODO See setIntakeAngle(angle);
    // intake_motor.setVoltage(intake_position_pid.calculate(getIntakeAngle()));
  }
}

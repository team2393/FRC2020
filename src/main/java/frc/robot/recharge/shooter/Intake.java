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

import edu.wpi.first.wpilibj.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.recharge.RobotMap;

/** Power cell handling: Intake
 * 
 *  Intake picks 'fuel cell' balls from floor into hopper.
 */
public class Intake extends SubsystemBase 
{
  // Motors
  private final WPI_VictorSPX spinner = new WPI_VictorSPX(RobotMap.INTAKE_SPINNER);
  
  // Must have encoder (angle) and limit switch (home)
  private final WPI_TalonFX rotator = new WPI_TalonFX(RobotMap.INTAKE_ROTATOR);
  private final WPI_TalonFX rotator_salve = new WPI_TalonFX(RobotMap.INTAKE_ROTATOR_SLAVE);

  // FF & PID
  // https://trickingrockstothink.com/blog_posts/2019/10/26/controls_supp_arm.html
  // TODO Tune, then turn into ProfiledPIDController?
  private ArmFeedforward angle_ff = new ArmFeedforward(0.0, 0.0, 0.0);
  private final PIDController angle_pid = new PIDController(0, 0, 0);

  private boolean run_spinner = false;

  /** Desired arm/rotator angle. Negative to disable PID */
  private double desired_angle = -1;

  public Intake()
  {
    PowerCellAccelerator.commonSettings(spinner, NeutralMode.Coast);
    // Spin up/down with delay to please the battery
    spinner.configOpenloopRamp(0.6);

    PowerCellAccelerator.commonSettings(rotator, NeutralMode.Brake);
    PowerCellAccelerator.commonSettings(rotator_salve, NeutralMode.Brake);

    // Encoder for position (angle)
    rotator.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);    

    // Limit switch to 'home'
    rotator.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);

    // TODO Enable when direction etc. have been determined
    // rotator_salve.setInverted(true);
    // rotator_salve.follow(rotator);
  }
    
  /** Move intake up towards 'home' switch.
   *  @return true if at 'home' position
   */
  public boolean homeIntake()
  {
    // TODO Find voltage for slow movement towards home switch
    // TODO Is home switch the forward or reverse limit switch?
    rotator.setVoltage(-0.1);
    final boolean homed = rotator.isFwdLimitSwitchClosed() == 1;
    if (homed)
     rotator.setSelectedSensorPosition(0);
    return homed;
  }

  /** @return Rotator arm angle, degrees. 0 for horizontal, towards 90 for 'up' */
  public double getAngle()
  {
    // TODO Calibrate conversion from encoder counts to angle

    // Try frc-characterization for 'arm'.
    // An angle of zero (degrees/radians) must be 'horizontal'
    // because  ArmFeedforward  uses cos(angle) to determine impact of gravity,
    // which is at maximum for angle 0 (cos(0)=1) and vanishes at 90 deg (cos(90)=0)

    return 1.0 * rotator.getSelectedSensorPosition();
  }

  /** @param speed Directly set rotator motor speed for testing */
  public void setRotatorMotor(final double speed)
  {
    // Disable automated control
    desired_angle = -1;
    rotator.set(speed);
  }

  /** @param kCos Cosine(angle) factor to compensate for gravity
   *  @param P Proportional gain for angle error
   */
  public void configure(final double kCos, final double P)
  {
    angle_pid.reset();
    angle_ff = new ArmFeedforward(0, kCos, 0);
    angle_pid.setP(P);
  }

  /** @param angle Set angle for PID-controlled angle rotator, negative to disable */
  public void setIntakeAngle(final double angle)
  {
    desired_angle = angle;
  }
  
  // TODO Command to lower intake and turn rollers on,
  //      or raise intake with rollers off.

  /** @param on Should intake spinner be on? */
  public void enableSpinner(final boolean on)
  {
    run_spinner = on;
  }
  
  @Override
  public void periodic()
  {
    // TODO find out which way motor spins and what's a good speed
    spinner.setVoltage(run_spinner ? 3.0 : 0.0);

    if (desired_angle >= 0)
    {
      final double correction = angle_pid.calculate(getAngle(), desired_angle);
      final double preset = angle_ff.calculate(getAngle(), 0);

      rotator.setVoltage(preset + correction);
    }
  }
}

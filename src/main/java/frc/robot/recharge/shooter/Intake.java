/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.shooter;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Timer;
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
  
  // Main rotator has encoder (angle)
  private final WPI_TalonSRX rotator = new WPI_TalonSRX(RobotMap.INTAKE_ROTATOR);
  private final WPI_TalonSRX rotator_salve = new WPI_TalonSRX(RobotMap.INTAKE_ROTATOR_SLAVE);

  // FF & PID
  // https://trickingrockstothink.com/blog_posts/2019/10/26/controls_supp_arm.html
  // TODO Tune, then turn into ProfiledPIDController?
  private ArmFeedforward angle_ff = new ArmFeedforward(0.0, 0.0, 0.0);
  private final PIDController angle_pid = new PIDController(0, 0, 0);

  private boolean run_spinner = false;

  /** Desired arm/rotator angle. Negative to disable PID */
  private double desired_angle = -1;

  /** Timer since last change of desired_angle */
  private final Timer timer = new Timer();

  public Intake()
  {
    PowerCellAccelerator.commonSettings(spinner, NeutralMode.Coast);
    // Spin up/down with delay to please the battery
    spinner.configOpenloopRamp(0.6);

    PowerCellAccelerator.commonSettings(rotator, NeutralMode.Brake);
    PowerCellAccelerator.commonSettings(rotator_salve, NeutralMode.Brake);

    // Encoder for position (angle)
    rotator.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);    

    // TODO Enable when direction etc. have been determined
    // rotator_salve.setInverted(true);
    // rotator_salve.follow(rotator);
  }
    
  /** @return Rotator arm angle, degrees. 0 for horizontal, towards 90 for 'up' */
  public double getAngle()
  {    
    // Try frc-characterization for 'arm'.
    // An angle of zero (degrees/radians) must be 'horizontal'
    // because  ArmFeedforward  uses cos(angle) to determine impact of gravity,
    // which is at maximum for angle 0 (cos(0)=1) and vanishes at 90 deg (cos(90)=0)
    
    // TODO Calibrate conversion from encoder counts to angle

    // Encoder provides 4096 ticks per 360 degrees
    final double encoder_angle = 360.0 / 4096;
    // Gears & chain result in actual arm moving slower than the motor output 
    final double gearing = 12.0 / 30.0;

    // Offset to get 0 degree == horizontal
    final double offset = 0.0;
    return offset + rotator.getSelectedSensorPosition() * encoder_angle * gearing;
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
    timer.start();
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
      // TODO If the angle is low (put arm out), check with timer.
      // After some time, simply turn motor off to let arm settle onto bumper.
      // if (desired_angle < 10   &&  timer.get() > 5.0)
      //   rotator.setVoltage(0);
      // else
      {
        final double correction = angle_pid.calculate(getAngle(), desired_angle);
        final double preset = angle_ff.calculate(desired_angle, 0);
  
        rotator.setVoltage(preset + correction);
      }
    }
  }
}

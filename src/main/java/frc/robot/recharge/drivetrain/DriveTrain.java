/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.recharge.drivetrain;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.recharge.RobotMap;

/** Drive train (them wheels) */
public class DriveTrain extends SubsystemBase
{
  private static final double TICKS_PER_METER = 1.0;

  // Motors
  private final WPI_TalonFX left_main = new WPI_TalonFX(RobotMap.LEFT_MOTOR_MAIN);
  private final WPI_TalonFX right_main = new WPI_TalonFX(RobotMap.RIGHT_MOTOR_MAIN);
  private final WPI_TalonFX left_slave = new WPI_TalonFX(RobotMap.LEFT_MOTOR_SLAVE);
  private final WPI_TalonFX right_slave = new WPI_TalonFX(RobotMap.RIGHT_MOTOR_SLAVE);

  // Combine (main) motors into diff' drive 
  private final DifferentialDrive differential_drive = new DifferentialDrive(left_main, right_main);

  // Gear shifter
  private final Solenoid shifter = new Solenoid(RobotMap.GEAR_SOLENOID);

  // Gyro
  // When attached to talon, we need that talon, which is outside of the drivetrain...
  // PigeonIMU gyro = new PigeonIMU(new TalonSRX(1));
  // For now use the other gyro since it's easy to install and access
  private final Gyro gyro = new ADXRS450_Gyro();

  // PID
  private final PIDController position_pid = new PIDController(0.0, 0.0, 0.0);
  private final PIDController heading_pid = new PIDController(0.0, 0.0, 0.0);


  public DriveTrain()
  {
    
    commonSettings(left_main);
    commonSettings(right_main);
    commonSettings(left_slave);
    commonSettings(right_slave);
    
    // Instruct slave motors to follow their respective main
    left_slave.follow(left_main);
    right_slave.follow(right_main);
    left_slave.setSafetyEnabled(false);
    right_slave.setSafetyEnabled(false);
    
    // No deadband on differential drive to allow
    // even small PID-driven moves.
    // If joystick needs dead zone, put that into OI.
    differential_drive.setDeadband(0.0);
    
    // Initially, set low gear
    setGear(false);
    
    gyro.calibrate();
    gyro.reset();
    // TODO  heading_pid.enableContinuousInput(-180.0, 180.0);

    SmartDashboard.putData("Position PID", position_pid);
    SmartDashboard.putData("Heading PID", heading_pid);
  }

  /** @param motor Motor to configure with common settings */
  private void commonSettings(final WPI_TalonFX motor)
  {
    motor.configFactoryDefault();
    motor.clearStickyFaults();
    motor.setNeutralMode(NeutralMode.Brake);
  }

  public void drive(final double speed, final double rotation)
  {
    differential_drive.arcadeDrive(speed, rotation);
  }

  public boolean isHighSpeed()
  {
    return shifter.get();
  }

  public void setGear(final boolean high_speed)
  {
    shifter.set(high_speed);
  }

  public boolean isIdle()
  {
    return left_main.getSelectedSensorVelocity() == 0  &&
           right_main.getSelectedSensorVelocity() == 0;
  }

  public PIDController getPositionPID()
  {
    return position_pid;
  }

  public double getPositionMeters()
  {
    final int avg_ticks = (left_main.getSelectedSensorPosition() +
                           right_main.getSelectedSensorPosition()) / 2;
    return avg_ticks / TICKS_PER_METER;
  }

  public double getSpeedMetersPerSecond()
  {
    // TODO "sensor per 100ms .. see phoenix documentation how to interprete"
    final int avg_tickspeed = (left_main.getSelectedSensorVelocity() +
                               right_main.getSelectedSensorVelocity()) / 2;
    return avg_tickspeed / TICKS_PER_METER;
  }

  /** Reset all encoders to 0 */
  public void reset()
  {
    gyro.reset();
    left_main.setSelectedSensorPosition(0);
    right_main.setSelectedSensorPosition(0);
  }

  public double getHeadingDegrees()
  {
    return gyro.getAngle();
  }

  public PIDController getHeadingPID()
  {
    return heading_pid;
  }

  @Override
  public void periodic()
  {
    SmartDashboard.putNumber("Position", getPositionMeters());
    SmartDashboard.putNumber("Speed", getSpeedMetersPerSecond());
    SmartDashboard.putNumber("Heading", getHeadingDegrees());
  }
}
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.recharge.drivetrain;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.recharge.RobotMap;

/** Drive train (them wheels)
 * 
 *  Geometry:
 *  Initial position is X=0, Y=0, angle=0, pointing toward the alliance station.
 *  Moving forward means moving along the X axis.
 *  Angle increases when turning left.
 *  Angle of 90 degrees means moving along the Y axis.
 */
public class DriveTrain extends SubsystemBase
{
  private static final double TICKS_PER_METER = 512651 / Units.inchesToMeters(288);

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
  private final PIDController position_pid = new PIDController(5.0, 0.0, 1.5);
  private final PIDController heading_pid = new PIDController(0.1, 0.0, 0.025);

  // Results of basic drive test:
  // Minimum voltage to move: 0.09 V
  //
  // SimpleMotorFeedforward:   Voltage = Vmin + K * Speed
  // Voltage  Speed [m/s]   K
  // 2.15     0.5           4.12
  // 4.4      1             4.31
  // 1.86     0.41          4.32
  private final SimpleMotorFeedforward feed_forward = new SimpleMotorFeedforward(0.09, 4.3);
  private final PIDController speed_pid = new PIDController(2, 0, 0);

  // Track current position based on gyro and encoders
  private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(0));

  // TODO Measure distance between left & ritgh wheels
  private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(1.0);

  public DriveTrain()
  {
    commonSettings(left_main);
    commonSettings(right_main);
    commonSettings(left_slave);
    commonSettings(right_slave);
    
    // Instruct slave motors to follow their respective main
    left_slave.follow(left_main);
    right_slave.follow(right_main);
    // left_slave.setSafetyEnabled(false);
    // right_slave.setSafetyEnabled(false);
    
    // No deadband on differential drive to allow
    // even small PID-driven moves.
    // If joystick needs dead zone, put that into OI.
    differential_drive.setDeadband(0.0);
    
    // Initially, set low gear
    setGear(false);
    
    position_pid.setTolerance(0.01, 0.01);
    gyro.calibrate();
    // Not using continuous,
    // heading_pid.enableContinuousInput(-180.0, 180.0),
    // because 360 degrees is not the same as 0 degrees.
    // To us, 360 means "turn one full rotation".
    heading_pid.setTolerance(0.1, 0.1);

    SmartDashboard.putData("Position PID", position_pid);
    SmartDashboard.putData("Heading PID", heading_pid);
    SmartDashboard.putData("Speed PID", speed_pid);

    reset();
  }

  /** @param motor Motor to configure with common settings */
  private void commonSettings(final WPI_TalonFX motor)
  {
    motor.configFactoryDefault();
    motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    motor.clearStickyFaults();
    motor.setNeutralMode(NeutralMode.Brake);
    // TODO Do this only for the main motors, follower will, well, follow?
    motor.configOpenloopRamp(1.0);
  }

  public void drive(final double speed, final double rotation)
  {
    differential_drive.arcadeDrive(speed, rotation);
  }

  public void driveSpeed(final double left_speed, final double right_speed)
  {
    // Predict necessary voltage, add the PID correction
    final double left_volt  = feed_forward.calculate(left_speed)
                            + speed_pid.calculate(getLeftSpeedMetersPerSecond(), left_speed);
    final double right_volt = feed_forward.calculate(right_speed)
                            + speed_pid.calculate(getRightSpeedMetersPerSecond(), right_speed);
    driveVoltage(left_volt, right_volt);
  }

  public void driveVoltage(final double left, final double right)
  {
    left_main.setVoltage(left);
    right_main.setVoltage(right);
    // When directly setting the motor voltage,
    // the differential_drive will trigger the motor safety
    // because it's not called frquently enough.
    // -> Pretend we called the differential_drive
    differential_drive.feed();
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
    // Right  encoder counts down, so '-' to add both
    final long avg_ticks = ((long)left_main.getSelectedSensorPosition() -
                                  right_main.getSelectedSensorPosition()) / 2;
    return avg_ticks / TICKS_PER_METER;
  }

  public double getLeftSpeedMetersPerSecond()
  {
    // "sensor per 100ms"
    return left_main.getSelectedSensorVelocity() * (10.0 / TICKS_PER_METER);
  }

  public double getRightSpeedMetersPerSecond()
  {
    // "sensor per 100ms"
    return right_main.getSelectedSensorVelocity() * (10.0 / TICKS_PER_METER);
  }

  public double getSpeedMetersPerSecond()
  {
    // Right  encoder counts down, so '-' to add both
    return (getLeftSpeedMetersPerSecond() -
            getRightSpeedMetersPerSecond()) / 2;
  }

  /** Reset all encoders to 0 */
  public void reset()
  {
    gyro.reset();
    heading_pid.setSetpoint(0);
    left_main.setSelectedSensorPosition(0);
    right_main.setSelectedSensorPosition(0);
    odometry.resetPosition(new Pose2d(), Rotation2d.fromDegrees(0));
  }

  /**@return Angle in degrees. 0 degrees is along X axis. 90 degrees is along Y axis. */
  public double getHeadingDegrees()
  {
    return -gyro.getAngle();
  }

  public PIDController getHeadingPID()
  {
    return heading_pid;
  }

  /** @param trajectory Trajectory
   *  @return Command that uses this drivebase to follow that trajectory
   */
  public CommandBase createRamsete(final Trajectory trajectory)
  {
    return new RamseteCommand(trajectory,
                              odometry::getPoseMeters,
                              new RamseteController(),
                              kinematics,
                              this::driveSpeed,
                              this);
  }

  @Override
  public void periodic()
  {
    // Update position tracker
    odometry.update(Rotation2d.fromDegrees(getHeadingDegrees()),
                     left_main.getSelectedSensorPosition() / TICKS_PER_METER,
                    -right_main.getSelectedSensorPosition()/ TICKS_PER_METER);
    // Publish odometry X, Y, Angle
    Pose2d pose = odometry.getPoseMeters();
    SmartDashboard.putNumber("X Position:", pose.getTranslation().getX());
    SmartDashboard.putNumber("Y Position:", pose.getTranslation().getY());
    SmartDashboard.putNumber("Angle: ", pose.getRotation().getDegrees());
                    
    SmartDashboard.putNumber("Position", getPositionMeters());
    SmartDashboard.putNumber("Speed", getSpeedMetersPerSecond());
    SmartDashboard.putNumber("Heading", getHeadingDegrees());

    SmartDashboard.putNumber("Motor Voltage", left_main.getMotorOutputVoltage());
  }
}
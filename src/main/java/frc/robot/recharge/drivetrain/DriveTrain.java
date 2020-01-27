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
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.recharge.RobotMap;

/** Drive train (them wheels)
 * 
 *  Geometry:
 *  Initial position is X=0, Y=0, angle=0, pointing toward the alliance station.
 *  Moving forward at 0 degree heading means moving along the X axis.
 *  Angle increases when turning left.
 *  Angle of 90 degrees means moving along the Y axis.
 */
public class DriveTrain extends SubsystemBase
{
  // Encoder ticks per meter,
  // based on driving some distance and reading raw ticks
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

  // Charact: kS - 0.845; kV - 3.56; kA - 0.66; r-squared 0.999; P - 18.7
  // Charact: kS - 0.778; kV - 3.6;  kA - 0.91; r-sqaured 0.999; P - 21.5
  private final SimpleMotorFeedforward feed_forward = new SimpleMotorFeedforward(0.846, 3.58, 0.175);
  // Left & right speed PID
  private final PIDController left_speed_pid = new PIDController(6.5, 0, 0);
  private final PIDController right_speed_pid = new PIDController(6.5, 0, 0);
  
  // Track current position based on gyro and encoders
  private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(0));

  // TODO Get 'distance' between left & right wheels from characterization
  public static final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0.672);

  public DriveTrain()
  {
    // Reset & configure motors
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

    reset();
  }

  /** @param motor Motor to configure with common settings */
  private void commonSettings(final WPI_TalonFX motor)
  {
    motor.configFactoryDefault();
    motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    motor.clearStickyFaults();
    motor.setNeutralMode(NeutralMode.Brake);
    // Do this only for the main motors, follower will, well, follow?
    // motor.configOpenloopRamp(1.0);
  }

  /** Reset all encoders to 0, low gear, ... */
  public void reset()
  {
    gyro.reset();
    left_main.setSelectedSensorPosition(0);
    right_main.setSelectedSensorPosition(0);
    odometry.resetPosition(new Pose2d(), Rotation2d.fromDegrees(0));
    setGear(false);
  }

  /** @return Is gear in high speed? */
  public boolean isHighSpeed()
  {
    return shifter.get();
  }

  /** @param high_speed Switch to high speed gearing? */
  public void setGear(final boolean high_speed)
  {
    shifter.set(high_speed);
  }

  /** @return Does robot stand still? */
  public boolean isIdle()
  {
    return left_main.getSelectedSensorVelocity() == 0  &&
           right_main.getSelectedSensorVelocity() == 0;
  }

  /** @return Left position (positive = forward) */
  public double getLeftPositionMeters()
  {
    return left_main.getSelectedSensorPosition() / TICKS_PER_METER;
  }

  /** @return Right position (positive = forward) */
  public double getRightPositionMeters()
  {
    return -right_main.getSelectedSensorPosition() / TICKS_PER_METER;
  }

  /** @return Averaged left/right position (positive = forward) */
  public double getPositionMeters()
  {
    // Right encoder counts down, so '-' to add both
    return (getLeftPositionMeters() + getRightPositionMeters()) / 2;
  }

  /** @return Left speed (positive = forward) */
  public double getLeftSpeedMetersPerSecond()
  {
    // "sensor per 100ms"
    return left_main.getSelectedSensorVelocity() * (10.0 / TICKS_PER_METER);
  }

  /** @return Right speed (positive = forward) */
  public double getRightSpeedMetersPerSecond()
  {
    // "sensor per 100ms"
    return -right_main.getSelectedSensorVelocity() * (10.0 / TICKS_PER_METER);
  }

  /** @return Averaged left/right speed (positive = forward) */
  public double getSpeedMetersPerSecond()
  {
    // Right  encoder counts down, so '-' to add both
    return (getLeftSpeedMetersPerSecond() +
            getRightSpeedMetersPerSecond()) / 2;
  }

  /** Fetch gyro angle in the 'normal' math coordinate system.
   *  0 degrees is along X axis. 90 degrees is along Y axis.
   *  Incrementing counter-clockwise when viewing robot from above.
   *  @return Angle in degrees.
   */
  public double getHeadingDegrees()
  {
    return -gyro.getAngle();
  }

  /** 'Arcade' drive
   *  @param speed Speed going forward
   *  @param rotation
   */ 
  public void drive(final double speed, final double rotation)
  {
    differential_drive.arcadeDrive(speed, rotation);
  }

  /** Direct control of left and right motors
   *  @param left Left speed -1..1
   *  @param right Right speed -1..1
   */
  public void tankDrive(final double left, final double right)
  {
    differential_drive.tankDrive(left, right, false);
  }

  /** Direct control of left and right motors
   *  @param left_speed Meters/second
   *  @param right_speed Meters/second
   */
  public void driveSpeed(final double left_speed, final double right_speed)
  {
    // Predict necessary voltage, add the PID correction
    final double left_volt  = feed_forward.calculate(left_speed)
                            + left_speed_pid.calculate(getLeftSpeedMetersPerSecond(), left_speed);
    final double right_volt = feed_forward.calculate(right_speed)
                            + right_speed_pid.calculate(getRightSpeedMetersPerSecond(), right_speed);
  //  System.out.println("Speeds: " + left_volt + ", " + right_volt);
    driveVoltage(left_volt, -right_volt);
  }

  /** Direct control of left and right motors
   *  @param left Motor voltage
   *  @param right Motor voltage
   */
  public void driveVoltage(final double left, final double right)
  {
    left_main.setVoltage(left);
    right_main.setVoltage(right);
    // When directly setting the motor voltage,
    // the differential_drive will trigger the motor safety
    // because it's not called frequently enough.
    // -> Pretend we called the differential_drive
    differential_drive.feed();
  }

  /** @return PID for holding position speed [-1..1] */
  public PIDController getPositionPID()
  {
    return position_pid;
  }

  /** @return PID for holding heading [-1..1] */
  public PIDController getHeadingPID()
  {
    return heading_pid;
  }

  /** Create RamseteCommand for following a trajectory
   *  with this drive base
   * 
   *  @param trajectory Trajectory to follow
   *  @return Command that uses this drivebase to follow that trajectory
   */
  public CommandBase createRamsete(final Trajectory trajectory)
  {
    return new RamseteCommand(trajectory,
                              odometry::getPoseMeters,
                              new RamseteController(2.0, 0.7),
                              kinematics,
                              this::driveSpeed,
                              this);
  }

  @Override
  public void periodic()
  {
    // Update position tracker
    odometry.update(Rotation2d.fromDegrees(getHeadingDegrees()),
                    getLeftPositionMeters(),
                    getRightPositionMeters());                   

    // Publish odometry X, Y, Angle
    Pose2d pose = odometry.getPoseMeters();
    SmartDashboard.putNumber("X Position:", pose.getTranslation().getX());
    SmartDashboard.putNumber("Y Position:", pose.getTranslation().getY());
    SmartDashboard.putNumber("Angle: ", pose.getRotation().getDegrees());
                    
    // SmartDashboard.putNumber("Position", getPositionMeters());
    // SmartDashboard.putNumber("Left Speed", getLeftSpeedMetersPerSecond());
    // SmartDashboard.putNumber("Right Speed", getRightSpeedMetersPerSecond());
    // SmartDashboard.putNumber("Heading", getHeadingDegrees());

    // SmartDashboard.putNumber("Motor Voltage", left_main.getMotorOutputVoltage());
  }
}
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.BasicRobot;
import frc.robot.recharge.auto.ApplySettings;
import frc.robot.recharge.auto.AutonomousBuilder;
import frc.robot.recharge.drivetrain.AutoShift;
import frc.robot.recharge.drivetrain.DriveByJoystick;
import frc.robot.recharge.drivetrain.DriveTrain;
import frc.robot.recharge.drivetrain.HeadingHold;
import frc.robot.recharge.drivetrain.Reset;
import frc.robot.recharge.drivetrain.RotateToTarget;
import frc.robot.recharge.shooter.Eject;
import frc.robot.recharge.shooter.Hood;
import frc.robot.recharge.shooter.Intake;
import frc.robot.recharge.shooter.IntakeDown;
import frc.robot.recharge.shooter.IntakeMid;
import frc.robot.recharge.shooter.IntakeUp;
import frc.robot.recharge.shooter.Load;
import frc.robot.recharge.shooter.PowerCellAccelerator;

/**
 * Robot for 'Infinite Recharge' - R!$E2geTHeR#2020
 */
public class Enterprise extends BasicRobot
{
  private final DriveTrain drive_train = new DriveTrain();

  // Commands that require the drive train, i.e. starting any of these commands
  // will cancel whatever else was running and required the drive train
  private final CommandBase reset_drivetrain = new Reset(drive_train);
  private final CommandBase drive_by_joystick = new DriveByJoystick(drive_train);
  private final HeadingHold heading_hold = new HeadingHold(drive_train);
  /** Most recent drive mode, either drive_by_joystick or heading_hold */
  private CommandBase drive_mode = heading_hold;
  // After align_on_target, return to current drive_mode
  private final CommandBase align_on_target = new RotateToTarget(drive_train);
  // Indicate heading_hold vs. drive_by_joystick
  private final Rumble rumble = new Rumble();
  
  // Shift commands can run concurrently with other commands that require the
  // drive train
  private final CommandBase shift_low = new InstantCommand(() -> drive_train.setGear(false));
  private final CommandBase shift_high = new InstantCommand(() -> drive_train.setGear(true));
  private final CommandBase auto_shift = new AutoShift(drive_train);

  // Ball handling
  private final Intake intake = new Intake();
  private final CommandBase intake_up = new IntakeUp(intake);
  private final CommandBase intake_down = new IntakeDown(intake);
  private final CommandBase intake_mid = new IntakeMid(intake);

  private final PowerCellAccelerator pca = new PowerCellAccelerator();
  private final CommandBase load = new Load(pca);
  private final CommandBase eject = new Eject(pca);
  
  private final Hood hood = new Hood();

  // TODO Connect control wheel commands to buttons
  // private final ColorSensor color_sensor = new ColorSensor();

  // private final ControlWheel fortune = new ControlWheel();
  // private final Command manual_wheel = new ManualWheelSpeed(fortune);
  // private final Command rotate_wheel = new RotateWheel(fortune, 3);
  // private final Command rotate_to_color = new RotateToColor(fortune);

  // private final LEDStrip led_strip = new LEDStrip();

  private final SendableChooser<Command> auto_commands = new SendableChooser<>();

  @Override
  public void robotInit()
  {
    super.robotInit();

    // TODO Remove test settings
    PowerCellAccelerator.SHOOTER_RPM = 2000;
    // pcm.clearAllPCMStickyFaults();

    // Manual shifting
    // Note that DriveByJoystick will automatically shift,
    // so while we can invoke 'shift_high' via button,
    // we would automatically shift back low when standing still.
    OI.shift_low.whenActive(shift_low);
    OI.shift_high.whenPressed(shift_high);

    // Bind buttons to actions (only active in teleop)
    // Pressing 'A' enables manual wheel control (and stops auto rotation)
    // OI.enable_wheel.whenPressed(manual_wheel);
    // // Pressing 'B' turns wheel automatically, then re-enables manual control
    // OI.autorotate_wheel.whenPressed(rotate_wheel.andThen(() ->
    // manual_wheel.schedule()));
    // // Pressing 'X' turns wheel to the desired color
    // OI.rotate_to_color.whenPressed(rotate_to_color.andThen(() ->
    // manual_wheel.schedule()));


    // Place some commands on dashboard
    SmartDashboard.putData("Reset Drive", reset_drivetrain);
    SmartDashboard.putData("Auto Shift", auto_shift);
    // SmartDashboard.putData("Heading Hold", heading_hold);
    // SmartDashboard.putData("Drive by Joystick", drive_by_joystick);
    
    SmartDashboard.putData("Intake Up", intake_up);
    SmartDashboard.putData("Intake Down", intake_down);
    SmartDashboard.putData("Intake Mid", intake_mid);

    SmartDashboard.setDefaultNumber("Hood Setpoint", -1);


    // Auto options: Start with fixed options
    auto_commands.setDefaultOption("Nothing", new PrintCommand("Doing nothing"));
    try
    {
      // Add moves from auto.txt
      final File auto_file = new File(Filesystem.getDeployDirectory(), "auto.txt");
      for (CommandBase moves : AutonomousBuilder.read(auto_file, drive_train, intake, hood))
        auto_commands.addOption(moves.getName(), moves);
    }
    catch (Exception ex)
    {
      System.err.println("Error in auto.txt:");
      ex.printStackTrace();
    }
    SmartDashboard.putData("Autonomous", auto_commands);

    // Allow selecting settings for different scenarios

    // Settings for different scenarios
    final CommandBase default_settings;
    SmartDashboard.putData("Near Settings", default_settings = new ApplySettings("near.txt"));
    SmartDashboard.putData("Far Settings", new ApplySettings("far.txt"));
    SmartDashboard.putData("Viewable Settings", new ApplySettings("viewable.txt"));
    default_settings.schedule();
  }

  @Override
  public void robotPeriodic()
  {
    super.robotPeriodic();

    // Control hood angle via manual entry on dashboard or ApplySettings()
    hood.setHoodAngle(SmartDashboard.getNumber("Hood Setpoint", -1));
  }

  @Override
  public void teleopInit()
  {
    super.teleopInit();

    auto_shift.schedule();
    drive_mode.schedule();
  }

  @Override
  public void teleopPeriodic()
  {
    // TODO Indicate direction to target on LED
    // final double direction = SmartDashboard.getNumber("Direction", 0) / 160;
    // TODO Filter direction sent by Raspberry/camera via
    // https://docs.wpilib.org/en/latest/docs/software/advanced-control/filters/median-filter.html
    // led_strip.indicateDirection(direction);

    // Toggle between drive_by_joystick and heading_hold
    if (OI.isToggleHeadingholdPressed())
    {
      if (drive_mode == heading_hold)
      { // Switching to 'rough' mode
        rumble.schedule(0.5);
        drive_mode = drive_by_joystick;
      }
      else
      { // Switching to 'smooth' mode
        rumble.schedule(0.1);
        drive_mode = heading_hold;
      }
      drive_mode.schedule();
    }

    // Holding the 'shoot' button starts or re-starts the command to shoot one ball.
    if (OI.isShootHeld())
      eject.schedule();
    // Otherwise we allow ongoing 'eject' to finish, then keep 'load'ing
    else if (eject.isFinished())
      load.schedule();

    // Align on target?
    if (OI.isAlignOnTargetHeld())
    {
      if (! align_on_target.isScheduled())
        align_on_target.schedule();
    }
    else
    {
        // Re-enable original drive mode, which cancels alignment
        if (! drive_mode.isScheduled())
          drive_mode.schedule();
    }
  }

  @Override
  public void autonomousInit()
  {
    super.autonomousInit();

    // Run the selected command.
    drive_train.reset();
    auto_commands.getSelected().schedule();
  }

  @Override
  public void autonomousPeriodic()
  {
    // TODO led_strip.rainbow();? Also indicate direction to target to debug what the robot sees?  
  }
}

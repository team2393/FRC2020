/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.BasicRobot;
import frc.robot.recharge.auto.ApplySettings;
import frc.robot.recharge.auto.AutonomousBuilder;
import frc.robot.recharge.climb.ClimbIdle;
import frc.robot.recharge.climb.Climber;
import frc.robot.recharge.climb.ControlClimber;
import frc.robot.recharge.ctrlpanel.ControlWheel;
import frc.robot.recharge.ctrlpanel.ExtendControlWheel;
import frc.robot.recharge.ctrlpanel.ManualWheelSpeed;
import frc.robot.recharge.ctrlpanel.RetractControlWheel;
import frc.robot.recharge.ctrlpanel.RotateToColor;
import frc.robot.recharge.ctrlpanel.RotateWheel;
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
import frc.robot.recharge.shooter.ShooterIdle;

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
  private CommandBase drive_mode = drive_by_joystick;
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
  private final CommandBase shooter_idle = new ShooterIdle(pca);
  private final CommandBase load = new Load(pca);
  private final CommandBase eject = new Eject(pca);
  
  private final Hood hood = new Hood();

  private final ControlWheel fortune = new ControlWheel();
  private final CommandBase extend_wheel = new ExtendControlWheel(fortune);
  private final CommandBase retract_wheel = new RetractControlWheel(fortune);
  private final CommandBase manual_wheel = new ManualWheelSpeed(fortune);
  private final CommandBase rotate_wheel = new RotateWheel(fortune, 3);
  private final CommandBase rotate_to_color = new RotateToColor(fortune);

  private Climber climber = new Climber();
  private CommandBase climb_idle = new ClimbIdle(climber);
  private CommandBase control_climb = new ControlClimber(climber);

  // private final LEDStrip led_strip = new LEDStrip();

  private final SendableChooser<Command> auto_commands = new SendableChooser<>();

  // Teleop modes
  private enum TeleopMode
  {
    Drive,
    Climb,
    Wheel
  };

  private TeleopMode teleop_mode = TeleopMode.Drive;

  @Override
  public void robotInit()
  {
    super.robotInit();

    // pcm.clearAllPCMStickyFaults();

    // Place some commands on dashboard
    SmartDashboard.putData("Reset Drive", reset_drivetrain);
    SmartDashboard.putData("Auto Shift", auto_shift);
    // SmartDashboard.putData("Heading Hold", heading_hold);
    // SmartDashboard.putData("Drive by Joystick", drive_by_joystick);
    
    // SmartDashboard.putData("Intake Up", intake_up);
    // SmartDashboard.putData("Intake Down", intake_down);
    // SmartDashboard.putData("Intake Mid", intake_mid);

    SmartDashboard.setDefaultNumber("Hood Setpoint", -1);
    SmartDashboard.setDefaultNumber("Shooter RPM", PowerCellAccelerator.SHOOTER_RPM);

    // Auto options: Start with fixed options
    auto_commands.setDefaultOption("Nothing", new PrintCommand("Doing nothing"));
    try
    {
      // Add moves from auto.txt
      final File auto_file = new File(Filesystem.getDeployDirectory(), "auto.txt");
      for (CommandBase moves : AutonomousBuilder.read(auto_file, drive_train, intake, pca, hood))
        auto_commands.addOption(moves.getName(), moves);
    }
    catch (Exception ex)
    {
      System.out.println("Error in auto.txt:");
      ex.printStackTrace();
      System.out.println("========================\n\n\n");

      Timer.delay(10.0);
    }
    SmartDashboard.putData("Autonomous", auto_commands);

    // Allow selecting settings for different scenarios

    // Settings for different scenarios
    final CommandBase default_settings;
    SmartDashboard.putData("Near Settings", new ApplySettings("near.txt"));
    SmartDashboard.putData("Far Settings", default_settings = new ApplySettings("far.txt"));
    SmartDashboard.putData("Viewable Settings", new ApplySettings("viewable.txt"));
    default_settings.schedule();
  }

  @Override
  public void disabledInit()
  {
    drive_train.lock(false);
  }
  
  @Override
  public void robotPeriodic()
  {
    super.robotPeriodic();

    PowerCellAccelerator.SHOOTER_RPM = SmartDashboard.getNumber("Shooter RPM", PowerCellAccelerator.SHOOTER_RPM);
    SmartDashboard.putString("Teleop Mode", teleop_mode.toString());
  }
  
  @Override
  public void teleopInit()
  {
    super.teleopInit();
    drive_train.lock(true);
    OI.reset();
    auto_shift.schedule();
    drive_mode.schedule();
    load.schedule();
  }
  
  @Override
  public void teleopPeriodic()
  {
    // Control hood angle via manual entry on dashboard or ApplySettings()
    hood.setHoodAngle(SmartDashboard.getNumber("Hood Setpoint", -1));
    if (teleop_mode == TeleopMode.Drive)
      teleop_drive();
    else if (teleop_mode == TeleopMode.Climb)
      teleop_climb();
    else if (teleop_mode == TeleopMode.Wheel)
      teleop_wheel();
  }

  private void teleop_drive()
  {
    if (OI.selectClimbMode())
    {
      teleop_mode = TeleopMode.Climb;
      return;
    }
    else if (OI.selectWheelMode())
    {
      teleop_mode = TeleopMode.Wheel;
      extend_wheel.schedule();
      return;
    }

    // Disable climb control
    climb_idle.schedule();

    if (! auto_shift.isScheduled())
    {
      // Manual shifting
      if (OI.isLowGearRequested())
        shift_low.schedule();
      else if (OI.isHighGearRequested())
        shift_high.schedule();
    }
    
    // Toggle between drive_by_joystick and heading_hold
    OI.force_low_speed = false;
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

    if (OI.isIntakeDownRequested())
      intake_down.schedule();
    else if (OI.isIntakeUpRequested())
    {
      // First, get to 'mid' angle.
      // If already there, go 'up'
      if (intake.getAngle() < 15)
        intake_mid.schedule();
      else
        intake_up.schedule();
    }

    // Holding the 'shoot' button starts or re-starts the command to shoot one ball.
    if (OI.isShootHeld())
      eject.schedule();
    // Otherwise we allow ongoing 'eject' to finish, then keep 'load'ing
    else if (eject.isFinished())
      load.schedule();

    // Align on target?
    if (OI.isAlignOnTargetHeld())
        align_on_target.schedule();
    else
    {
        // Re-enable original drive mode, which cancels alignment
        drive_mode.schedule();
    }
  }

  private void teleop_climb()
  {
    if (OI.selectDriveMode())
    {
      teleop_mode = TeleopMode.Drive;
      return;
    }

    intake_up.schedule();
    shooter_idle.schedule();

    auto_shift.cancel();
    shift_low.schedule();
    OI.force_low_speed = true;
    drive_by_joystick.schedule();
    
    control_climb.schedule();
  }

  private void teleop_wheel()
  {
    if (OI.selectDriveMode())
    {
      teleop_mode = TeleopMode.Drive;
      retract_wheel.schedule();
      return;
    }

    intake_up.schedule();
    shooter_idle.schedule();

    auto_shift.cancel();
    shift_low.schedule();
    OI.force_low_speed = true;
    drive_by_joystick.schedule();
    
    if (OI.isAutorotateWheelRequested())
      rotate_wheel.schedule();
    else if (OI.isRotateToColorRequested())
      rotate_to_color.schedule();

    if (!rotate_to_color.isScheduled()  &&   !rotate_wheel.isScheduled())
      manual_wheel.schedule();
  }

  @Override
  public void autonomousInit()
  {
    super.autonomousInit();

    OI.reset();
    hood.reset();
    drive_train.reset();
    drive_train.lock(true);
    
    // Run the selected command.
    auto_commands.getSelected().schedule();
  }

  @Override
  public void autonomousPeriodic()
  {
    // If nothing else is using the shooter: Load
    if (pca.getCurrentCommand() == null)
      load.schedule();
  }
}

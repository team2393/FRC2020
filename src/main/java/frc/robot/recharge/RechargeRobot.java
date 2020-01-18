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
import frc.robot.recharge.auto.AutonomousBuilder;
import frc.robot.recharge.ctrlpanel.ColorSensor;
import frc.robot.recharge.ctrlpanel.ControlWheel;
import frc.robot.recharge.ctrlpanel.ManualWheelSpeed;
import frc.robot.recharge.ctrlpanel.RotateToColor;
import frc.robot.recharge.ctrlpanel.RotateWheel;
import frc.robot.recharge.drivetrain.AutoShift;
import frc.robot.recharge.drivetrain.DriveByJoystick;
import frc.robot.recharge.drivetrain.DriveToPosition;
import frc.robot.recharge.drivetrain.DriveTrain;
import frc.robot.recharge.drivetrain.HeadingHold;
import frc.robot.recharge.drivetrain.TurnToHeading;
import frc.robot.recharge.led.LEDStrip;

/** Robot for 'Infinite Recharge' - R!$E2geTHeR#2020
 */
public class RechargeRobot extends BasicRobot
{  
  private final DriveTrain drive_train = new DriveTrain();
  
  private final CommandBase drive_by_joystick = new DriveByJoystick(drive_train);
  private final CommandBase auto_shift = new AutoShift(drive_train);
  private final DriveToPosition drive_to_position = new DriveToPosition(drive_train);
  private final TurnToHeading turn_to_heading = new TurnToHeading(drive_train);
  private final HeadingHold heading_hold = new HeadingHold(drive_train);
  private final CommandBase reset_drivetrain = new InstantCommand(drive_train::reset);
  private final CommandBase shift_low = new InstantCommand(() -> drive_train.setGear(false));
  private final CommandBase shift_high = new InstantCommand(() -> drive_train.setGear(true));

  // TODO Tune PIDs for drive-to-position, turn-to-heading with actual robot
  // TODO Trajectory: Create, follow
  // TODO Use simple position and heading PID to follow trajectory
  // TODO RamseteCommand to follow trajectory
  // TODO Command to drive left/right based on vision info (in network tables, set by pi)
  // TODO Control motor w/ encoder for lowering/raising intake
  // TODO Control motors for intake, conveyor belt, shooter
  // TODO IR detector to check if there are any balls in hopper?
  // TODO Connect control wheel commands to buttons
  // TODO Climb: Raise/lower telescope via motor that pulls/gives rope
  // TODO Climb: Pull climbin rope _in_ (cannot feed out because of ratchet)
  
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
    // pcm.clearAllPCMStickyFaults();
    
    // Bind buttons to actions (only active in teleop)
    // Pressing 'A' enables manual wheel control (and stops auto rotation)
    // OI.enable_wheel.whenPressed(manual_wheel);
    // // Pressing 'B' turns wheel automatically, then re-enables manual control
    // OI.autorotate_wheel.whenPressed(rotate_wheel.andThen(() -> manual_wheel.schedule()));
    // // Pressing 'X' turns wheel to the desired color
    // OI.rotate_to_color.whenPressed(rotate_to_color.andThen(() -> manual_wheel.schedule()));

    // Manual shifting
    // Note that DriveByJoystick will automatically shift,
    // so while we can invoke 'shift_high' via button,
    // we would automatically shift back low when standing still.
    OI.shift_low.whenActive(shift_low);
    OI.shift_high.whenPressed(shift_high);

    // Place some commands on dashboard
    SmartDashboard.putData("Reset Drive", reset_drivetrain);
    SmartDashboard.putData("Auto Shift", auto_shift);
    SmartDashboard.putData("Heading Hold", heading_hold);
    SmartDashboard.putData("Drive by Joystick", drive_by_joystick);

    // Auto options: Start with fixed options
    auto_commands.setDefaultOption("Nothing", new PrintCommand("Doing nothing"));
    auto_commands.addOption("Another", new PrintCommand("Another option"));
    // Add moves from auto.txt
    try
    {
      // Read commands from auto file.
      // Drivebase turns trajectories into ramsete commands.
      final File auto_file = new File(Filesystem.getDeployDirectory(), "auto.txt");
      for (CommandBase moves : AutonomousBuilder.read(auto_file, drive_train::createRamsete))
        auto_commands.addOption(moves.getName(), moves);
    }
    catch (Exception ex)
    {
      System.err.println("Error in auto.txt:");
      ex.printStackTrace();
    }
    SmartDashboard.putData("Autonomous", auto_commands);
  }
  
  @Override
  public void disabledInit()
  {
    super.disabledInit();
    // led_strip.idle();
  }

  @Override
  public void teleopInit()
  {
    // TODO add toggle between modes
    super.teleopInit();
    // manual_wheel.schedule();
    drive_by_joystick.schedule();
    // auto_shift.schedule();
    // heading_hold.schedule();
  }
  
  @Override
  public void teleopPeriodic()
  {
    // TODO Indicate direction to target on LED
    //final double direction = OI.getDirection();
    final double direction = SmartDashboard.getNumber("Direction", 0) / 160;
    // led_strip.indicateDirection(direction);    
  }
  
  @Override
  public void autonomousInit()
  {
    super.autonomousInit();

    auto_commands.getSelected().schedule();

    // drive_to_position.schedule();
    // turn_to_heading.schedule();
  }

  @Override
  public void autonomousPeriodic()
  {
    // TODO led_strip.rainbow();? Also indicate direction to target to debug what the robot sees?
  
    // Every 3 seconds, toggle between two positions
    // long test_index = (System.currentTimeMillis() / 3000) % 2;
    // // double test_pos_meters = 2;
    // // drive_to_position.setDesiredPosition(test_index * test_pos_meters);

    // double test_degrees = 10;
    // // turn_to_heading.setDesiredHeading(test_index * test_degrees);
    // if (turn_to_heading.isFinished())
    // {
    //   System.out.println("At heading");
    //   turn_to_heading.schedule();
    // }
  }
}

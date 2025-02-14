// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.LEDs.FlashSolidLEDCommand;
import frc.robot.LEDs.RepeatedFlashLEDCommand;
import frc.robot.auto.AutoProvider;
import frc.robot.commands.ActivateAlgaeL2;
import frc.robot.commands.ActivateAlgaeL3;
import frc.robot.commands.ActivateAlgaeUnload;
import frc.robot.commands.ActivateCoralL1;
import frc.robot.commands.ActivateCoralL2;
import frc.robot.commands.ActivateCoralL3;
import frc.robot.commands.ActivateCoralLoad;
import frc.robot.commands.resetAll;
import frc.robot.subsystems.FlowerSub;
import frc.robot.teleop.TeleopProvider;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including
 * Subsystemsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final AutoProvider autoProvider = AutoProvider.getInstance();
  private final TeleopProvider teleopProvider = TeleopProvider.getInstance();

  /**
   * The container for the robot. Contains Subsystemsystems, OI devices, and
   * commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Robot Automations
    // flash leds yellow during endgame
    new Trigger(() -> DriverStation.isTeleop() && DriverStation.getMatchTime() <= 30)
        .onTrue(new RepeatedFlashLEDCommand(
            (FlashSolidLEDCommand) (new FlashSolidLEDCommand(Color.kYellow, 300).withZone()), 5));

    // +----------------+
    // | PILOT CONTROLS |
    // +----------------+

    // --- Manual Controls ---

    // Invert Drive
    OI.pilot.start().onTrue(new InstantCommand(() -> Variables.invertDriveDirection = !Variables.invertDriveDirection));

    // OI.pilot.y().onTrue(new
    // InstantCommand(()->BatteryPercentLEDCommand.runFor(50)));
    OI.pilot.a().onTrue(new FlashSolidLEDCommand(Color.kCrimson, 1000).withZone());
    OI.pilot.b().onTrue(new RepeatedFlashLEDCommand(
        (FlashSolidLEDCommand) (new FlashSolidLEDCommand(Color.kYellow, 200).withZone(new int[] { 1, 2 })),
        5).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    OI.pilot.x().onTrue(new RepeatedFlashLEDCommand(
        (FlashSolidLEDCommand) (new FlashSolidLEDCommand(Color.kBlue, 200).withZone(new int[] { 0 })),
        5));

    OI.copilot.povUp().whileTrue(new ActivateCoralL3());
    OI.copilot.povRight().whileTrue(new ActivateCoralL2());
    OI.copilot.povDown().whileTrue(new ActivateCoralL1());
    OI.copilot.povLeft().whileTrue(new ActivateCoralLoad());
    
    OI.copilot.y().whileTrue(new ActivateAlgaeL3());
    OI.copilot.b().whileTrue(new ActivateAlgaeL2());
    OI.copilot.a().whileTrue(new ActivateAlgaeUnload());

    OI.copilot.rightBumper()
        .onTrue(new InstantCommand(() -> Subsystems.flower.extend()))
        .onFalse(new InstantCommand(() -> Subsystems.flower.retract()));
    
    OI.copilot.leftBumper()
        .onTrue(new InstantCommand(() -> Subsystems.algaeGrabber.unload()))
        .onFalse(new InstantCommand(() -> Subsystems.algaeGrabber.stop()));
    OI.copilot.leftTrigger()
        .onTrue(new InstantCommand(() -> Subsystems.algaeGrabber.loadSlow()))
        .onFalse(new InstantCommand(() -> Subsystems.algaeGrabber.stop()));

    OI.copilot.start().onTrue(new resetAll());
    
    // Drive bindings handled in teleop command
  }

  /**
   * Use this to pass the teleop command to the main {@link Robot} class.
   *
   * @return the command to run in teleop
   */
  public Command getTeleopCommand() {
    return teleopProvider.getSelected();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoProvider.getSelected();
  }
}

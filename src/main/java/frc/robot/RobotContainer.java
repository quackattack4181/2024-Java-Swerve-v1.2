// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final SendableChooser<Command> autoChooser;
  
  /* Controllers */
  public final static XboxController driver = new XboxController(0);
  public final static XboxController driverTwo = new XboxController(1);

  public static double InvertControlModifier = 1.00;


  /* Subsystems */
  public final Swerve s_Swerve = new Swerve();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    NamedCommands.registerCommand("shootRingClose", Robot.shootRingClose());
    NamedCommands.registerCommand("shootRingLine", Robot.shootRingLine());
    NamedCommands.registerCommand("runRingIntake", Robot.runRingIntake());


    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            () -> -driver.getLeftY() * InvertControlModifier,
            () -> driver.getLeftX() * InvertControlModifier,
            () -> driver.getRightX() * 0.2,
            () -> false));
            // () -> robotCentric.getAsBoolean()));

    // Configure the button bindings
    configureButtonBindings();

    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* Driver Buttons */

    // if (driver.getBackButton()) {
    //   if (driver.getStartButton()) {
    //     s_Swerve.zeroGyro();
    //   }
    // }


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  // public Command getAutonomousCommand() {
  //   return new PathPlannerAuto("Test Auto");
  // }
}

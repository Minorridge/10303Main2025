// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.AlignToDynamicReefTagCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.CoralSubsystem.Setpoint;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {

  private final SendableChooser<Command> autoChooser;
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final CoralSubsystem m_coralSubSystem = new CoralSubsystem();
  private final AlgaeSubsystem m_algaeSubsystem = new AlgaeSubsystem();

  CommandXboxController m_driverController =
      new CommandXboxController(OIConstants.kDriverControllerPort);
    // ---> DEFINE YOUR DESIRED ROBOT OFFSET RELATIVE TO THE TAG <---
    // Example: 0.5m directly forward (+X) from tag center, facing tag (180 deg)
    // ** Adjust X distance (0.5) and Angle (180.0) as needed for your robot/game **
    private final Pose2d ROBOT_OFFSET_FROM_TAG =
        new Pose2d(0.4, 0.0, Rotation2d.fromDegrees(180.0));
         // ---> INSTANTIATE THE COMMAND <---
    // Pass in the drive subsystem, Limelight name, and the desired offset
    private final AlignToDynamicReefTagCommand m_alignToReefCommand =
        new AlignToDynamicReefTagCommand(m_robotDrive, "limelight", ROBOT_OFFSET_FROM_TAG);

  public RobotContainer() {
    NamedCommands.registerCommand("L2", m_coralSubSystem.setSetpointCommand(Setpoint.kLevel2).withTimeout(4));
    NamedCommands.registerCommand("Feeder", m_coralSubSystem.setSetpointCommand(Setpoint.kFeederStation));
    NamedCommands.registerCommand(
        "RunCoralIntake", m_coralSubSystem.runIntakeCommand().withTimeout(2));
    NamedCommands.registerCommand(
        "ReverseCoralIntake", m_coralSubSystem.reverseIntakeCommand().withTimeout(2));
    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    SmartDashboard.putData("Auto Chooser", autoChooser);
    // Configure the button bindings
    configureButtonBindings();

    m_robotDrive.setDefaultCommand(
        new RunCommand(
            () ->
                m_robotDrive.drive(
                    -MathUtil.applyDeadband(
                        m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(
                        m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(
                        m_driverController.getRightX(), OIConstants.kDriveDeadband),
                    true),
            m_robotDrive));

    m_algaeSubsystem.setDefaultCommand(m_algaeSubsystem.idleCommand());
  }

  private void configureButtonBindings() {
    m_driverController.leftStick().whileTrue(m_robotDrive.setXCommand());
    m_driverController.leftBumper().whileTrue(m_coralSubSystem.runIntakeCommand());
    m_driverController.rightBumper().whileTrue(m_coralSubSystem.reverseIntakeCommand());
    m_driverController
        .b()
        .onTrue(
            m_coralSubSystem
                .setSetpointCommand(Setpoint.kFeederStation)
                .alongWith(m_algaeSubsystem.stowCommand()));
    m_driverController.a().onTrue(m_coralSubSystem.setSetpointCommand(Setpoint.kLevel2));
    m_driverController.x().onTrue(m_coralSubSystem.setSetpointCommand(Setpoint.kLevel3));
    m_driverController.y().onTrue(m_coralSubSystem.setSetpointCommand(Setpoint.kLevel4));
    m_driverController
        .rightTrigger(OIConstants.kTriggerButtonThreshold)
        .whileTrue(m_algaeSubsystem.runIntakeCommand());
    m_driverController
        .leftTrigger(OIConstants.kTriggerButtonThreshold)
        .whileTrue(m_algaeSubsystem.reverseIntakeCommand());
    m_driverController.start().onTrue(m_robotDrive.zeroHeadingCommand());
    m_driverController.povDown().onTrue(m_alignToReefCommand);

  }



  public DriveSubsystem getDriveSubsystem() {
    return m_robotDrive;
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}

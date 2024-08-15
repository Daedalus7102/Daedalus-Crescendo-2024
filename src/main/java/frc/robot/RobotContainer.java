// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.APMScoreAutomatically;
import frc.robot.commands.ShootingAimAutomatically;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ShootNoteAutomatically;
import frc.robot.commands.SpinIntakeRollersManual;
import frc.robot.commands.MoveClimber;
import frc.robot.commands.PivotIntakeAutomatically;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive.Chassis;
import frc.robot.subsystems.Climber;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ConstantsShooter;
import frc.robot.Constants.IntakeConstants;

public class RobotContainer {
  private static final Chassis chasis = new Chassis();
  private static final Intake intake = new Intake();
  private static final Climber climber = new Climber();
  private static final Shooter shooter = new Shooter();

  private static final PS4Controller driveControl = new PS4Controller(0);
  public static final PS4Controller mecanismsControl = new PS4Controller(1);

  private final SendableChooser<Command> autoChooser;


  public RobotContainer() {
    NamedCommands.registerCommand("SHOOT", new ShootNoteAutomatically(shooter, intake, -ConstantsShooter.shooterMotorVelocity, IntakeConstants.intakeMotorVelocityThrowForShooter).withTimeout(0.8));
    NamedCommands.registerCommand("LOWER_INTAKE", new PivotIntakeAutomatically(intake, 1).withTimeout(1.2));
    NamedCommands.registerCommand("RISE_INTAKE", new PivotIntakeAutomatically(intake, 3));
    NamedCommands.registerCommand("AIMBOT", new ShootingAimAutomatically(chasis, shooter, intake).withTimeout(4));
    
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode", autoChooser);

    chasis.setDefaultCommand(
      new DriveCommand(
        chasis,
        () -> (-driveControl.getRawAxis(1)),
        () -> (driveControl.getRawAxis(0)),
        () -> (driveControl.getRawAxis(2))
      )
    );

    configureBindings();
  }

  private void configureBindings() {
    //Chassis driver controls
    new JoystickButton(driveControl, Constants.IOConstants.buttonTriangle).whileTrue(new RunCommand(chasis::zeroHeading));
    new JoystickButton(driveControl, Constants.IOConstants.triggerRight).whileTrue(new PivotIntakeAutomatically(intake, 1)); //Floor
    new JoystickButton(driveControl, Constants.IOConstants.triggerRight).whileFalse(new PivotIntakeAutomatically(intake, 3)); //Shooter
    new JoystickButton(driveControl, Constants.IOConstants.buttonCross).whileTrue(new ShootingAimAutomatically(chasis, shooter, intake));

    //Mechanisms driver controls
    new JoystickButton(mecanismsControl, Constants.IOConstants.triggerRight).whileTrue(new SpinIntakeRollersManual(intake, IntakeConstants.intakeMotorVelocityThrow));
    new JoystickButton(mecanismsControl, Constants.IOConstants.triggerLeft).whileTrue(new SpinIntakeRollersManual(intake, IntakeConstants.intakeMotorVelocitySuck));

    new POVButton(mecanismsControl, Constants.IOConstants.arrowDown).whileTrue(new MoveClimber(climber, ClimberConstants.rise));
    new POVButton(mecanismsControl, Constants.IOConstants.arrowUp).whileTrue(new MoveClimber(climber, ClimberConstants.lower));
    
    new JoystickButton(mecanismsControl, Constants.IOConstants.buttonTriangle).toggleOnTrue(new ShootNoteAutomatically(shooter, intake, -ConstantsShooter.shooterMotorVelocity, IntakeConstants.intakeMotorVelocityThrowForShooter));      
    new JoystickButton(mecanismsControl, Constants.IOConstants.buttonSquare).toggleOnTrue(new APMScoreAutomatically(shooter, intake)); 

    new POVButton(mecanismsControl, Constants.IOConstants.arrowLeft).whileTrue(new PivotIntakeAutomatically(intake, 1)); //Floor
    new JoystickButton(mecanismsControl, Constants.IOConstants.bumperRight).toggleOnTrue(new PivotIntakeAutomatically(intake, 2)); //Amp    
    new JoystickButton(mecanismsControl, Constants.IOConstants.bumoerLeft).toggleOnTrue(new PivotIntakeAutomatically(intake, 3)); //Shooter

    new POVButton(mecanismsControl, Constants.IOConstants.arrowLeft).whileFalse(new PivotIntakeAutomatically(intake, 3)); //Shooter
  }

  public Command getAutonomousCommand() {
    //Reads the information sent from the auto chooser
    return autoChooser.getSelected();
  }

  public Chassis getChasisSubsystem() {
    return chasis;
  }

  public Intake getIntakeSubsystem() {
    return intake;
  }
}

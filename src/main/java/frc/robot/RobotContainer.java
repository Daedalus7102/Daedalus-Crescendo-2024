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
import frc.robot.commands.AutoAMPScore;
import frc.robot.commands.AutoAimLimelight;
import frc.robot.commands.Manejo;
import frc.robot.commands.MoverShooterAutomatico;
import frc.robot.commands.intakeNoteManual;
import frc.robot.commands.moverClimberManual;
import frc.robot.commands.moverShooterManual;
import frc.robot.commands.PivotearIntakeAutomatico;
import frc.robot.subsystems.Chasis;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Leds;
import frc.robot.utils.PathChooser;
import frc.robot.utils.ThePaths;
import frc.robot.subsystems.Climber;
import frc.robot.Constants.ConstantesClimber;
import frc.robot.Constants.ConstantesIO;
import frc.robot.Constants.ConstantesIntake;
import frc.robot.Constants.ConstantsShooter;

//El container construye el chasis y el módulo directamente (accede a esas clases estableciendo unión al código del robot)
public class RobotContainer {
  private static final Chasis chasis = new Chasis();
  private static final Intake intake = new Intake();
  private static final Climber climber = new Climber();
  private static final Shooter shooter = new Shooter();
  private static final Leds leds = new Leds();

  private static final PS4Controller driveControl = new PS4Controller(0);
  public static final PS5Controller mecanismsControl = new PS5Controller(1);

  // private frc.robot.utils.PathChooser pathChooser;

  private final SendableChooser<Command> autoChooser;


  public RobotContainer() {
    //Para que la información de los controles se actualice constantemente, el método get de la clase "Manejo" pedirá la información 
    //a esta clase, para así en vez de dejar un valor fijo, asignar valores requeridos al robot constantemente.
    /* */
    NamedCommands.registerCommand("SHOOT", new MoverShooterAutomatico(shooter, intake, -ConstantsShooter.velocidadNeoShooter, ConstantesIntake.velocidadIntakeNeoEscupirParaShooter).withTimeout(0.8));
    NamedCommands.registerCommand("LOWER_INTAKE", new PivotearIntakeAutomatico(intake, shooter, 1).withTimeout(1.2));
    NamedCommands.registerCommand("RISE_INTAKE", new PivotearIntakeAutomatico(intake, shooter, 3));
    NamedCommands.registerCommand("AIMBOT", new AutoAimLimelight(chasis, shooter, intake, null).withTimeout(4));
    // pathChooser = new PathChooser(new ThePaths());
    
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode", autoChooser);


    chasis.setDefaultCommand(
      new Manejo(
        chasis,
        () -> (-driveControl.getRawAxis(1)),
        () -> (driveControl.getRawAxis(0)),
        () -> (driveControl.getRawAxis(2))
      )
    );

    configureBindings();
  }

  private void configureBindings() {
    new JoystickButton(driveControl, Constants.ConstantesIO.botonTriangulo).whileTrue(new RunCommand(chasis::zeroHeading));
    // new JoystickButton(driveControl, ConstantesIO.bumperIzquierdo).toggleOnTrue(new RunCommand(Manejo::toggleDriveMode).withTimeout(0.2));

    //new JoystickButton(mecanismsControl, Constants.ConstantesIO.botonCuadrado).whileTrue(new pivoterIntake(intake, Constants.ConstantesIntake.velocidadIntakePivotNeo));
    //new JoystickButton(mecanismsControl, Constants.ConstantesIO.botonCirculo).whileTrue(new pivoterIntake(intake, -Constants.ConstantesIntake.velocidadIntakePivotNeo));
    new JoystickButton(mecanismsControl, Constants.ConstantesIO.gatilloDerecho).whileTrue(new intakeNoteManual(intake, ConstantesIntake.velocidadIntakeNeoEscupir));
    new JoystickButton(mecanismsControl, Constants.ConstantesIO.gatilloIzquierdo).whileTrue(new intakeNoteManual(intake, ConstantesIntake.velocidadIntakeNeoChupar));
    new POVButton(mecanismsControl, Constants.ConstantesIO.flechaAbajo).whileTrue(new moverClimberManual(climber, ConstantesClimber.subir));
    new POVButton(mecanismsControl, Constants.ConstantesIO.flechaArriba).whileTrue(new moverClimberManual(climber, ConstantesClimber.bajar));
    new JoystickButton(mecanismsControl, Constants.ConstantesIO.botonCruz).whileTrue(new moverShooterManual(shooter, Constants.ConstantsShooter.velocidadNeoShooter));

    
    //new JoystickButton(mecanismsControl, Constants.ConstantesIO.botonOptions).whileTrue(new RunCommand(intake::sumarUnoValorBotonOptions));9
    //if (Intake.valorBotonOptions % 2 == 0){
    new JoystickButton(mecanismsControl, Constants.ConstantesIO.botonTriangulo).toggleOnTrue(new MoverShooterAutomatico(shooter, intake, -ConstantsShooter.velocidadNeoShooter, ConstantesIntake.velocidadIntakeNeoEscupirParaShooter));      
    new JoystickButton(driveControl, Constants.ConstantesIO.gatilloDerecho).whileTrue(new PivotearIntakeAutomatico(intake, shooter, 1)); //Floor
    new JoystickButton(mecanismsControl, Constants.ConstantesIO.bumperDerecho).toggleOnTrue(new PivotearIntakeAutomatico(intake, shooter, 2)); //Amp
    new JoystickButton(driveControl, Constants.ConstantesIO.gatilloDerecho).whileFalse(new PivotearIntakeAutomatico(intake, shooter, 3)); //Shooter
    

    new POVButton(mecanismsControl, Constants.ConstantesIO.flechaIzquierda).whileTrue(new PivotearIntakeAutomatico(intake, shooter, 1)); //Floor
    new JoystickButton(mecanismsControl, Constants.ConstantesIO.bumperIzquierdo).toggleOnTrue(new PivotearIntakeAutomatico(intake, shooter, 3)); //Shooter
    new POVButton(mecanismsControl, Constants.ConstantesIO.flechaIzquierda).whileFalse(new PivotearIntakeAutomatico(intake, shooter, 3)); //Floor
    // new JoystickButton(mecanismsControl, Constants.ConstantesIO.bumperDerecho).whileFalse(new PivotearIntakeAutomatico(intake, shooter, 3)); //Shooter

    new JoystickButton(driveControl, Constants.ConstantesIO.botonCruz).whileTrue(new AutoAimLimelight(chasis, shooter, intake, () -> (-driveControl.getRawAxis(0))));

    new JoystickButton(mecanismsControl, ConstantesIO.botonCuadrado).toggleOnTrue(new AutoAMPScore(shooter, intake));

    new JoystickButton(driveControl, ConstantesIO.bumperIzquierdo).toggleOnTrue(new RunCommand(chasis::changeDriveMode));
    /*}

    else {
    new JoystickButton(mecanismsControl, Constants.ConstantesIO.flechaDerecha).whileTrue(new pivotearIntakeManual(intake, Constants.ConstantesIntake.velocidadIntakePivotNeo));
    new JoystickButton(mecanismsControl, Constants.ConstantesIO.flechaIzquierda).whileTrue(new pivotearIntakeManual(intake, -Constants.ConstantesIntake.velocidadIntakePivotNeo));
    }
    

    new JoystickButton(driveControl, Constants.ConstantesIO.gatilloDerecho);*/    
  }


  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public Chasis getChasisSubsystem() {
    return chasis;
  }

  public Intake getIntakeSubsystem() {
    return intake;
  }

  public Leds getLedsSubsystem() {
    return leds;
  }
}

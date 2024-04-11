// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.utils.DriverReadouts;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  @SuppressWarnings("unused")
  private final DriverReadouts driverReadout = new DriverReadouts(m_robotContainer);

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    AddressableLED m_led = new AddressableLED(0);
    AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(50);
    m_led.setLength(m_ledBuffer.getLength());
    m_led.start();

    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      int hue = (0 + (i * 180 / m_ledBuffer.getLength())) % 180;
      m_ledBuffer.setHSV(i, hue, 255, 128);
    }
    m_led.setData(m_ledBuffer);

    // UsbCamera camera = CameraServer.startAutomaticCapture();
    // camera.setResolution(18, 14);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    m_robotContainer.getIntakeSubsystem().pivotMotorCoast();
    m_robotContainer.getChasisSubsystem().setChassisToBreak();
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);

    //m_robotContainer.getChasisSubsystem().setChassisToCoast();

    // UsbCamera cameraIntake = CameraServer.startAutomaticCapture();
    // cameraIntake.setResolution(18, 14);
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {
    m_robotContainer.getIntakeSubsystem().pivotMotorBreak();
    m_robotContainer.getChasisSubsystem().setChassisToBreak();
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);

  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Subsystems.StateMachine.RobotState;
import frc.robot.Subsystems.StateMachine.TargetState;
import frc.robot.Util.Controllers;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  
  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
    for (int port = 5801; port <= 5809; port++) {
      PortForwarder.add(port, "limelight.local", port);
    }

    PortForwarder.add(5800, "photonvision.local", 5800);
  }
  


  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    m_robotContainer.s_StateMachine.setRobotState(RobotState.NONE);
    m_robotContainer.s_StateMachine.setTargetState(TargetState.PREP_NONE);
  }

  @Override
  public void disabledPeriodic()
  {
        m_robotContainer.updateControls();
  }

  @Override
  public void disabledExit() {}

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
    m_robotContainer.configureDriverBindings(new Controllers(m_robotContainer.driveControllerChooser.getSelected(), m_robotContainer.driveHandChooser.getSelected()));
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

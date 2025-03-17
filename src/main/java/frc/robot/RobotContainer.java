// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;




import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.Auto;
import frc.robot.Commands.DriveCommand;
import frc.robot.Util.Controllers;
import frc.robot.Util.LimelightHelpers;
import frc.robot.Util.Constants.constants_Drive;
import frc.robot.Util.Constants.constants_Sim;
import frc.robot.Subsystems.AlgaeRollers;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Lights;
import frc.robot.Subsystems.StateMachine;
import frc.robot.Subsystems.StateMachine.RobotState;
import frc.robot.Subsystems.StateMachine.TargetState;
import frc.robot.Subsystems.Vision;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Drive.GyroIO;
import frc.robot.Subsystems.Drive.GyroIONavX;
import frc.robot.Subsystems.Drive.ModuleIO;
import frc.robot.Subsystems.Drive.ModuleIODrive;
import frc.robot.Subsystems.Drive.ModuleIOSim;


/**
 * This class is where the bulk
 *  of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  
  public Controllers u_Controllers;
  public Drive s_Drive;
  public LimelightHelpers h_Limelight;
  public Vision s_Vision;
  public DriveCommand c_Drive;
  public StateMachine s_StateMachine;
  public Climber s_Climber;
  public Elevator s_Elevator;
  public AlgaeRollers s_Rollers;
  public Lights s_Lights;
  public Auto c_Auto;
  
  public Trigger gamePieceStoredTrigger_Coral = new Trigger(() -> s_Elevator.getGamePieceStored());
  public Trigger gamePieceCollectedTrigger_Algae = new Trigger(() -> s_Rollers.getGamePieceCollected());

  private final LoggedDashboardChooser<Command> autoChooser;


  public RobotContainer() 
  {

    switch (constants_Sim.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        s_Drive =
            new Drive(
                new GyroIONavX(),
                new ModuleIODrive(0),
                new ModuleIODrive(1),
                new ModuleIODrive(2),
                new ModuleIODrive(3));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        s_Drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        s_Drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
    }
   
    
    configureFiles();
    configureDriverBindings();
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());


    gamePieceStoredTrigger_Coral.onTrue(Commands.deferredProxy(
      () -> s_StateMachine.tryState(RobotState.CORAL, s_StateMachine, c_Drive,s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights)).andThen(intakeRumble()));

    gamePieceCollectedTrigger_Algae.onTrue(Commands.deferredProxy(
      ()-> s_StateMachine.tryState(RobotState.ALGAE, s_StateMachine, c_Drive, s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights)).andThen(intakeRumble()));

    gamePieceStoredTrigger_Coral.onFalse(Commands.deferredProxy(
      ()-> s_StateMachine.tryState(RobotState.NONE, s_StateMachine, c_Drive, s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights)).andThen(intakeRumble()));

    gamePieceCollectedTrigger_Algae.onFalse(Commands.deferredProxy(
      ()-> s_StateMachine.tryState(RobotState.NONE, s_StateMachine, c_Drive, s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights)).andThen(intakeRumble()));

  }

  public SequentialCommandGroup intakeRumble()
  {
    return new SequentialCommandGroup(Commands.runOnce(
    ()->{
      u_Controllers.leftStick.setRumble(RumbleType.kBothRumble, 0.5);
      u_Controllers.rightStick.setRumble(RumbleType.kBothRumble, 0.5);
    }).andThen(
    Commands.waitSeconds(2.5)).andThen(
    Commands.runOnce(()->{
      u_Controllers.leftStick.setRumble(RumbleType.kBothRumble, 0);
      u_Controllers.rightStick.setRumble(RumbleType.kBothRumble, 0);
    })));
  }

  public void configureFiles()
  {
      u_Controllers = new Controllers();
      h_Limelight = new LimelightHelpers();
      // c_Drive = new DriveCommand(s_Drive, u_Controllers.leftStick, u_Controllers.rightStick);
      s_Vision = new Vision(c_Drive, s_Drive, u_Controllers);
      s_StateMachine = new StateMachine();
      s_Climber = new Climber();
      s_Elevator = new Elevator();
      s_Rollers = new AlgaeRollers();
      s_Lights = new Lights();
      c_Auto = new Auto(s_StateMachine, c_Drive, s_Drive, s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights);
  }


  public final void configureDriverBindings() {

      // Default command, normal field-relative drive
    s_Drive.setDefaultCommand(
        DriveCommand.joystickDrive(
            s_Drive,
            () -> -u_Controllers.rightStick.getY(),
            () -> -u_Controllers.rightStick.getX(),
            () -> -u_Controllers.leftStick.getX()));
  
    // Intake Algae
    u_Controllers.leftStick.trigger().whileTrue(Commands.deferredProxy(()->
    s_StateMachine.tryState(RobotState.INTAKE_ALGAE, s_StateMachine, c_Drive, s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights)))
    .onFalse(Commands.deferredProxy(()->
    s_StateMachine.tryState(RobotState.NONE, s_StateMachine, c_Drive, s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights)));

    // Intake Algae Vision
    // u_Controllers.leftStick.trigger().and(u_Controllers.leftStick.button(3)).onTrue(Commands.deferredProxy(()->
    // s_StateMachine.tryState(RobotState.INTAKE_ALGAE, s_StateMachine, c_Drive, s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights)))
    // .onTrue(s_Vision.autoAlgae)
    // .onFalse(Commands.deferredProxy(()->
    // s_StateMachine.tryState(RobotState.NONE, s_StateMachine, c_Drive, s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights)));

    //Align Reef
    // u_Controllers.leftStick.button(4).whileTrue(s_Vision.autoReef);
    
    // Intake Coral
    u_Controllers.leftStick.button(2).whileTrue(Commands.deferredProxy(()->
    s_StateMachine.tryState(RobotState.SOURCE, s_StateMachine, c_Drive, s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights)))
    .onFalse(Commands.deferredProxy(()->
    s_StateMachine.tryState(RobotState.NONE, s_StateMachine, c_Drive, s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights)));
    
    //Auto Reef
    u_Controllers.leftStick.button(4).onTrue(Commands.runOnce(()->s_Vision.isRightScore = true)).onTrue(s_Vision.alignReef);
    u_Controllers.leftStick.button(3).onTrue(Commands.runOnce(()->s_Vision.isRightScore = false)).onTrue(s_Vision.alignReef);
    // //Clean L2
    // u_Controllers.leftStick.button(3).onTrue(Commands.deferredProxy(()->
    // s_StateMachine.tryState(RobotState.CLEAN_L2, s_StateMachine, c_Drive, s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights)))
    // .onFalse(Commands.deferredProxy(()->
    // s_StateMachine.tryState(RobotState.NONE, s_StateMachine, c_Drive, s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights)));
    
    // // //Clean L3
    // u_Controllers.leftStick.button(4).onTrue(Commands.deferredProxy(()->
    // s_StateMachine.tryState(RobotState.CLEAN_L3, s_StateMachine, c_Drive, s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights)))
    // .onFalse(Commands.deferredProxy(()->
    // s_StateMachine.tryState(RobotState.NONE, s_StateMachine, c_Drive, s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights)));
    
    //Shooting
    u_Controllers.rightStick.trigger().whileTrue(Commands.deferredProxy(()->
    s_StateMachine.tryState(RobotState.SCORING, s_StateMachine, c_Drive, s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights)))
    .onFalse(Commands.deferredProxy(()->
    s_StateMachine.tryState(RobotState.NONE, s_StateMachine, c_Drive, s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights)));
    
    
    //Drive Controls
    // u_Controllers.rightStick.button(2).toggleOnTrue(Commands.runOnce(() -> s_Drive.zeroHeading()));
    // u_Controllers.rightStick.button(3).toggleOnTrue(s_Drive.fieldOrientedToggle());
    u_Controllers.rightStick.button(4).onTrue(Commands.runOnce(()->s_Drive.faceAllFoward())); //window looking button
    
    
    //PREPS For Opperator controller
    
    //PREP_L1
    u_Controllers.PREP_L1.onTrue(Commands.runOnce(() ->
    s_StateMachine.setTargetState(TargetState.PREP_L1)))
    .onTrue(Commands.deferredProxy(()->
    s_StateMachine.tryState(RobotState.PREP_L1, s_StateMachine, c_Drive, s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights)));

    //PREP_L2
    u_Controllers.PREP_L2.onTrue(Commands.runOnce(() ->
    s_StateMachine.setTargetState(TargetState.PREP_L2)))
    .onTrue(Commands.deferredProxy(()->
    s_StateMachine.tryState(RobotState.PREP_L2, s_StateMachine, c_Drive, s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights)));

    //PREP_L3
    u_Controllers.PREP_L3.onTrue(Commands.runOnce(() ->
    s_StateMachine.setTargetState(TargetState.PREP_L3)))
    .onTrue(Commands.deferredProxy(()->
    s_StateMachine.tryState(RobotState.PREP_L3, s_StateMachine, c_Drive, s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights)));
    
    //PREP_ALgae
    u_Controllers.PREP_ALGAE.onTrue(Commands.runOnce(() ->
    s_StateMachine.setTargetState(TargetState.PREP_ALGAE)))
    .onTrue(Commands.deferredProxy(()->
    s_StateMachine.tryState(RobotState.PREP_ALGAE, s_StateMachine, c_Drive, s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights)));

    //PREP_None
    u_Controllers.PREP_NONE.onTrue(Commands.runOnce(() ->
    s_StateMachine.setTargetState(TargetState.PREP_NONE)))
    .onTrue(Commands.deferredProxy(()->
    s_StateMachine.tryState(RobotState.PREP_NONE, s_StateMachine, c_Drive, s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights)));

    u_Controllers.ELE_UP.onTrue(s_Elevator.moveElevatorUp());
    u_Controllers.ELE_UP.whileFalse(s_Elevator.stopElevator());

    u_Controllers.ELE_DOWN.onTrue(s_Elevator.moveElevatorDown());
    u_Controllers.ELE_DOWN.whileFalse(s_Elevator.stopElevator());

    u_Controllers.CLIMB_DOWN.whileTrue(s_Climber.climberDown());
    u_Controllers.CLIMB_DOWN.whileFalse(s_Climber.stopClimber());
    
    u_Controllers.CLIMB_UP.whileTrue(s_Climber.climberUp());
    u_Controllers.CLIMB_UP.whileFalse(s_Climber.stopClimber());


    u_Controllers.CLIMB_AUTO.onTrue(Commands.runOnce(()->s_Rollers.intakeClimb()));
    u_Controllers.CLIMB_RESET.onTrue(Commands.runOnce(()->s_Climber.zeroPosition()));

    u_Controllers.ALGAE_OVERRIDE.onTrue(Commands.runOnce(()->{s_Rollers.ALGAE_OVERRIDE = !s_Rollers.ALGAE_OVERRIDE;}));
    u_Controllers.CORAL_OVERRIDE.onTrue(Commands.runOnce(()->{s_Elevator.CORAL_OVERRIDE = !s_Elevator.CORAL_OVERRIDE;}));

  }


 /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}

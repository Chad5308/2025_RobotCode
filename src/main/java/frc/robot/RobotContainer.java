// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;




import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.Auto;
import frc.robot.Commands.Drive;
import frc.robot.Util.Controllers;
import frc.robot.Util.LimelightHelpers;
import frc.robot.Subsystems.AlgaeRollers;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Lights;
import frc.robot.Subsystems.StateMachine;
import frc.robot.Subsystems.StateMachine.RobotState;
import frc.robot.Subsystems.StateMachine.TargetState;
import frc.robot.Subsystems.Vision;
import frc.robot.Subsystems.Drive.Swerve;


/**
 * This class is where the bulk
 *  of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  public SendableChooser<String> autoChooser1, autoChooser2, autoChooser3;
  public String selection1, selection2, selection3, finalSelection;

  
  public Controllers u_Controllers;
  public Swerve s_Swerve;
  public LimelightHelpers h_Limelight;
  public Vision s_Vision;
  public Drive c_Drive;
  public StateMachine s_StateMachine;
  public Climber s_Climber;
  public Elevator s_Elevator;
  public AlgaeRollers s_Rollers;
  public Lights s_Lights;
  public Auto c_Auto;
  
  public Trigger gamePieceStoredTrigger_Coral = new Trigger(() -> s_Elevator.getGamePieceStored());
  public Trigger gamePieceCollectedTrigger_Algae = new Trigger(() -> s_Rollers.getGamePieceCollected());

  public RobotContainer() 
  {
    autoChooser1 = new SendableChooser<>();
    autoChooser2 = new SendableChooser<>();
    autoChooser3 = new SendableChooser<>();
    SmartDashboard.putData(autoChooser1);
    SmartDashboard.putData(autoChooser2);
    SmartDashboard.putData(autoChooser3);

    configureAutoChoosers();
    configureFiles();
    s_Swerve.setDefaultCommand(c_Drive);
    configureDriverBindings();


    gamePieceStoredTrigger_Coral.onTrue(Commands.deferredProxy(
      () -> s_StateMachine.tryState(RobotState.CORAL, s_StateMachine, c_Drive,s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights)).andThen(intakeRumble()));

    gamePieceCollectedTrigger_Algae.onTrue(Commands.deferredProxy(
      ()-> s_StateMachine.tryState(RobotState.ALGAE, s_StateMachine, c_Drive, s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights)).andThen(intakeRumble()));

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
      s_Swerve = new Swerve();
      h_Limelight = new LimelightHelpers();
      s_Vision = new Vision(s_Swerve);
      c_Drive = new Drive(s_Swerve, u_Controllers.leftStick, u_Controllers.rightStick);
      s_StateMachine = new StateMachine();
      s_Climber = new Climber();
      s_Elevator = new Elevator();
      s_Rollers = new AlgaeRollers();
      s_Lights = new Lights();
      c_Auto = new Auto(s_StateMachine, c_Drive, s_Swerve, s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights);
  }


  public final void configureDriverBindings() {

  
    // Intake Algae
    u_Controllers.leftStick.trigger().whileTrue(Commands.deferredProxy(()->
    s_StateMachine.tryState(RobotState.INTAKE_ALGAE, s_StateMachine, c_Drive, s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights)))
    .onFalse(Commands.deferredProxy(()->
    s_StateMachine.tryState(RobotState.NONE, s_StateMachine, c_Drive, s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights)));
    
    // Intake Coral
    u_Controllers.leftStick.button(2).whileTrue(Commands.deferredProxy(()->
    s_StateMachine.tryState(RobotState.SOURCE, s_StateMachine, c_Drive, s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights)))
    .onFalse(Commands.deferredProxy(()->
    s_StateMachine.tryState(RobotState.NONE, s_StateMachine, c_Drive, s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights)));
    
    // //Clean L2
    // u_Controllers.leftStick.button(3).onTrue(Commands.deferredProxy(()->
    // s_StateMachine.tryState(RobotState.CLEAN_L2, s_StateMachine, c_Drive, s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights)))
    // .onFalse(Commands.deferredProxy(()->
    // s_StateMachine.tryState(RobotState.NONE, s_StateMachine, c_Drive, s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights)));
    
    // //Clean L3
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
    u_Controllers.rightStick.button(2).toggleOnTrue(Commands.runOnce(() -> s_Swerve.zeroHeading()));
    u_Controllers.rightStick.button(3).toggleOnTrue(s_Swerve.fieldOrientedToggle());
    u_Controllers.rightStick.button(4).onTrue(s_Swerve.resetWheels()); //window looking button
    
    
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

    u_Controllers.ELE_UP.whileTrue(s_Elevator.moveElevatorUp());
    u_Controllers.ELE_UP.whileFalse(s_Elevator.stopElevator());

    u_Controllers.ELE_DOWN.whileTrue(s_Elevator.moveElevatorDown());
    u_Controllers.ELE_DOWN.whileFalse(s_Elevator.stopElevator());

    u_Controllers.CLIMB_DOWN.whileTrue(s_Climber.climberDown());
    u_Controllers.CLIMB_DOWN.whileFalse(s_Climber.stopClimber());
    
    u_Controllers.CLIMB_UP.whileTrue(s_Climber.climberUp());
    u_Controllers.CLIMB_UP.whileFalse(s_Climber.climberUp());

    u_Controllers.ALGAE_OVERRIDE.onChange(Commands.runOnce(()->{s_Rollers.ALGAE_OVERRIDE = !s_Rollers.ALGAE_OVERRIDE;}));
    u_Controllers.CORAL_OVERRIDE.onChange(Commands.runOnce(()->{s_Elevator.CORAL_OVERRIDE = !s_Elevator.CORAL_OVERRIDE;}));
  }


  
  public void configureAutoChoosers()
  {
    //1 Paths
    autoChooser1.addOption("1A", selection1 = "1A");
    autoChooser1.addOption("1J", selection1 = "1J");
    autoChooser1.addOption("1K", selection1 = "1K");
    autoChooser1.addOption("1L", selection1 = "1L");
    //2 Paths
    autoChooser1.addOption("2H", selection1 = "2H");
    autoChooser1.addOption("2I", selection1 = "2I");
    autoChooser1.addOption("2J", selection1 = "2J");
    autoChooser1.addOption("2K", selection1 = "2K");
    //3 Paths
    autoChooser1.addOption("3E", selection1 = "3E");
    autoChooser1.addOption("3F", selection1 = "3F");
    autoChooser1.addOption("3G", selection1 = "3G");
    autoChooser1.addOption("3H", selection1 = "3H");
    autoChooser1.addOption("3I", selection1 = "3I");
    autoChooser1.addOption("Test", selection1 = "Test");
  
    //X Paths
    autoChooser2.addOption("AX", selection2 = "AX");
    autoChooser2.addOption("IX", selection2 = "IX");
    autoChooser2.addOption("JX", selection2 = "JX");
    autoChooser2.addOption("KX", selection2 = "KX");
    autoChooser2.addOption("LX", selection2 = "LX");
    //Y Paths
    autoChooser2.addOption("AY", selection2 = "AY");
    autoChooser2.addOption("EY", selection2 = "EY");
    autoChooser2.addOption("FY", selection2 = "FY");
    autoChooser2.addOption("GY", selection2 = "GY");
    autoChooser2.addOption("HY", selection2 = "HY");

    //Return X Paths
    autoChooser3.addOption("XA", selection3 = "XA");
    autoChooser3.addOption("XI", selection3 = "XI");
    autoChooser3.addOption("XJ", selection3 = "XJ");
    autoChooser3.addOption("XK", selection3 = "XK");
    autoChooser3.addOption("XL", selection3 = "XL");
    //Return Y Paths
    autoChooser3.addOption("YA", selection3 = "YA");
    autoChooser3.addOption("YB", selection3 = "YB");
    autoChooser3.addOption("YC", selection3 = "YC");
    autoChooser3.addOption("YD", selection3 = "YD");
    autoChooser3.addOption("YE", selection3 = "YE");
    autoChooser3.addOption("YF", selection3 = "YF");

  }



  public Command getAutonomousCommand()
  {
    // return autoChooser.getSelected();

    // Before initializing finalSelection, get the current
    // choice for each autoChooser object.
    selection1 = autoChooser1.getSelected();
    selection2 = autoChooser2.getSelected();
    selection3 = autoChooser3.getSelected();
    System.out.printf("***** Auto selected: %s %s %s\n", 
                      selection1, 
                      selection2, 
                      selection3);

    // We need some logic to handle the case when the path is only
    // one or two legs.  Below is a sample of one way to deal with 
    // that.  Question: What should we do if the selection1 is null?
    finalSelection = selection1;
    if(finalSelection == null)
    {
      System.out.print("***** WARNING: No auto path selected.");
    }
    else
    {
      if(selection2 != null)
      {
        finalSelection += ", " + selection2;
      }

      if(selection3 != null)
      {
        finalSelection += ", " + selection3;
      }
   }

    //finalSelection = selection1 + ", " + selection2 + ", " + selection3;
    SmartDashboard.putString("Auto Selection", finalSelection);
    return new PathPlannerAuto(finalSelection);
  }




}

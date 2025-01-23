// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import javax.lang.model.util.ElementScanner14;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.Auto;
import frc.robot.Commands.Drive;
import frc.robot.Util.Constants.constants_OI;
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

  private final CommandXboxController drive_Controller = new CommandXboxController(constants_OI.DRIVER_CONTROLLER_PORT);
  // private final XboxControllerSim op_Controller = new XboxControllerSim(constants_OI.OP_CONTROLLER_PORT);

  public Swerve s_Swerve = new Swerve();
  public LimelightHelpers h_Limelight = new LimelightHelpers();
  public Vision s_Vision = new Vision(s_Swerve);
  public Drive c_Drive = new Drive(s_Swerve, drive_Controller);
  public StateMachine s_StateMachine = new StateMachine();
  public Climber s_Climber = new Climber();
  public Elevator s_Elevator = new Elevator();
  public AlgaeRollers s_Rollers = new AlgaeRollers();
  public Lights s_Lights = new Lights();
  public Auto c_Auto = new Auto(c_Drive, s_Swerve, s_Vision, s_Elevator, s_StateMachine, s_Lights);

  public Trigger gamePieceStoredTrigger_Coral = new Trigger(() -> s_Elevator.getGamePieceStored());
  public Trigger gamePieceCollectedTrigger_Algae = new Trigger(() -> s_Rollers.getGamePieceCollected());

  // private SendableChooser<Command> autoChooser;

  public SendableChooser<String> autoChooser1, autoChooser2, autoChooser3;
  public String selection1, selection2, selection3, finalSelection;

 
  public RobotContainer() 
  {
    // autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser1 = new SendableChooser<>();
    autoChooser2 = new SendableChooser<>();
    autoChooser3 = new SendableChooser<>();
    s_Swerve.setDefaultCommand(c_Drive);
    configureDriverBindings(drive_Controller);
    configureAutoChoosers();

    // SmartDashboard.putData("Auto Chooser", autoChooser);   
    SmartDashboard.putData(autoChooser1);
    SmartDashboard.putData(autoChooser2);
    SmartDashboard.putData(autoChooser3);


    gamePieceStoredTrigger_Coral.onTrue(Commands.deferredProxy(
      () -> s_StateMachine.tryState(RobotState.CORAL, s_StateMachine, c_Drive,s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights)));


    gamePieceCollectedTrigger_Algae.onTrue(Commands.deferredProxy(
      ()-> s_StateMachine.tryState(RobotState.ALGAE, s_StateMachine, c_Drive, s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights)));

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

    // autoChooser.addOption("AutoDrive", limelightTestAuto());
  }

  // public SequentialCommandGroup limelightTestAuto()
  // {
  //   return new SequentialCommandGroup(new PathPlannerAuto("Start Auto").andThen(s_Vision.autoDrive).andThen(new PathPlannerAuto("Return Auto")));
  // }



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


  public void configureDriverBindings(CommandXboxController drive_Controller) {
    //Drive Controls
    drive_Controller.povRight().toggleOnTrue(Commands.runOnce(() -> s_Swerve.zeroHeading()));
    drive_Controller.povLeft().toggleOnTrue(s_Swerve.fieldOrientedToggle());
    drive_Controller.button(7).onTrue(s_Swerve.resetWheels()); //window looking button

        // Intake
    drive_Controller.leftTrigger()
        .whileTrue(Commands.deferredProxy(
            () -> s_StateMachine.tryState(RobotState.INTAKE_ALGAE, s_StateMachine, c_Drive, s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights)))
        .onFalse(Commands.deferredProxy(
            () -> s_StateMachine.tryState(RobotState.NONE, s_StateMachine, c_Drive, s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights))
            .unless(gamePieceCollectedTrigger_Algae));


    //Shooting
    drive_Controller.rightTrigger().whileTrue(Commands.deferredProxy(
      ()-> s_StateMachine.tryState(RobotState.SCORING, s_StateMachine, c_Drive, s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights)
    )).onFalse(Commands.deferredProxy(
      ()-> s_StateMachine.tryState(RobotState.NONE, s_StateMachine, c_Drive, s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights)
    ));
      
    
    //PREP_L4
    drive_Controller.rightBumper().onTrue(Commands.runOnce(() -> s_StateMachine.setTargetState(TargetState.PREP_L4)))
    .onTrue(Commands.deferredProxy(
      ()-> s_StateMachine.tryState(RobotState.PREP_L4, s_StateMachine, c_Drive, s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights)));


    drive_Controller.a().onTrue(Commands.runOnce(()-> {
      s_Rollers.testBool = !s_Rollers.testBool;
    }));

    drive_Controller.b().onTrue(Commands.runOnce(()-> {
      s_Elevator.testBool = !s_Elevator.testBool;
    }));
  }


  public void debugging(String string)
  {
    System.out.println(string);
  }
}

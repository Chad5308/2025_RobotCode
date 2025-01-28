// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.Auto;
import frc.robot.Commands.Drive;
import frc.robot.Util.DriverProfile.ControllerTypes;
import frc.robot.Util.RobotMap.MAP_CONTROLLER;
import frc.robot.Util.DriverProfile;
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
  public SendableChooser<DriverProfile> driverChooser, opperatorChooser;


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

  public DriverProfile selectedDriver;
  public DriverProfile selectedOpperator;


  // private SendableChooser<Command> autoChooser;


 
  public RobotContainer() 
  {
    // autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser1 = new SendableChooser<>();
    autoChooser2 = new SendableChooser<>();
    autoChooser3 = new SendableChooser<>();
    driverChooser = new SendableChooser<>();
    opperatorChooser = new SendableChooser<>();
    System.out.println("______________FILES DONE______________________");
    configureDriverProfiles();
    System.out.println("______________PROFILES DONE______________________");
    configureAutoChoosers();
    System.out.println("______________AUTOS DONE______________________");
    
    // SmartDashboard.putData("Auto Chooser", autoChooser);   
    SmartDashboard.putData(autoChooser1);
    SmartDashboard.putData(autoChooser2);
    SmartDashboard.putData(autoChooser3);
    SmartDashboard.putData(driverChooser);
    SmartDashboard.putData(opperatorChooser);


    configureFiles();
    s_Swerve.setDefaultCommand(c_Drive);
    
    configureDriverBindings();
    System.out.println("______________BINDINGS DONE______________________");
    System.out.println(selectedDriver.name);
    System.out.println(selectedDriver.FOToggle.toString());
    



    gamePieceStoredTrigger_Coral.onTrue(Commands.deferredProxy(
      () -> s_StateMachine.tryState(RobotState.CORAL, s_StateMachine, c_Drive,s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights)));


    gamePieceCollectedTrigger_Algae.onTrue(Commands.deferredProxy(
      ()-> s_StateMachine.tryState(RobotState.ALGAE, s_StateMachine, c_Drive, s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights)));

  }

  public void configureFiles()
  {
      s_Swerve = new Swerve();
      h_Limelight = new LimelightHelpers();
      s_Vision = new Vision(s_Swerve);
      c_Drive = new Drive(s_Swerve, selectedDriver);
      s_StateMachine = new StateMachine();
      s_Climber = new Climber();
      s_Elevator = new Elevator();
      s_Rollers = new AlgaeRollers();
      s_Lights = new Lights();
      c_Auto = new Auto(c_Drive, s_Swerve, s_Vision, s_Elevator, s_StateMachine, s_Lights);
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


  public void configureDriverBindings() {
    
    //Drive Controls
    selectedDriver.FOToggle.toggleOnTrue(s_Swerve.fieldOrientedToggle());
    selectedDriver.zeroHeading.toggleOnTrue(Commands.runOnce(() -> s_Swerve.zeroHeading()));
    selectedDriver.resetWheels.onTrue(s_Swerve.resetWheels()); //window looking button

    // Intake Algae
    selectedDriver.intakeAlgae.whileTrue(Commands.deferredProxy(()->
      s_StateMachine.tryState(RobotState.INTAKE_ALGAE, s_StateMachine, c_Drive, s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights)))
      .onFalse(Commands.deferredProxy(()->
      s_StateMachine.tryState(RobotState.NONE, s_StateMachine, c_Drive, s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights)));

     // Intake Coral
    selectedDriver.intakeCoral.whileTrue(Commands.deferredProxy(()->
      s_StateMachine.tryState(RobotState.SOURCE, s_StateMachine, c_Drive, s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights)))
      .onFalse(Commands.deferredProxy(()->
      s_StateMachine.tryState(RobotState.NONE, s_StateMachine, c_Drive, s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights)));

       //Clean L2
    selectedDriver.cleanL2.onTrue(Commands.deferredProxy(()->
      s_StateMachine.tryState(RobotState.CLEAN_L2, s_StateMachine, c_Drive, s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights)))
      .onFalse(Commands.deferredProxy(()->
      s_StateMachine.tryState(RobotState.NONE, s_StateMachine, c_Drive, s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights)));

       //Clean L3
    selectedDriver.cleanL3.onTrue(Commands.deferredProxy(()->
      s_StateMachine.tryState(RobotState.CLEAN_L3, s_StateMachine, c_Drive, s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights)))
      .onFalse(Commands.deferredProxy(()->
      s_StateMachine.tryState(RobotState.NONE, s_StateMachine, c_Drive, s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights)));

    //Shooting
    selectedDriver.score.whileTrue(Commands.deferredProxy(()->
      s_StateMachine.tryState(RobotState.SCORING, s_StateMachine, c_Drive, s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights)))
      .onFalse(Commands.deferredProxy(()->
      s_StateMachine.tryState(RobotState.NONE, s_StateMachine, c_Drive, s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights)));
      


    //PREPS For Opperator controller

    //PREP_L1
    selectedOpperator.PREP_L1.onTrue(Commands.runOnce(() ->
    s_StateMachine.setTargetState(TargetState.PREP_L1)))
    .onTrue(Commands.deferredProxy(()->
    s_StateMachine.tryState(RobotState.PREP_L1, s_StateMachine, c_Drive, s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights)));

    //PREP_L2
    selectedOpperator.PREP_L2.onTrue(Commands.runOnce(() ->
    s_StateMachine.setTargetState(TargetState.PREP_L2)))
    .onTrue(Commands.deferredProxy(()->
    s_StateMachine.tryState(RobotState.PREP_L2, s_StateMachine, c_Drive, s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights)));

    //PREP_L3
    selectedOpperator.PREP_L3.onTrue(Commands.runOnce(() ->
    s_StateMachine.setTargetState(TargetState.PREP_L3)))
    .onTrue(Commands.deferredProxy(()->
    s_StateMachine.tryState(RobotState.PREP_L3, s_StateMachine, c_Drive, s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights)));
    
    //PREP_L4
    selectedOpperator.PREP_L4.onTrue(Commands.runOnce(() ->
    s_StateMachine.setTargetState(TargetState.PREP_L4)))
    .onTrue(Commands.deferredProxy(()->
    s_StateMachine.tryState(RobotState.PREP_L4, s_StateMachine, c_Drive, s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights)));

    //PREP_ALgae
    selectedOpperator.PREP_ALGAE.onTrue(Commands.runOnce(() ->
    s_StateMachine.setTargetState(TargetState.PREP_ALGAE)))
    .onTrue(Commands.deferredProxy(()->
    s_StateMachine.tryState(RobotState.PREP_ALGAE, s_StateMachine, c_Drive, s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights)));

    //PREP_None
    selectedOpperator.PREP_NONE.onTrue(Commands.runOnce(() ->
    s_StateMachine.setTargetState(TargetState.PREP_NONE)))
    .onTrue(Commands.deferredProxy(()->
    s_StateMachine.tryState(RobotState.PREP_NONE, s_StateMachine, c_Drive, s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights)));

    // drive_Controller.a().onTrue(Commands.runOnce(()-> {
    //   s_Rollers.testBool = !s_Rollers.testBool;
    // }));

    // drive_Controller.b().onTrue(Commands.runOnce(()-> {
    //   s_Elevator.testBool = !s_Elevator.testBool;
    // }));
  }


  public void configureDriverProfiles()
  {
    CommandXboxController xboxMain = new CommandXboxController(MAP_CONTROLLER.MAIN_CONTROLLER_PORT);
    CommandXboxController xboxOpp = new CommandXboxController(MAP_CONTROLLER.OPP_CONTROLLER_PORT);

    CommandPS4Controller ps4Main = new CommandPS4Controller(MAP_CONTROLLER.MAIN_CONTROLLER_PORT);
    CommandPS4Controller ps4Opp = new CommandPS4Controller(MAP_CONTROLLER.OPP_CONTROLLER_PORT);

    CommandPS5Controller ps5Main = new CommandPS5Controller(MAP_CONTROLLER.MAIN_CONTROLLER_PORT);
    CommandPS5Controller ps5Opp = new CommandPS5Controller(MAP_CONTROLLER.OPP_CONTROLLER_PORT);
    
    CommandJoystick leftJoystick = new CommandJoystick(MAP_CONTROLLER.LEFT_JOYSTICK);
    CommandJoystick rightJoystick = new CommandJoystick(MAP_CONTROLLER.RIGHT_JOYSTICK);
    
    DriverProfile Martin;
    DriverProfile Morgan;
    DriverProfile Test;
    //Martins Bindings
    driverChooser.addOption("Martin", Martin = new DriverProfile(ControllerTypes.JoySticks, ControllerTypes.Xbox, true, "Martin", 
      leftJoystick.trigger(), //Intake Algae
      leftJoystick.povUp(), //Intake Coral
      rightJoystick.povLeft(), //Clean L2
      rightJoystick.povRight(), //Clean L3
      rightJoystick.trigger(), //Score
      leftJoystick.povLeft(), //ZeroHeading
      leftJoystick.povDown(), //FOToggle
      leftJoystick.povRight(), //Reset Wheels
      rightJoystick.getX(), //Holox
      rightJoystick.getY(), //HoloY
      leftJoystick.getX(), //Rotation
      xboxOpp.leftTrigger(), //PREP_L1
      xboxOpp.leftBumper(), //PREP_L2
      xboxOpp.rightTrigger(), //PREP_L3
      xboxOpp.rightBumper(), //PREP_L4
      xboxOpp.a(), //PREP_ALGAE
      xboxOpp.b())); //PREP_NONE
    opperatorChooser.addOption("Martin", Martin);
 
    //Morgans Bindings
    driverChooser.addOption("Morgan", Morgan = new DriverProfile(ControllerTypes.JoySticks, ControllerTypes.Xbox, true, "Morgan",
      leftJoystick.trigger(), //Intake Algae
      leftJoystick.povUp(), //Intake Coral
      rightJoystick.povLeft(), //Clean L2
      rightJoystick.povRight(), //Clean L3
      rightJoystick.trigger(), //Score
      leftJoystick.povLeft(), //ZeroHeading
      leftJoystick.povDown(), //FOToggle
      leftJoystick.povRight(), //Reset Wheels
      rightJoystick.getX(), //Holox
      rightJoystick.getY(), //HoloY
      leftJoystick.getX(), //Rotation
      xboxOpp.leftTrigger(), //PREP_L1
      xboxOpp.leftBumper(), //PREP_L2
      xboxOpp.rightTrigger(), //PREP_L3
      xboxOpp.rightBumper(), //PREP_L4
      xboxOpp.a(), //PREP_ALGAE
      xboxOpp.b())); //PREP_NONE
    opperatorChooser.addOption("Morgan", Morgan);
      
    //Test Bindings
    driverChooser.setDefaultOption("Test", Test = new DriverProfile(ControllerTypes.Xbox, ControllerTypes.Xbox, true, "Test", 
      xboxMain.leftTrigger(), //Intake Algae
      xboxMain.rightTrigger(), //Intake Coral
      xboxMain.leftBumper(), //Clean L2
      xboxMain.rightBumper(), //Clean L3
      xboxMain.a(), //Score
      xboxMain.povRight(), //ZeroHeading
      xboxMain.povLeft(), //FOToggle
      xboxMain.povDown(), //Reset Wheels
      xboxMain.getLeftX(), //Holox
      xboxMain.getLeftY(), //HoloY
      xboxMain.getRightX(), //Rotation
      xboxOpp.leftTrigger(), //PREP_L1
      xboxOpp.leftBumper(), //PREP_L2
      xboxOpp.rightTrigger(), //PREP_L3
      xboxOpp.rightBumper(), //PREP_L4
      xboxOpp.a(), //PREP_ALGAE
      xboxOpp.b())); //PREP_NONE
    opperatorChooser.setDefaultOption("Test", Test);
    System.out.println("______________WHYYYYYYYYY______________________");
    
    selectedDriver = driverChooser.getSelected();
    selectedOpperator = opperatorChooser.getSelected();
  }




}

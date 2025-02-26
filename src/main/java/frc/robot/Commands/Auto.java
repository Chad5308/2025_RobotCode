package frc.robot.Commands;




import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Util.Constants.constants_Auto;
import frc.robot.Util.Constants.constants_Drive;
import frc.robot.Util.Constants.constants_Module;
import frc.robot.Subsystems.AlgaeRollers;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Lights;
import frc.robot.Subsystems.StateMachine;
import frc.robot.Subsystems.Vision;
import frc.robot.Subsystems.Drive.Swerve;
import frc.robot.Subsystems.StateMachine.RobotState;
import frc.robot.Subsystems.StateMachine.TargetState;

public class Auto {
    
public Drive c_Drive;
public Swerve s_Swerve;
public Vision s_Vision;
public PIDController translationConstants = new PIDController(constants_Auto.P_TRANSLATION, constants_Auto.I_TRANSLATION, constants_Auto.D_TRANSLATION);
public PIDController rotationConstants = new PIDController(constants_Auto.P_THETA, constants_Auto.I_THETA, constants_Auto.D_THETA);

    public Auto(StateMachine s_StateMachine, Drive c_Drive, Swerve s_Swerve, Elevator s_Elevator, Climber s_Climber, AlgaeRollers s_Rollers, Vision s_Vision, Lights s_Lights)
    {
        this.c_Drive = c_Drive;
        this.s_Swerve = s_Swerve;
        this.s_Vision = s_Vision;


        AutoBuilder.configure(
            s_Swerve::getPose,
            s_Swerve::resetOdometry,
            s_Swerve::getRobotRelativeSpeeds,
            s_Swerve::setModuleStates,
            pathController,
            robotConfig,
            s_Swerve::allianceCheck,
            s_Swerve);

            
        NamedCommands.registerCommand("FaceForward Wheels", Commands.runOnce(() -> s_Swerve.faceAllFoward()));
        // NamedCommands.registerCommand("AutoDrive", Commands.runOnce(() -> s_Vision.autoDrive.schedule()));

        NamedCommands.registerCommand("PREP_ALGAE", Commands.runOnce(()->{s_StateMachine.setTargetState(TargetState.PREP_ALGAE);}).andThen
        (Commands.deferredProxy(()->s_StateMachine.tryState(RobotState.PREP_ALGAE, s_StateMachine, c_Drive, s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights))));

        NamedCommands.registerCommand("SCORING", Commands.runOnce(()->{s_StateMachine.setTargetState(TargetState.SCORING);}).andThen
        (Commands.deferredProxy(()->s_StateMachine.tryState(RobotState.SCORING, s_StateMachine, c_Drive, s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights))).andThen((Commands.deferredProxy(()->s_StateMachine.tryState(RobotState.NONE, s_StateMachine, c_Drive, s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights)))));

        NamedCommands.registerCommand("PREP_NONE", Commands.runOnce(()->{s_StateMachine.setTargetState(TargetState.PREP_NONE);}).andThen
        (Commands.deferredProxy(()->s_StateMachine.tryState(RobotState.PREP_NONE, s_StateMachine, c_Drive, s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights))));
        

        new EventTrigger("PREP_L3").onTrue(Commands.runOnce(()->{s_StateMachine.setTargetState(TargetState.PREP_L3);}).andThen
        (Commands.deferredProxy(()->s_StateMachine.tryState(RobotState.PREP_L3, s_StateMachine, c_Drive, s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights))));

        new EventTrigger("PREP_L2").onTrue(Commands.runOnce(()->{s_StateMachine.setTargetState(TargetState.SOURCE);}).andThen
        (Commands.deferredProxy(()->s_StateMachine.tryState(RobotState.PREP_L2, s_StateMachine, c_Drive, s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights))));

        new EventTrigger("SOURCE").onTrue( Commands.runOnce(()->{s_StateMachine.setTargetState(TargetState.SOURCE);}).andThen
        (Commands.deferredProxy(()->s_StateMachine.tryState(RobotState.SOURCE, s_StateMachine, c_Drive, s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights))));
    }

    public PathFollowingController pathController = new PPHolonomicDriveController(
        new com.pathplanner.lib.config.PIDConstants(translationConstants.getP(), translationConstants.getI(), translationConstants.getD()),
        new com.pathplanner.lib.config.PIDConstants(rotationConstants.getP(), rotationConstants.getI(), rotationConstants.getD()));

    public ModuleConfig moduleConfig = new ModuleConfig(constants_Module.WHEEL_RADIUS_METERS, constants_Drive.MAX_SPEED_METERS_PER_SEC, constants_Drive.COF, DCMotor.getKrakenX60(1).withReduction(constants_Module.DRIVE_GEAR_RATIO), 21, 1);
    public RobotConfig robotConfig = new RobotConfig(63.5029, 6.883, moduleConfig, constants_Drive.TRACK_WIDTH);


}

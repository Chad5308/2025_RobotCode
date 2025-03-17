package frc.robot.Commands;




import org.littletonrobotics.junction.Logger;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Util.Constants.constants_Auto;
import frc.robot.Util.LocalADStarAK;
import frc.robot.Subsystems.AlgaeRollers;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Lights;
import frc.robot.Subsystems.StateMachine;
import frc.robot.Subsystems.Vision;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.StateMachine.RobotState;
import frc.robot.Subsystems.StateMachine.TargetState;

public class Auto {
    
public DriveCommand c_Drive;
public Drive s_Drive;
public Vision s_Vision;
public RobotConfig config;
public PIDController translationConstants = new PIDController(constants_Auto.P_TRANSLATION, constants_Auto.I_TRANSLATION, constants_Auto.D_TRANSLATION);
public PIDController rotationConstants = new PIDController(constants_Auto.P_THETA, constants_Auto.I_THETA, constants_Auto.D_THETA);

    public Auto(StateMachine s_StateMachine, DriveCommand c_Drive, Drive s_Drive, Elevator s_Elevator, Climber s_Climber, AlgaeRollers s_Rollers, Vision s_Vision, Lights s_Lights)
    {
        this.c_Drive = c_Drive;
        this.s_Drive = s_Drive;
        this.s_Vision = s_Vision;

        try{
        config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
        // Handle exception as needed
            e.printStackTrace();
        }


        // AutoBuilder.configure(
        //     s_Swerve::getPose,
        //     s_Swerve::resetOdometry,
        //     s_Swerve::getRobotRelativeSpeeds,
        //     s_Swerve::setModuleStates,
        //     pathController,
        //     config,
        //     s_Swerve::allianceCheck,
        //     s_Swerve);

        // Configure AutoBuilder for PathPlanner
    AutoBuilder.configure(
        s_Drive::getPose,
        s_Drive::setPose,
        s_Drive::getChassisSpeeds,
        s_Drive::runVelocity,
        pathController,
        config,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        s_Drive);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

            
        NamedCommands.registerCommand("FaceForward Wheels", Commands.runOnce(() -> s_Drive.faceAllFoward()));
        NamedCommands.registerCommand("FaceRight", Commands.runOnce(()-> s_Drive.faceAllRight()));
        NamedCommands.registerCommand("SCORING", Commands.deferredProxy(()->s_StateMachine.tryState(RobotState.SCORING, s_StateMachine, c_Drive, s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights)));
        NamedCommands.registerCommand("SOURCE", Commands.deferredProxy(()->s_StateMachine.tryState(RobotState.SOURCE, s_StateMachine, c_Drive, s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights)));

        new EventTrigger("PREP_L3").onTrue(Commands.runOnce(()->{s_StateMachine.setTargetState(TargetState.PREP_L3);}).andThen
        (Commands.deferredProxy(()->s_StateMachine.tryState(RobotState.PREP_L3, s_StateMachine, c_Drive, s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights))));

        new EventTrigger("PREP_L2").onTrue(Commands.runOnce(()->{s_StateMachine.setTargetState(TargetState.PREP_L2);}).andThen
        (Commands.deferredProxy(()->s_StateMachine.tryState(RobotState.PREP_L2, s_StateMachine, c_Drive, s_Elevator, s_Climber, s_Rollers, s_Vision, s_Lights))));

    }

    public PathFollowingController pathController = new PPHolonomicDriveController(
        new com.pathplanner.lib.config.PIDConstants(translationConstants.getP(), translationConstants.getI(), translationConstants.getD()),
        new com.pathplanner.lib.config.PIDConstants(rotationConstants.getP(), rotationConstants.getI(), rotationConstants.getD()));



}

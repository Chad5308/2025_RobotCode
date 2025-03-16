package frc.robot.Subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Util.LimelightHelpers;
import frc.robot.Commands.Drive;
import frc.robot.Subsystems.Drive.Swerve;
import frc.robot.Util.Constants;
import frc.robot.Util.Constants.constants_Auto;
import frc.robot.Util.Constants.constants_Limelight;
import frc.robot.Util.Controllers;
import frc.robot.Util.LimelightHelpers.LimelightResults;

import java.util.function.*;

import com.ctre.phoenix6.signals.System_StateValue;

public class Vision extends SubsystemBase{
    
public Drive c_Drive;
public Swerve s_Swerve;
public Controllers u_Controllers;
public NetworkTable networkTables;
public IntegerSubscriber pipeline;
public IntegerPublisher pipelinePublisher;
public double[] localizedPose;
public double[] botPose_targetSpace, targetPose_robotSpace;
public BooleanSupplier interrupted;
public String limelight_Coral = "limelight-coral";
public String limelight_Algae = "limelight-algae";

public LimelightResults r_limelight;

public LimelightHelpers.PoseEstimate mt2;

public boolean isRightScore = false;


// public Apri fieldLayout;

// public SlewRateLimiter tLimiter, xLimiter, zLimiter;

    public Vision(Drive c_Drive, Swerve s_Swerve, Controllers u_Controllers){
        this.c_Drive = c_Drive;
        this.s_Swerve = s_Swerve;
        this.u_Controllers = u_Controllers;
        networkTables = NetworkTableInstance.getDefault().getTable("limelight");
        pipelinePublisher = networkTables.getIntegerTopic("limelight.getpipeline").publish();
        

        // LimelightHelpers.setCameraPose_RobotSpace(limelight_Algae, constants_Limelight.getCameraList(limelight_Algae).get(1), constants_Limelight.getCameraList(limelight_Algae).get(2), constants_Limelight.getCameraList(limelight_Algae).get(3), 0, constants_Limelight.getCameraList(limelight_Algae).get(0), 0);
        // LimelightHelpers.setCameraPose_RobotSpace(limelight_Coral, constants_Limelight.getCameraList(limelight_Coral).get(1), constants_Limelight.getCameraList(limelight_Coral).get(2), constants_Limelight.getCameraList(limelight_Coral).get(3), 0, constants_Limelight.getCameraList(limelight_Coral).get(0), 0);

    }

    // public Optional<Pose2d> getPoseFromAprilTags() {
    //     double[] botpose = localizedPose;
    //     if(botpose.length < 7 || targetID_Coral == -1) return Optional.empty();
    //     return Optional.of(new Pose2d(botpose[0], botpose[1], new Rotation2d(botpose[5])));
    // }
  


    // public Command autoAlgae = new Command()
    // {
        
    //     @Override
    //     public void initialize()
    //     {
    //         s_Swerve.faceAllFoward();
    //         thetaPIDController.setGoal(0);
    //     }
        
    //     @Override
    //     public void execute()
    //     {
    //         turningSpeed = thetaPIDController.calculate(getXAng_Rad(limelight_Algae)); /*Rads / sec */
    //         xSpeed = xPIDController.calculate(distanceX(limelight_Algae, "Algae")); /*m/sec */
    //         c_Drive.driveAlgae(xSpeed, turningSpeed);
    //     }

    //     @Override
    //     public boolean isFinished()
    //     {
    //         return !u_Controllers.leftStick.button(4).getAsBoolean() || !hasTarget(limelight_Algae);
    //     }

    //     @Override
    //     public void end(boolean interrupted)
    //     {

    //     }
    // };


    public Command alignReef = new Command()
    {

        private Timer dontSeeTagTimer, stopTimer;
        private double tagID = -1;
        private double xSpeed, ySpeed, thetaSpeed;
        private double[] positions;
        private ProfiledPIDController thetaPIDController, xPIDController, yPIDController;


        

        
        @Override
        public void initialize()
        {
            s_Swerve.faceAllFoward();

            this.stopTimer = new Timer();
            this.stopTimer.start();
            this.dontSeeTagTimer = new Timer();
            this.dontSeeTagTimer.start();

            xPIDController = new ProfiledPIDController(constants_Limelight.X_REEF_ALIGNMENT_P, 0, 0, constants_Auto.LINEAR_CONSTRAINTS);
            yPIDController = new ProfiledPIDController(constants_Limelight.Y_REEF_ALIGNMENT_P, 0, constants_Limelight.Y_REEF_ALIGNMENT_D, constants_Auto.LINEAR_CONSTRAINTS);
            thetaPIDController = new ProfiledPIDController(constants_Limelight.ROT_REEF_ALIGNMENT_P, 0, 0, constants_Auto.THETA_CONTROLLER_CONSTRAINTS);


            thetaPIDController.setGoal(constants_Limelight.ROT_SETPOINT_REEF_ALIGNMENT);
            thetaPIDController.setTolerance(constants_Limelight.ROT_TOLERANCE_REEF_ALIGNMENT);

            xPIDController.setGoal(constants_Limelight.X_SETPOINT_REEF_ALIGNMENT);
            xPIDController.setTolerance(constants_Limelight.X_TOLERANCE_REEF_ALIGNMENT);

            yPIDController.setGoal(isRightScore ? constants_Limelight.Y_SETPOINT_REEF_ALIGNMENT_RIGHT : -constants_Limelight.Y_SETPOINT_REEF_ALIGNMENT_LEFT);
            yPIDController.setTolerance(constants_Limelight.Y_TOLERANCE_REEF_ALIGNMENT);

            tagID = LimelightHelpers.getFiducialID(limelight_Coral);
        }
        
        @Override
        public void execute()
        {
            if (hasTarget(limelight_Coral) && getTargetID(limelight_Coral) == tagID)
            {
                this.dontSeeTagTimer.reset();

                positions = LimelightHelpers.getBotPose_TargetSpace(limelight_Coral);
                SmartDashboard.putNumber("x", positions[2]);

                xSpeed = -xPIDController.calculate(positions[2]);
                SmartDashboard.putNumber("xSpeed", xSpeed);
                ySpeed = -yPIDController.calculate(positions[0]);
                SmartDashboard.putNumber("ySpeed", ySpeed);
                thetaSpeed = -thetaPIDController.calculate(Math.toRadians(positions[4]));
                SmartDashboard.putNumber("Theta Speed", thetaSpeed);

                c_Drive.driveReef(0, ySpeed, 0); //x -> forward   y -> left

                if (!thetaPIDController.atSetpoint() ||
                    !yPIDController.atSetpoint() ||
                    !xPIDController.atSetpoint()) {
                    stopTimer.reset();
                }

            } else {
                c_Drive.driveReef(0, 0, 0);
            }

                SmartDashboard.putNumber("poseValidTimer", stopTimer.get());
        }

        @Override
        public boolean isFinished()
        {
            // Requires the robot to stay in the correct position for 0.3 seconds, as long as it gets a tag in the camera
            return this.dontSeeTagTimer.hasElapsed(constants_Limelight.DONT_SEE_TAG_WAIT_TIME) ||
                stopTimer.hasElapsed(constants_Limelight.POSE_VALIDATION_TIME);
        }

        @Override
        public void end(boolean interrupted)
        {
            c_Drive.driveReef(0, 0, 0);
        }
    };


    public double getXAng_Rad(String camera)
    {
        return Math.toRadians(LimelightHelpers.getTX(camera));
    }

    public double getYAng_Rad(String camera)
    {
        return Math.toRadians(LimelightHelpers.getTY(camera));
    }

    public boolean hasTarget(String camera)
    {
        return LimelightHelpers.getTV(camera);
    }

    public double distanceX(String camera, String target)
    {
        return ((constants_Limelight.getTargetHeight(target) - constants_Limelight.getCameraList(camera).get(3)) / (Math.tan((getYAng_Rad(camera)+constants_Limelight.getCameraList(camera).get(0))))); //meters from target to the camera
    }

    public int getTargetID(String camera)
    {
        return (int)(LimelightHelpers.getFiducialID(camera));
    }

    // pub\\l\ic double distanceX(String camera)
    // {
    //     return (Math.cos(Math.toRadians(distanceY/xAng_Coral))) * 0.0254;//meters to center of robot
    // }

    @Override
    public void periodic()
    {
        LimelightHelpers.setPipelineIndex(limelight_Coral, 0);
        LimelightHelpers.setPipelineIndex(limelight_Algae, 0);
    }
}

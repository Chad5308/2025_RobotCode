package frc.robot.Commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Subsystems.Drive.Swerve;

public class TimedDrive extends Command{
    private Swerve swerve;
    private Timer timer;
    private double x;
    private double y;
    private double time;
    private double degreesPerSecond;
    
    public TimedDrive(Swerve swerve, double time, double x, double y, double degreesPerSecond){
        this.swerve = swerve;
        timer = new Timer();
        this.time = time;

        this.x = x;
        this.y = y;
        this.degreesPerSecond = degreesPerSecond;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        swerve.setModuleStates(new ChassisSpeeds(x,y,degreesPerSecond));
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setModuleStates(new ChassisSpeeds(0,0,0));
    }
    @Override
    public boolean isFinished() {
        return timer.get() > time;
    }
}

package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Robot;

public class AutonomousCommand extends CommandBase {
    
    private final DriveSubsystem m_autoDrive;

    /**
     * Creates a new ExampleCommand.
     *
     * @param aDrive The subsystem used by this command.
     */
    public AutonomousCommand(DriveSubsystem aDrive) {
        m_autoDrive = aDrive;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_autoDrive);
    }

    @Override
    public void initialize() {
        m_autoDrive.resetDistance();
    }

    @Override
    public void execute() {
        // m_autoDrive.drive(0.0, 0.75, 0, false);
        // shoot ball
        // load path
    }

    @Override
    public boolean isFinished() {
        double distance = (m_autoDrive.getDistance() * DriveConstants.DrivetrainKf
                * DriveConstants.circumferenceOfWheel);
        
        return distance >= DriveConstants.distanceGoal;
    }


    // @Override
    // public void cancel(){
    // // if (isFinished() == true){
    // // end(m_autonomousCommand);
    // // }
    // }


    
}
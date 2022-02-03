package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class AutonomousCommand extends CommandBase {

    private final DriveSubsystem m_subsystem;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public AutonomousCommand(DriveSubsystem subsystem) {
        m_subsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        m_subsystem.resetDistance();
    }

    @Override
    public void execute() {
        m_subsystem.drive(0.0, 0.75, 0, false);

    }

    @Override
    public boolean isFinished() {
        double distance = (m_subsystem.getDistance() * DriveConstants.DrivetrainKf
                * DriveConstants.circumferenceOfWheel);
        if (distance >= DriveConstants.distanceGoal) {
            return true;
        } else {
            return false;
        }
    }


    // @Override
    // public void cancel(){
    // // if (isFinished() == true){
    // // end(m_autonomousCommand);
    // // }
    // }


    
}
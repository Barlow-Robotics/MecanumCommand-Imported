package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.AutonomousNavigation;
import frc.robot.subsystems.DriveSubsystem;

public class AutonomousCommand extends CommandBase {

    private final DriveSubsystem m_autoDrive;
    private final AutonomousNavigation m_autoPath;

    /**
     * Creates a new ExampleCommand.
     *
     * @param aDrive The subsystem used by this command.
     */
    public AutonomousCommand(DriveSubsystem aDrive, AutonomousNavigation nav) {
        m_autoDrive = aDrive;
        m_autoPath = nav;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_autoDrive, m_autoPath);
    }

    @Override
    public void initialize() {
        m_autoDrive.resetDistance();
    }

    @Override
    public void execute() {
        // m_autoDrive.drive(0.0, 0.75, 0, false);
        // shoot ball
        //m_autoPath.testPath.loadPath();
    
            //PPSwerveControllerCommand command = new PPSwerveControllerCommand(
        // m_autoPath.testPath,
        // m_autoDrive.m_odometry,
        // m_autoDrive.kinematics,
        // m_autoDrive.xController,
        // m_autoDrive.yController,
        // m_autoDrive.thetaController,
        // m_autoDrive.outputModuleStates,
        // m_autoDrive.requirements
        // );
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
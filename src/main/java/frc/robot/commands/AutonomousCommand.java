package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
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
     * @param auto The subsystem used by this command.
     */
    public AutonomousCommand(DriveSubsystem auto, AutonomousNavigation nav) {
        m_autoDrive = auto;
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
        m_autoPath.testPath.loadPath();


        /*PPSwerveControllerCommand command = new PPSwerveControllerCommand(
        m_autoPath.testPath,
        m_autoDrive.m_odometry,
        kinematics,
        xController,
        yController,
        thetaController,
        outputModuleStates,
        requirements
        );
        */
  }

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
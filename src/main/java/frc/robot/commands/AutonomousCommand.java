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

        // This will load the file "Example Path.path" and generate it with a max velocity of 8 m/s and a max acceleration of 5 m/s^2
        // PathPlannerTrajectory tarmacB1ToBBallDBBallC = PathPlanner.loadPath("TarmacB1_to_BBallD_BBallC", 8, 5);
        // PathPlannerTrajectory tarmacB1ToBBallD = PathPlanner.loadPath("TarmacB1_to_BBallD", 8, 5);
        // PathPlannerTrajectory tarmacB2ToBBallBBBallC = PathPlanner.loadPath("TarmacB2_to_BBallB_BBallC", 8, 5);
        // PathPlannerTrajectory tarmacB2ToBBallB = PathPlanner.loadPath("TarmacB2_to_BBallB", 8, 5);
        // PathPlannerTrajectory tarmacB2ToBBallCBBallB = PathPlanner.loadPath("TarmacB2_to_BBallC_BBallB", 8, 5);
        // PathPlannerTrajectory tarmacB2ToBBallCBallD = PathPlanner.loadPath("TarmacB2_to_BBallC_BBallD", 8, 5);
        // PathPlannerTrajectory tarmacB2ToBBallC = PathPlanner.loadPath("TarmacB2_to_BBallC", 8, 5);
        // PathPlannerTrajectory tarmacR1ToRBallDRBallE = PathPlanner.loadPath("TarmacR1_to_RBallD_RBallE", 8, 5);
        // PathPlannerTrajectory tarmacR1ToRBallD = PathPlanner.loadPath("TarmacR1_to_RBallD", 8, 5);
        // PathPlannerTrajectory tarmacR1ToRBallERBallF = PathPlanner.loadPath("TarmacR1_to_RBallE_RBallF", 8, 5);
        // PathPlannerTrajectory tarmacR1ToRBallE = PathPlanner.loadPath("TarmacR1_to_RBallE", 8, 5);
        // PathPlannerTrajectory tarmacR2ToRBallFRBallE = PathPlanner.loadPath("TarmacR2_to_RBallF_RBallE", 8, 5);
        // PathPlannerTrajectory tarmacR2ToRBallF = PathPlanner.loadPath("TarmacR2_to_RBallF", 8, 5);
        // PathPlannerTrajectory tarmacR1ToRBallD = PathPlanner.loadPath("TarmacR1_to_RBallD", 8, 5);
        PathPlannerTrajectory testPath = PathPlanner.loadPath("Test_Path", 8, 5);
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
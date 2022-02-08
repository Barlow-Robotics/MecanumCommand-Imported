package frc.robot.autopathplanner;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.autopathplanner.TrajectoryFollowerCommand;

public class MPController {

    public DriveSubsystem drive = new DriveSubsystem();

    /**
     * Creates a new trajectory follower command for a mecanum drivetrain
     * @param trajectory - The desired trajectory you want the command to follow.
     * @param maxSpeedFt - the max speed of the trajectory you supplied in m/s.
     * @return MecanumControllerCommand that can be scheduled to run.
     */
    public Command createTrajectoryFollowerCommand(Trajectory trajectory, Trajectory headingTrajectory, double trajectoryMaxSpeed) {
        TrajectoryFollowerCommand command = new TrajectoryFollowerCommand(
            trajectory,
            headingTrajectory,
            drive::getPose,
            drive.getFeedforward(),
            drive.getKinematics(),
            drive.getXController(),
            drive.getYController(),
            drive.getThetaController(),
            trajectoryMaxSpeed,
            drive.getFrontLeftPIDController(),
            drive.getBackLeftPIDController(),
            drive.getFrontRightPIDController(),
            drive.getBackRightPidController(),
        );
        return command.andThen(() -> drive.setOutputVolts(new MecanumDriveMotorVoltages(0, 0, 0, 0)));
         // .andThen(() -> drive.setOutputVolts(new MecanumDriveMotorVoltages(0, 0, 0, 0)))
      }
      /**
     * Creates a new trajectory follower command for a mecanum drivetrain
     * @param trajectory - The desired trajectory you want the command to follow.
     * @param maxSpeedFt - the max speed of the trajectory you supplied in m/s.
     * @param nextCommand - The next command to run afterwards
     * @return MecanumControllerCommand that can be scheduled to run.
     */
    public Command createTrajectoryFollowerCommand(Trajectory trajectory, Trajectory headingTrajectory, double trajectoryMaxSpeedMs, Command nextCommand) {
        TrajectoryFollowerCommand command = new TrajectoryFollowerCommand(
            trajectory,
            headingTrajectory,
            drive::getPose,
            drive.getFeedforward(),
            drive.getKinematics(),
            drive.getXController(),
            drive.getYController(),
            drive.getThetaController(),
            trajectoryMaxSpeedMs,
            drive.getFrontLeftPIDController(),
            drive.getBackLeftPIDController(),
            drive.getFrontRightPIDController(),
            drive.getBackRightPidController(),
        );
        return command.andThen(nextCommand);
         // .andThen(() -> drive.setOutputVolts(new MecanumDriveMotorVoltages(0, 0, 0, 0)))
      }  


}
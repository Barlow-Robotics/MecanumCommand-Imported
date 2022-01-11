package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.PWM;
import frc.robot.Constants.DriveConstants;

public class AutonomousCommand extends CommandBase{

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

  @Override //implement this: see the drive method in DriveSubsytems, not sure what numbers r needed
  public void execute() {
    m_subsystem.drive(0.0, 0.75, 3, true); //JUST PLACEHOLDER CODE!!
  }

  @Override
  public boolean isFinished(){
    double distance = (m_subsystem.getDistance()*DriveConstants.DrivetrainKf*DriveConstants.circumferenceOfWheel);
    if (distance>=DriveConstants.distanceGoal){
        return true;
    }
    else {
        return false;
    }
  }
  // @Override
  // public void cancel(){ 
  //   // if (isFinished() == true){
  //   //     end(m_autonomousCommand); 
  //   // }
  // }
}
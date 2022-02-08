// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.ArmBarConstants;
import frc.robot.autopathplanner.MPController;
import frc.robot.autopathplanner.Trajectories;
import frc.robot.commands.Climb;
//import frc.robot.sim.PhysicsSim;
import frc.robot.subsystems.ArmBar;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  
  Trajectories trajectories = new Trajectories();
  MPController mpController;

  public static Trajectory[] selectedTrajectory = new Trajectory[2];

  public static Timer m_autoTimer = new Timer();

  String selectedPath;

  double trajectoryTime;

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  // Paths 
  String testPathBackwards = "Test_Path_Backwards.csv";
  String testPathForwards = "Test_Path_Forwards.csv";
  String testPathLeft = "Test_Path_Left.csv";
  String testPathRight = "Test_Path_Right.csv";
  String testPathLoop = "Test_Path_Loop.csv";

  String b1_BBallD_BBallC = "TarmacB1_to_BBallD_BBallC.csv";
  String b1_BBallD = "TarmacB1_to_BBallD.csv";
  String b2_BBallB_BBallC = "TarmacB2_to_BBallB_BBallC.csv";
  String b2_BBallB = "TarmacB2_to_BBallB.csv";
  String b2_BBallC_BBallB = "TarmacB2_to_BBallC_BBallB.csv";
  String b2_BBallC_BBallD = "TarmacB2_to_BBallC_BBalD.csv";
  String b2_BBallC = "TarmacB2_to_BBallC.csv";

  String r1_RBallD_RBallE = "TarmacR1_to_RBallD_RBallE.csv";
  String r1_RBallD = "TarmacR1_to_RBallD.csv";
  String r1_RBallE_RBallF = "TarmacR1_to_RBallE_RBallF.csv";
  String r1_RBallE = "TarmacR1_to_RBallE.csv";
  String r2_RBallF_RBallE = "TarmacR2_to_RBallF_RBallE.csv";
  String r2_RBallF = "TarmacR2_to_RBallF.csv";

  Command pathplannerCommand;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    //Climb.m_armBarMotor.set(ArmBarConstants.defaultangle); (<-- set to 180, set motors to break)
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {

    mpController = new MPController();
    
    // Sets the path to be driven. 
    selectedPath = testPathForwards;

    for (int i = 0; i < 2; i++){
      selectedTrajectory[i] = trajectories.getTrajectoryFromCSV(selectedPath)[i];
    }

    mpController.drive.setMotorConfig();

    trajectoryTime = selectedTrajectory[0].getTotalTimeSeconds();
    System.out.println("Total Trajectory Time: " + trajectoryTime + "s");

    // Reset encoders
    mpController.drive.resetEncoders();

    // Initialize our odometry
    mpController.drive.initializeOdometry();

    // Ensure our odometry is at 0
    mpController.drive.reset();

    // Reset odometry to starting point of path
    mpController.drive.resetOdometry(selectedTrajectory[0].getInitialPose());

    // Update our odometry
    mpController.drive.periodic();    

    pathplannerCommand = mpController.createTrajectoryFollowerCommand(selectedTrajectory[0], selectedTrajectory[1], 2.5);

    pathplannerCommand.schedule();

    m_autoTimer.reset();
    m_autoTimer.start();
  

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    PathPlannerTrajectory examplePath = PathPlanner.loadPath("Example Path", 8, 5);

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    mpController.drive.periodic();
    
    if(pathplannerCommand.isFinished()){
      m_autoTimer.stop();
    }
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}


  @Override
  public void simulationInit() {
  }
  
  @Override
  public void simulationPeriodic() {
		//PhysicsSim.getInstance().run();
	}
}

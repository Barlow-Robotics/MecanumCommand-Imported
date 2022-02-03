// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;

import frc.robot.subsystems.ArmBar;

import frc.robot.commands.ExtendIntake;
import frc.robot.commands.RetractIntake;
import frc.robot.commands.StartIntake;
//import frc.robot.commands.StartReceiving;
import frc.robot.commands.StartShooting;
import frc.robot.commands.StopIntake;
//import frc.robot.commands.StopReceiving;
import frc.robot.commands.StopShooting;
import frc.robot.commands.Climb;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ShooterIndex;
import frc.robot.subsystems.ArmBar;
import frc.robot.subsystems.UnderGlow;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import java.util.List;
import edu.wpi.first.networktables.*;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
@SuppressWarnings("PMD.ExcessiveImports")
public class RobotContainer {
    // The robot's subsystems
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    // private final Intake m_intake = new Intake();
    // private final ShooterIndex m_shooter = new ShooterIndex();
    // private final ArmBar m_armBar = new ArmBar();
    // private final UnderGlow underGlow = new UnderGlow() ;

    // The driver's controller
    Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort); // change

    // ArmBar armBar = new ArmBar();

    // private final JoystickButton intakeButton = new JoystickButton(m_driverController,Constants.Logitech_F310_Controller.Right_Bumper);
    // private final JoystickButton extendButton = new JoystickButton(m_driverController,Constants.Logitech_F310_Controller.Left_Bumper);
    // private final JoystickButton shooterButton = new JoystickButton(m_driverController,Constants.Logitech_F310_Controller.Button_A);
    // private final JoystickButton climbButton = new JoystickButton(m_driverController,Constants.Logitech_F310_Controller.Back_Button);

    private final JoystickButton receiverButton = new JoystickButton(m_driverController,Constants.Logitech_F310_Controller.Button_Y); 

   

   // Commands

    // private final StartIntake startIntakeCommand = new StartIntake(m_intake);
    // private final StopIntake stopIntakeCommand = new StopIntake(m_intake);
    // private final ExtendIntake extendIntakeCommand = new ExtendIntake(m_intake);
    // private final RetractIntake retractIntakeCommand = new RetractIntake(m_intake);
    // private final StartShooting startShootingCommand = new StartShooting(m_shooter);
    // private final StopShooting stopShootingCommand = new StopShooting(m_shooter);
    // private final Climb climbCommand = new Climb(m_armBar);

//     private final StartReceiving startReceivingCommand = new StartReceiving (m_shooter);
//     private final StopReceiving stopReceivingCommand = new StopReceiving(m_shooter);
    
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        if ( m_driverController.getName() == Constants.OIConstants.LogitechF310Name ) {
            m_driverController.setTwistChannel(Constants.OIConstants.rightXAxis);            
        }

        System.out.println("The name of the controller is " + m_driverController.getName()) ;

        // // Configure default commands
        // // Set the default drive command to split-stick arcade drive
        m_robotDrive.setDefaultCommand(
                // A split-stick arcade command, with forward/backward controlled by the left
                // hand, and turning controlled by the right.

                // new RunCommand(() -> {
                //     m_robotDrive.drive(-m_driverController.getRawAxis(Constants.OIConstants.leftYAxis)* 0.5,
                //             m_driverController.getRawAxis(Constants.OIConstants.leftXAxis)* 0.5,
                //             m_driverController.getRawAxis(Constants.OIConstants.rightXAxis)* 0.5, false);
                // }, m_robotDrive));

                new RunCommand(() -> {
//                    m_robotDrive.drive(-0.5, 0.0,0.0, false);
                    m_robotDrive.drive(-0.0, 0.0,0.0, false);
                }, m_robotDrive));

                // new RunCommand(() -> {

                //     NetworkTableInstance.getDefault().getEntry("joystick/raw_left_y").setDouble(m_driverController.getRawAxis(Constants.OIConstants.leftYAxis));
                //     NetworkTableInstance.getDefault().getEntry("joystick/raw_left_x").setDouble(m_driverController.getRawAxis(Constants.OIConstants.leftXAxis));
                //     NetworkTableInstance.getDefault().getEntry("joystick/raw_right_x").setDouble(m_driverController.getRawAxis(Constants.OIConstants.rightXAxis));

                //     NetworkTableInstance.getDefault().getEntry("joystick/getY").setDouble(m_driverController.getY());
                //     NetworkTableInstance.getDefault().getEntry("joystick/getX").setDouble(m_driverController.getX());
                //     NetworkTableInstance.getDefault().getEntry("joystick/getTwist").setDouble(m_driverController.getTwist());
                    

                //     m_robotDrive.drive(
                //         //0.0,
                //         m_driverController.getRawAxis(Constants.OIConstants.leftYAxis),
                //         ////m_driverController.getY(),
                //         //0.0,
                //         m_driverController.getRawAxis(Constants.OIConstants.leftXAxis),
                //         //0.2,
                //         m_driverController.getRawAxis(Constants.OIConstants.rightXAxis), 
                //         // m_driverController.getX(),
                //         // m_driverController.getTwist(), 
                //         false
                //         );
                // }, m_robotDrive);



        // underGlow.setDefaultCommand( new RunCommand( () -> {}, underGlow ));

    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * calling passing it to a {@link JoystickButton}.
     */
    private void configureButtonBindings() {
        // Drive at half speed when the right bumper is held
        // new JoystickButton(m_driverController, Constants.OIConstants.halfSpeedButton);
        // .whenPressed(() -> m_robotDrive.setMaxOutput(0.5));
        // .whenReleased(() -> m_robotDrive.setMaxOutput(1));
        // intakeButton.whenPressed(startIntakeCommand).whenReleased(stopIntakeCommand);
        // extendButton.whenPressed(extendIntakeCommand).whenReleased(retractIntakeCommand);
        // shooterButton.whenPressed(startShootingCommand).whenReleased(stopShootingCommand);
        // climbButton.whenPressed(climbCommand); //is whenPressed right or will it keep trying to restart itself
        //receiverButton.whenPressed(startReceivingCommand).whenReleased(stopReceivingCommand);
        
}

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(DriveConstants.kDriveKinematics);

        // An example trajectory to follow. All units in meters.

        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(3, 0, new Rotation2d(0)), config);

        MecanumControllerCommand mecanumControllerCommand = new MecanumControllerCommand(exampleTrajectory,
                m_robotDrive::getPose, DriveConstants.kFeedforward, DriveConstants.kDriveKinematics,

                // Position contollers
                new PIDController(AutoConstants.kPXController, 0, 0),
                new PIDController(AutoConstants.kPYController, 0, 0),
                new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0,
                        AutoConstants.kThetaControllerConstraints),

                // Needed for normalizing wheel speeds
                AutoConstants.kMaxSpeedMetersPerSecond,

                // Velocity PID's
                new PIDController(DriveConstants.kPFrontLeftVel, 0, 0),
                new PIDController(DriveConstants.kPRearLeftVel, 0, 0),
                new PIDController(DriveConstants.kPFrontRightVel, 0, 0),
                new PIDController(DriveConstants.kPRearRightVel, 0, 0), m_robotDrive::getCurrentWheelSpeeds,
                m_robotDrive::setDriveSpeedControllersVolts, // Consumer for the output motor voltages
                m_robotDrive);

        // Reset odometry to the starting pose of the trajectory.
        m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

        // Run path following command, then stop at the end.
        return mecanumControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));

        // return null ;
    }
}

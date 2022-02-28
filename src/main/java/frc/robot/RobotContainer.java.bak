// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;

import frc.robot.subsystems.ArmBar;

import frc.robot.commands.GotoShootingPosition;
import frc.robot.commands.GotoIntakePosition;
import frc.robot.commands.StartIntake;
import frc.robot.commands.StartShooting;
import frc.robot.commands.StopIntake;
import frc.robot.commands.StopShooting;
import frc.robot.commands.Climb;

import frc.robot.subsystems.DriveSubsystem;
//import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ShooterIndex;
import frc.robot.subsystems.ArmBar;
import frc.robot.subsystems.UnderGlow;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;
import edu.wpi.first.networktables.*;

import edu.wpi.first.wpilibj2.command.button.Button ;
import frc.robot.JoystickAnalogButton; 

import frc.robot.subsystems.Intake;

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
    private final Intake m_intake = new Intake();
    private final ShooterIndex m_shooter = new ShooterIndex();
    private final ArmBar m_armBar = new ArmBar();
    // private final UnderGlow underGlow = new UnderGlow() ;

    // The driver's controller
    Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort); // change

    private JoystickButton intakeButton ;

    private JoystickButton liftToShootingButton ;

    private JoystickButton liftToIntakeButton ;

    private JoystickButton shooterButton ;

    private JoystickButton climbButton ;

    private int Forward_Speed_Axis ;
    private int Lateral_Speed_Axis ;
    private int Yaw_Axis ;

    private double Forward_Speed_Attenuation = 0.5 ;
    private double Lateral_Speed_Attenuation = 0.5 ;
    private double Yaw_Attenuation = 0.5 ;




    // Commands

    private final StartIntake startIntakeCommand = new StartIntake(m_shooter, m_intake);
    private final StopIntake stopIntakeCommand = new StopIntake(m_intake, m_shooter);

    private final GotoShootingPosition shootingPositionCommand = new GotoShootingPosition(m_shooter);
    private final GotoIntakePosition intakePositionCommand = new GotoIntakePosition(m_shooter);

    private final StartShooting startShootingCommand = new StartShooting(m_shooter);
    private final StopShooting stopShootingCommand = new StopShooting(m_shooter);

    private final Climb climbCommand = new Climb(m_armBar, m_robotDrive);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        // Configure the button bindings
        configureButtonBindings();


        // // Configure default commands
        // // Set the default drive command to split-stick arcade drive
        m_robotDrive.setDefaultCommand(
                // A split-stick arcade command, with forward/backward controlled by the left
                // hand, and turning controlled by the right.

                new RunCommand(() -> {
                    m_robotDrive.drive(
                        m_driverController.getRawAxis(Forward_Speed_Axis) * Forward_Speed_Attenuation,
                        m_driverController.getRawAxis(Lateral_Speed_Axis) * Lateral_Speed_Attenuation,
                        m_driverController.getRawAxis(Yaw_Axis) * Yaw_Attenuation,
                        false
                    );
                }, m_robotDrive));

        // new RunCommand(() -> {
        // m_robotDrive.drive(0.1, 0.0,0.0, false);
        // //m_robotDrive.drive(-0.0, 0.0,0.0, false);
        // }, m_robotDrive));

        // underGlow.setDefaultCommand( new RunCommand( () -> {}, underGlow ));

        // m_armBar.setDefaultCommand(
        // new RunCommand(() -> {
        // m_armBar.ResetPosition( ) ;
        // }, m_armBar)
        // );

    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * calling passing it to a {@link JoystickButton}.
     */
    private void configureButtonBindings() {

        String controllerType = m_driverController.getName() ;

        if ( controllerType.equals( "RadioMas TX16S Joystick") ){
            Forward_Speed_Axis = Constants.RadioMaster_Controller.Right_Gimbal_Y;
            Lateral_Speed_Axis = Constants.RadioMaster_Controller.Right_Gimbal_Y;
            Yaw_Axis = Constants.RadioMaster_Controller.Left_Gimbal_X;

            Forward_Speed_Attenuation = Constants.RadioMaster_Controller.Forward_Axis_Attenuation ;
            Lateral_Speed_Attenuation = Constants.RadioMaster_Controller.Lateral_Axis_Attenuation ;
            Yaw_Attenuation = Constants.RadioMaster_Controller.Yaw_Axis_Attenuation ;


            // these all need to be updated.
            
            // wpk need to test this out.

            intakeButton = new JoystickAnalogButton(m_driverController, 1, 0.5);
//            intakeButton = new JoystickButton(m_driverController, Constants.Logitech_F310_Controller.Right_Bumper);
            liftToShootingButton = new JoystickButton(m_driverController, Constants.Logitech_F310_Controller.Button_Y);
            liftToIntakeButton = new JoystickButton(m_driverController, Constants.Logitech_F310_Controller.Button_B);
            shooterButton = new JoystickButton(m_driverController, Constants.Logitech_F310_Controller.Button_A);
            climbButton = new JoystickButton(m_driverController, Constants.Logitech_F310_Controller.Back_Button);

        } else if (controllerType.equals( "Controller (Gamepad F310)"))  {

            Forward_Speed_Axis = Constants.Logitech_F310_Controller.Right_Stick_Y;
            Lateral_Speed_Axis = Constants.Logitech_F310_Controller.Right_Stick_X;
            Yaw_Axis = Constants.Logitech_F310_Controller.Left_Stick_X;
            Forward_Speed_Attenuation = Constants.Logitech_F310_Controller.Forward_Axis_Attenuation ;
            Lateral_Speed_Attenuation = Constants.Logitech_F310_Controller.Lateral_Axis_Attenuation ;
            Yaw_Attenuation = Constants.Logitech_F310_Controller.Yaw_Axis_Attenuation ;

            intakeButton = new JoystickButton(m_driverController, Constants.Logitech_F310_Controller.Right_Bumper);
            liftToShootingButton = new JoystickButton(m_driverController, Constants.Logitech_F310_Controller.Button_Y);
            liftToIntakeButton = new JoystickButton(m_driverController, Constants.Logitech_F310_Controller.Button_B);
            shooterButton = new JoystickButton(m_driverController, Constants.Logitech_F310_Controller.Button_A);
            climbButton = new JoystickButton(m_driverController, Constants.Logitech_F310_Controller.Back_Button);
        
        } else if (controllerType.equals( "Xbox Controller")) {

            // sometimes the logi-tech controller shows up this way when the X/D switch is in the "X" position
            Forward_Speed_Axis = Constants.Xbox_Controller.Right_Stick_Y;
            Lateral_Speed_Axis = Constants.Xbox_Controller.Right_Stick_X;
            Yaw_Axis = Constants.Xbox_Controller.Left_Stick_X;
            Forward_Speed_Attenuation = Constants.Xbox_Controller.Forward_Axis_Attenuation ;
            Lateral_Speed_Attenuation = Constants.Xbox_Controller.Lateral_Axis_Attenuation ;
            Yaw_Attenuation = Constants.Xbox_Controller.Yaw_Axis_Attenuation ;

            intakeButton = new JoystickButton(m_driverController, Constants.Xbox_Controller.Right_Bumper);
            liftToShootingButton = new JoystickButton(m_driverController, Constants.Xbox_Controller.Button_Y);
            liftToIntakeButton = new JoystickButton(m_driverController, Constants.Xbox_Controller.Button_B);
            shooterButton = new JoystickButton(m_driverController, Constants.Xbox_Controller.Button_A);
            climbButton = new JoystickButton(m_driverController, Constants.Xbox_Controller.Back_Button);


        } else if ( controllerType.equals("Logitech Dual Action" )) {
            Forward_Speed_Axis = Constants.Logitech_Dual_Action.Right_Stick_Y;
            Lateral_Speed_Axis = Constants.Logitech_Dual_Action.Right_Stick_X;
            Yaw_Axis = Constants.Logitech_Dual_Action.Left_Stick_X;
            Forward_Speed_Attenuation = Constants.Logitech_Dual_Action.Forward_Axis_Attenuation ;
            Lateral_Speed_Attenuation = Constants.Logitech_Dual_Action.Lateral_Axis_Attenuation ;
            Yaw_Attenuation = Constants.Logitech_Dual_Action.Yaw_Axis_Attenuation ;

            intakeButton = new JoystickButton(m_driverController, Constants.Logitech_Dual_Action.Right_Bumper);
            liftToShootingButton = new JoystickButton(m_driverController, Constants.Logitech_Dual_Action.Button_Y);
            liftToIntakeButton = new JoystickButton(m_driverController, Constants.Logitech_Dual_Action.Button_B);
            shooterButton = new JoystickButton(m_driverController, Constants.Logitech_Dual_Action.Button_A);
            climbButton = new JoystickButton(m_driverController, Constants.Logitech_Dual_Action.Back_Button);

        } else {

            // no buttons will be configured and we'll crash shortly after. That's OK, we want to crash so we can find this issue.

        }





        // Drive at half speed when the right bumper is held
        // new JoystickButton(m_driverController,
        // Constants.OIConstants.halfSpeedButton);
        // .whenPressed(() -> m_robotDrive.setMaxOutput(0.5));
        // .whenReleased(() -> m_robotDrive.setMaxOutput(1));

        intakeButton.whenPressed(startIntakeCommand).whenReleased(stopIntakeCommand);

        liftToShootingButton.whenPressed(shootingPositionCommand);

        liftToIntakeButton.whenPressed(intakePositionCommand);

        // extendButton.whenPressed(extendIntakeCommand).whenReleased(retractIntakeCommand);

        shooterButton.whenPressed(startShootingCommand).whenReleased(stopShootingCommand);
        climbButton.whenPressed(climbCommand);

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(DriveConstants.kDriveKinematics);

        String trajectoryJSON = "./pathplanner/generatedJSON/Test_Path_Forwards.wpilib.json";
        Trajectory trajectory = null;

        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
        }

        if (trajectory == null) {
            // An example trajectory to follow. All units in meters.
            trajectory = TrajectoryGenerator.generateTrajectory(
                    // Start at the origin facing the +X direction
                    new Pose2d(0, 0, new Rotation2d(0)),
                    // List.of(new Translation2d(1, 1), new Translation2d(-2,1)),
                    List.of(new Translation2d(0, 1)),
                    new Pose2d(0, 3, new Rotation2d(0)),
                    config);
        }

        MecanumControllerCommand mecanumControllerCommand = new MecanumControllerCommand(
                trajectory,
                m_robotDrive::getPose,
                DriveConstants.kFeedforward,
                DriveConstants.kDriveKinematics,

                // Position contollers
                new PIDController(AutoConstants.kPXController, 0, 0),
                new PIDController(AutoConstants.kPYController, 0, 0),
                new ProfiledPIDController(
                        AutoConstants.kPThetaController, 0, 0,
                        AutoConstants.kThetaControllerConstraints),

                // Needed for normalizing wheel speeds
                AutoConstants.kMaxSpeedMetersPerSecond,

                // Velocity PID's
                new PIDController(DriveConstants.kPFrontLeftVel, 0, 0),
                new PIDController(DriveConstants.kPRearLeftVel, 0, 0),
                new PIDController(DriveConstants.kPFrontRightVel, 0, 0),
                new PIDController(DriveConstants.kPRearRightVel, 0, 0),
                m_robotDrive::getCurrentWheelSpeeds,
                m_robotDrive::setDriveMotorControllersVolts, // Consumer for the output motor voltages
                m_robotDrive);

        // Reset odometry to the starting pose of the trajectory.
        Pose2d ip = trajectory.getInitialPose();
        m_robotDrive.resetOdometry(ip);
        // Run path following command, then stop at the end.
        return mecanumControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));

    }

}

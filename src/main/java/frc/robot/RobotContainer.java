// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;

import frc.robot.Constants.*;

import frc.robot.subsystems.*;
import frc.robot.commands.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand ;

import java.io.FileWriter;
import java.util.ArrayList;
import java.util.HashMap;
import edu.wpi.first.networktables.*;
import java.util.Map;

import com.pathplanner.lib.*;
import com.pathplanner.lib.PathPlannerTrajectory.*;
import java.lang.Runnable ;



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
    private final ShooterIndex m_shooterIndex = new ShooterIndex();
    public final ArmBar m_armBar = new ArmBar();
    public final Vision m_vision = new Vision();
//    public final NavigationSubsystem m_nav = new NavigationSubsystem() ;
    // private final UnderGlow underGlow = new UnderGlow() ;

    // The driver's controller
    Joystick m_driverController;

    // Gamepad
    Joystick m_gamepad;

    private JoystickButton intakeButton;
    private JoystickButton liftToShootingButton;
    private JoystickButton lowerToIntakeButton;
    private JoystickButton shooterLowButton;
    private JoystickButton shooterHighButton;
    private JoystickButton climbButton;
    private JoystickButton abortClimbButton;
    private JoystickButton ejectButton;
    private JoystickButton alignWithTargetButton;
    private JoystickButton switchCameraButton;

    private int Forward_Speed_Axis;
    private int Lateral_Speed_Axis;
    private int Yaw_Axis;

    private double Forward_Speed_Attenuation = 0.5;
    private double Lateral_Speed_Attenuation = 0.5;
    private double Yaw_Attenuation = 0.5;

    // PathPlannerTrajectory trajectory;
    //ArrayList<PathPlannerTrajectory> trajectories;
    HashMap<String, PathPlannerTrajectory> trajectories ;

    // Commands

    private final StartIntake startIntakeCommand = new StartIntake(m_shooterIndex, m_intake);
    private final StopIntake stopIntakeCommand = new StopIntake(m_intake, m_shooterIndex);

    private final GotoShootingPosition shootingPositionCommand = new GotoShootingPosition(m_shooterIndex);
    private final GotoIntakePosition intakePositionCommand = new GotoIntakePosition(m_shooterIndex);

    private final StartShootingLow startShootingLowCommand = new StartShootingLow(m_shooterIndex);
    private final StartShootingHigh startShootingHighCommand = new StartShootingHigh(m_shooterIndex);
    private final StopShooting stopShootingCommand = new StopShooting(m_shooterIndex);

    private Climb climbCommand;
    private AbortClimb abortClimbCommand ;

    private final StartEjecting startEjectingCommand = new StartEjecting(m_intake, m_shooterIndex);
    private final StopEjecting stopEjectingCommand = new StopEjecting(m_intake, m_shooterIndex);

    //private final AlignWithTarget alignWithTargetCommand = new AlignWithTarget(m_vision, m_robotDrive);
    private final SwitchCamera switchCameraCommand = new SwitchCamera();


    private class AutonomousCommandHolder {
        public Pose2d initialPose ;
        public Command autonomousCommand ;
    }

    private Map<String, AutonomousCommandHolder> autonomousCommands ;



    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        // Configure the button bindings
        configureButtonBindings();
        loadTrajectories() ;
        createAutonomousCommands();

        m_armBar.neutralGripperA();
        m_armBar.neutralGripperB();

        m_robotDrive.setDefaultCommand(
            new RunCommand(() -> {
                m_robotDrive.drive(
                    m_driverController.getRawAxis(Forward_Speed_Axis) * Forward_Speed_Attenuation,
                    m_driverController.getRawAxis(Lateral_Speed_Axis) * Lateral_Speed_Attenuation,
                    m_driverController.getRawAxis(Yaw_Axis) * Yaw_Attenuation,
                    false);
            }, m_robotDrive)
        );
    }


    private void loadTrajectory( String name, double maxVel, double maxAccel) {
        PathPlannerTrajectory theTrajectory = PathPlanner.loadPath(name, maxVel, maxAccel) ;
        for( var s : theTrajectory.getStates()) {
            PathPlannerState pps = (PathPlannerState) s ;
            if ( pps.holonomicRotation.getDegrees() > 180.0 ) {

            }
        }
        trajectories.put(name, theTrajectory );
    }

    private void loadTrajectories() {
        trajectories = new HashMap<String, PathPlannerTrajectory>();
        double maxVel = AutoConstants.kMaxSpeedMetersPerSecond ;
        double maxAccel = AutoConstants.kMaxAccelerationMetersPerSecondSquared ;

        // loadTrajectory("0_TarmacB1_to_BBallD", maxVel, maxAccel);
        // loadTrajectory("1_TarmacB1_to_BBallD_BBallC", maxVel, maxAccel);
        // loadTrajectory("2_TarmacB2_to_BBallB", maxVel, maxAccel);
        // loadTrajectory("3_TarmacB2_to_BBallB_BBallC", maxVel, maxAccel);
        // loadTrajectory("4_TarmacB2_to_BBallC", maxVel, maxAccel);
        // loadTrajectory("5_TarmacB2_to_BBallC_BBallB", maxVel, maxAccel);
        // loadTrajectory("6_TarmacB2_to_BBallC_BBallD", maxVel, maxAccel);
        // loadTrajectory("7_TarmacR1_to_RBallD", maxVel, maxAccel);
        // loadTrajectory("8_TarmacR1_to_RBallD_RBallE", maxVel, maxAccel);
        // loadTrajectory("9_TarmacR1_to_RBallE",    maxVel, maxAccel);
        // loadTrajectory("10_TarmacR1_to_RBallE_RBallF", maxVel, maxAccel);
        // loadTrajectory("11_TarmacR2_to_RBallF", maxVel, maxAccel);
        // loadTrajectory("12_TarmacR2_to_RBallF_RBallE", maxVel, maxAccel);
        // loadTrajectory("13_Test_Constant_x", maxVel, maxAccel);
        // loadTrajectory("14_Test_Constant_y", maxVel, maxAccel);
        // loadTrajectory("15_Test_Diagonal", maxVel, maxAccel);
        // loadTrajectory("16_Test_Loop",    maxVel, maxAccel);
        // loadTrajectory("17_Test_Sideways",   maxVel, maxAccel);
        // loadTrajectory("18_Test_U_Shape_Dif_Angle", maxVel, maxAccel);
        // loadTrajectory("19_Test_U_Shape_Same_Angle",  maxVel, maxAccel);
        loadTrajectory("20_TarmacB1_Back_Off_Tarmac", maxVel, maxAccel);
        // loadTrajectory("21_TarmacB1_to_BBallD_Vicinity", maxVel, maxAccel);
        // loadTrajectory("22_TarmacR1_Back_Off_Tarmac", maxVel, maxAccel);
        // loadTrajectory("23_TarmacB2_to_BBallB_Vicinity", maxVel, maxAccel);
        // loadTrajectory("24_TarmacB2_to_BBallC_Vicinity", maxVel, maxAccel);
        // loadTrajectory("25_TarmacR1_to_RBallD_Vicinity", maxVel, maxAccel);
        // loadTrajectory("26_TarmacR1_to_RBallE_Vicinity", maxVel, maxAccel);
        // loadTrajectory("27_TarmacR2_to_RBallF_Vicinity", maxVel, maxAccel);
        // loadTrajectory("29_Autonomous_Path", maxVel, maxAccel);
//        loadTrajectory("Two_Ball_Low_Goal", maxVel, maxAccel);
        loadTrajectory("Two_Ball_Low_Goal", 3.0, maxAccel);
        loadTrajectory("One_Ball_Low_Goal", 3.0, maxAccel);
        loadTrajectory("Two_Ball_High_Goal", 4.0, 3.0);

        // // wpk delete this code block after testing complete
        // PathPlannerTrajectory temp  = PathPlanner.loadPath("Two_Ball_Low_Goal", maxVel, maxAccel) ;
        // try {
        //     String fileName = Filesystem.getDeployDirectory().getPath() + "\\path_samples.csv" ; 
        //     FileWriter myWriter = new FileWriter(fileName);
        //     myWriter.write("Sample#,Rotation\n") ;
        //     for ( int i = 0; i < temp.getStates().size(); i++) {
        //         PathPlannerState pps = (PathPlannerState) temp.getStates().get(i) ;
        //         String s = String.format("%d, %7.4f%n", i, pps.holonomicRotation.getDegrees()) ;
        //         myWriter.write(s);
        //     }
        //     myWriter.close();        
        // } catch ( Exception ex) {
        //     System.out.println("failed to load path") ;
        // }

    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * calling passing it to a {@link JoystickButton}.
     */
    private void configureButtonBindings() {

        if (m_driverController == null) {
            System.out.println("null controller. Using joystick 2");
            m_driverController = new Joystick(2);
        }

        if (m_gamepad == null) {
            System.out.println("null controller. Using joystick 2");
            m_gamepad = new Joystick(1);
        }

        String controllerType = m_driverController.getName();
        System.out.println("The controller name is " + controllerType);
        boolean controllerFound = false;

        Forward_Speed_Axis = Constants.RadioMaster_Controller.Right_Gimbal_Y;
        Lateral_Speed_Axis = Constants.RadioMaster_Controller.Right_Gimbal_X;
        Yaw_Axis = Constants.RadioMaster_Controller.Left_Gimbal_X;

        Forward_Speed_Attenuation = Constants.RadioMaster_Controller.Forward_Axis_Attenuation;
        Lateral_Speed_Attenuation = Constants.RadioMaster_Controller.Lateral_Axis_Attenuation;
        Yaw_Attenuation = Constants.RadioMaster_Controller.Yaw_Axis_Attenuation;

        intakeButton = new JoystickAnalogButton(m_driverController, Constants.RadioMaster_Controller.SE_Axis, -1.0, -0.5);

        liftToShootingButton = new JoystickAnalogButton(m_driverController, Constants.RadioMaster_Controller.SF_Axis, 0.75, 1.0);
        lowerToIntakeButton = new JoystickAnalogButton(m_driverController, Constants.RadioMaster_Controller.SF_Axis, -1.0, -0.5);
        shooterLowButton = new JoystickButton(m_driverController, Constants.RadioMaster_Controller.SH_Momentary);
        shooterHighButton = new JoystickButton(m_driverController, Constants.RadioMaster_Controller.SC_Button);
        alignWithTargetButton = new JoystickButton(m_driverController, Constants.RadioMaster_Controller.SB3_Axis);// Button should be changed

        climbCommand = new Climb(m_armBar, m_driverController, m_gamepad);
        abortClimbCommand = new AbortClimb(climbCommand, m_armBar) ;

        controllerFound = true;

        // game pad
        climbButton = new JoystickButton(m_gamepad, Constants.Logitech_F310_Controller.Button_X);
        abortClimbButton = new JoystickButton(m_gamepad, Constants.Logitech_F310_Controller.Button_B);
        ejectButton = new JoystickButton(m_gamepad, Constants.Logitech_F310_Controller.Left_Bumper);
        switchCameraButton = new JoystickAnalogButton(m_gamepad, Constants.Fight_Stick.Right_Trigger, 0.5, 1.0);

        climbButton.whenPressed(climbCommand);
        abortClimbButton.whenPressed(abortClimbCommand) ;
        ejectButton.whenPressed(startEjectingCommand).whenReleased(stopEjectingCommand);
        switchCameraButton.whenPressed(switchCameraCommand);

        if (controllerFound) {
            intakeButton.whenPressed(startIntakeCommand).whenReleased(stopIntakeCommand);
            // liftToShootingButton.whenPressed(shootingPositionCommand);
            lowerToIntakeButton.whenPressed(intakePositionCommand);
            liftToShootingButton.whenPressed(shootingPositionCommand);
            shooterLowButton.whenPressed(startShootingLowCommand).whenReleased(stopShootingCommand);
            shooterHighButton.whenPressed(startShootingHighCommand).whenReleased(stopShootingCommand);
            // moveToTargetButton.whenPressed(moveToTargetCommand);
        }
    }


    private class InitPose implements Runnable {

        DriveSubsystem drive ;
        Pose2d pose ;

        public InitPose( DriveSubsystem d, Pose2d p) {
            drive = d ;
            pose = p ;
        }
        @Override
        public void run() {
            drive.resetOdometry(pose);
        }
    }



    public Command getAutonomousCommand() {
        String autoCommandName = NetworkTableInstance.getDefault().getEntry("autonomous/auto_command_name").getString("Simple Shoot and Back Up") ;
        System.out.println("Using autonomous commad " + autoCommandName) ;
        AutonomousCommandHolder commandHolder = autonomousCommands.get(autoCommandName) ;
        if ( commandHolder != null ) {
            m_robotDrive.resetOdometry(commandHolder.initialPose);
            return commandHolder.autonomousCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false)); 
        } else {
            return new PrintCommand("Invalid autonomous command name").andThen(() -> m_robotDrive.drive(0, 0, 0, false));
        }
    }



    private void createAutonomousCommands() {

        autonomousCommands = new HashMap<String, AutonomousCommandHolder>() ;

        // Create list of know cargo locations
        ArrayList<Translation2d> cargoPoints = new ArrayList<Translation2d>() ;
        cargoPoints.add( new Translation2d(5.0, 6.2)) ;
        cargoPoints.add( new Translation2d(5.16, 1.9)) ;
        cargoPoints.add( new Translation2d(7.65, 0.30)) ;
        cargoPoints.add( new Translation2d(1.2, 1.2)) ;

        AutonomousCommandHolder commandHolder ;
        PathPlannerTrajectory trajectory ;
        Pose2d initialPose ;
        PPMecanumControllerCommand pathCommand ;
        Command autoCommand ;
        ProfiledPIDController thetaController ;

        ///////////////////////////////////////////////////////
        // Create One plus Two Low Goal command
        ///////////////////////////////////////////////////////

        thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints) ;
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        trajectory = trajectories.get("Two_Ball_Low_Goal");
        pathCommand = new PPMecanumControllerCommand(
                trajectory,
                m_robotDrive::getPose,
                DriveConstants.kDriveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                AutoConstants.kMaxSpeedMetersPerSecond,
                m_robotDrive::setWheelSpeeds,
                cargoPoints ,
                m_robotDrive ,
                m_vision
                );

        initialPose = new Pose2d(trajectory.getInitialPose().getTranslation(), ((PathPlannerState) trajectory.getStates().get(0)).holonomicRotation);

        autoCommand = new SequentialCommandGroup(
                // Shoot, Follow Path (2 balls) With Vision, Shoot
                new StartShootingLow(m_shooterIndex).withTimeout(Constants.AutoConstants.AutoShootingTimeout),
                new StopShooting(m_shooterIndex),
                new GotoIntakePosition(m_shooterIndex).andThen(new WaitCommand(Constants.AutoConstants.AutoIndexLowerTimeout)), 
                new StartIntake(m_shooterIndex, m_intake),
                pathCommand,
//                new StopIntake(m_intake, m_shooterIndex),
                new GotoShootingPosition(m_shooterIndex).andThen(new WaitCommand(Constants.AutoConstants.AutoIndexRaiseTimeout)),
                new StopIntake(m_intake, m_shooterIndex),
                new StartShootingLow(m_shooterIndex).withTimeout(Constants.AutoConstants.AutoShootingTimeout),
                new StopShooting(m_shooterIndex)
        );

        commandHolder = new AutonomousCommandHolder() ;
        commandHolder.initialPose = initialPose ;
        commandHolder.autonomousCommand = autoCommand ;
        autonomousCommands.put("One Plus Two Low Goal", commandHolder) ;

        ///////////////////////////////////////////////////////
        // Create Left Side command
        ///////////////////////////////////////////////////////

        thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints) ;
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        trajectory = trajectories.get("One_Ball_Low_Goal");
        pathCommand = new PPMecanumControllerCommand(
                trajectory,
                m_robotDrive::getPose,
                DriveConstants.kDriveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                AutoConstants.kMaxSpeedMetersPerSecond,
                m_robotDrive::setWheelSpeeds,
                cargoPoints ,
                m_robotDrive ,
                m_vision
                );

        initialPose = new Pose2d(trajectory.getInitialPose().getTranslation(), ((PathPlannerState) trajectory.getStates().get(0)).holonomicRotation);

        autoCommand = new SequentialCommandGroup(
                // Shoot, Follow Path (1 ball) With Vision, Shoot
                new StartShootingLow(m_shooterIndex).withTimeout(Constants.AutoConstants.AutoShootingTimeout),
                new StopShooting(m_shooterIndex),
                new GotoIntakePosition(m_shooterIndex).andThen(new WaitCommand(Constants.AutoConstants.AutoIndexLowerTimeout)), 
                new StartIntake(m_shooterIndex, m_intake),
                pathCommand,
//                new StopIntake(m_intake, m_shooterIndex),
                new GotoShootingPosition(m_shooterIndex).andThen(new WaitCommand(Constants.AutoConstants.AutoIndexRaiseTimeout)),
                new StopIntake(m_intake, m_shooterIndex),
                new StartShootingLow(m_shooterIndex).withTimeout(Constants.AutoConstants.AutoShootingTimeout),
                new StopShooting(m_shooterIndex)
        );

        commandHolder = new AutonomousCommandHolder() ;
        commandHolder.initialPose = initialPose ;
        commandHolder.autonomousCommand = autoCommand ;
        autonomousCommands.put("Left Side", commandHolder) ;

        ///////////////////////////////////////////////////////
        // Create One plus Two Low Goal Human Station command
        ///////////////////////////////////////////////////////

        thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints) ;
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        trajectory = trajectories.get("Two_Ball_High_Goal");
        pathCommand = new PPMecanumControllerCommand(
                trajectory,
                m_robotDrive::getPose,
                DriveConstants.kDriveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                AutoConstants.kMaxSpeedMetersPerSecond,
                m_robotDrive::setWheelSpeeds,
                cargoPoints ,
                m_robotDrive ,
                m_vision
                );

        initialPose = new Pose2d(trajectory.getInitialPose().getTranslation(), ((PathPlannerState) trajectory.getStates().get(0)).holonomicRotation);

        autoCommand = new SequentialCommandGroup(
                // Shoot, Follow Path (2 balls) With Vision, Shoot
                new StartShootingLow(m_shooterIndex).withTimeout(Constants.AutoConstants.AutoShootingTimeout),
                new StopShooting(m_shooterIndex),
                new GotoIntakePosition(m_shooterIndex).andThen(new WaitCommand(Constants.AutoConstants.AutoIndexLowerTimeout)), 
                new StartIntake(m_shooterIndex, m_intake),
                pathCommand,
                new StopIntake(m_intake, m_shooterIndex),
                new GotoShootingPosition(m_shooterIndex).andThen(new WaitCommand(Constants.AutoConstants.AutoIndexRaiseTimeout)),
//                new StartShootingHigh(m_shooterIndex).withTimeout(Constants.AutoConstants.AutoShootingTimeout),
                new StartShootingLow(m_shooterIndex).withTimeout(Constants.AutoConstants.AutoShootingTimeout),
                new StopShooting(m_shooterIndex)
        );

        commandHolder = new AutonomousCommandHolder() ;
        commandHolder.initialPose = initialPose ;
        commandHolder.autonomousCommand = autoCommand ;
        autonomousCommands.put("Human Player Station", commandHolder) ;

        // ///////////////////////////////////////////////////////
        // // Create Five Ball
        // ///////////////////////////////////////////////////////

        thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints) ;
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        trajectory = trajectories.get("Two_Ball_Low_Goal");
        pathCommand = new PPMecanumControllerCommand(
                trajectory,
                m_robotDrive::getPose,
                DriveConstants.kDriveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                AutoConstants.kMaxSpeedMetersPerSecond,
                m_robotDrive::setWheelSpeeds,
                cargoPoints ,
                m_robotDrive ,
                m_vision
                );

        initialPose = new Pose2d(trajectory.getInitialPose().getTranslation(), ((PathPlannerState) trajectory.getStates().get(0)).holonomicRotation);

        PathPlannerTrajectory trajectory2 ;

        thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints) ;
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        trajectory2 = trajectories.get("Two_Ball_High_Goal");
        PPMecanumControllerCommand pathCommand2 = new PPMecanumControllerCommand(
                trajectory2,
                m_robotDrive::getPose,
                DriveConstants.kDriveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                AutoConstants.kMaxSpeedMetersPerSecond,
                m_robotDrive::setWheelSpeeds,
                cargoPoints ,
                m_robotDrive ,
                m_vision
                );

        autoCommand = new SequentialCommandGroup(
                // Shoot, Follow Path (2 balls) With Vision, Shoot
                new StartShootingLow(m_shooterIndex).withTimeout(Constants.AutoConstants.AutoShootingTimeout),
                new StopShooting(m_shooterIndex),
                new GotoIntakePosition(m_shooterIndex).andThen(new WaitCommand(Constants.AutoConstants.AutoIndexLowerTimeout)), 
                new StartIntake(m_shooterIndex, m_intake),
                pathCommand,
                new GotoShootingPosition(m_shooterIndex).andThen(new WaitCommand(Constants.AutoConstants.AutoIndexRaiseTimeout)),
                new StopIntake(m_intake, m_shooterIndex),
                new StartShootingLow(m_shooterIndex).withTimeout(Constants.AutoConstants.AutoShootingTimeout),
                new StopShooting(m_shooterIndex) ,
                new GotoIntakePosition(m_shooterIndex).andThen(new WaitCommand(Constants.AutoConstants.AutoIndexLowerTimeout)), 
                new StartIntake(m_shooterIndex, m_intake),
                new InstantCommand( 
                    new InitPose(
                        m_robotDrive, 
                        new Pose2d(trajectory2.getInitialPose().getTranslation(), ((PathPlannerState) trajectory2.getStates().get(0)).holonomicRotation)
                        ), m_robotDrive ),
                pathCommand2 ,
                new GotoShootingPosition(m_shooterIndex).andThen(new WaitCommand(Constants.AutoConstants.AutoIndexRaiseTimeout)),
                new StopIntake(m_intake, m_shooterIndex),
                new StartShootingLow(m_shooterIndex).withTimeout(Constants.AutoConstants.AutoShootingTimeout), // make this high
                new StopShooting(m_shooterIndex)
        );

        commandHolder = new AutonomousCommandHolder() ;
        commandHolder.initialPose = initialPose ;
        commandHolder.autonomousCommand = autoCommand ;
        autonomousCommands.put("Five Ball Autonomous", commandHolder) ;



        ///////////////////////////////////////////////////////
        // Create Simple Shoot and Back Up Auto Command
        ///////////////////////////////////////////////////////
        thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints) ;
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        trajectory = trajectories.get("20_TarmacB1_Back_Off_Tarmac");
        pathCommand = new PPMecanumControllerCommand(
                trajectory,
                m_robotDrive::getPose,
                DriveConstants.kDriveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                AutoConstants.kMaxSpeedMetersPerSecond,
                m_robotDrive::setWheelSpeeds,
                cargoPoints ,
                m_robotDrive ,
                m_vision
                );

        initialPose = new Pose2d(trajectory.getInitialPose().getTranslation(), ((PathPlannerState) trajectory.getStates().get(0)).holonomicRotation);

        autoCommand = new SequentialCommandGroup(
            // Shoot, Back Up
            new StartShootingLow(m_shooterIndex).withTimeout(Constants.AutoConstants.AutoShootingTimeout),
            new StopShooting(m_shooterIndex),
            new GotoIntakePosition(m_shooterIndex),
            pathCommand
            );

        commandHolder = new AutonomousCommandHolder() ;
        commandHolder.initialPose = initialPose ;
        commandHolder.autonomousCommand = autoCommand ;
        autonomousCommands.put("Simple Shoot and Back Up", commandHolder) ;
        

        ///////////////////////////////////////////////////////
        // Create Vision Target Align Test Command
        ///////////////////////////////////////////////////////
        initialPose = new Pose2d();

        autoCommand = new SequentialCommandGroup(
            // Shoot, Back Up
            new TurnOnVisionLight(m_vision ).andThen(new WaitCommand(3.0)) ,
            new AlignWithTarget(m_vision, m_robotDrive) ,
            new TurnOffVisionLight(m_vision ) 
            );

        commandHolder = new AutonomousCommandHolder() ;
        commandHolder.initialPose = initialPose ;
        commandHolder.autonomousCommand = autoCommand ;
        autonomousCommands.put("Auto Align Test", commandHolder) ;


        ///////////////////////////////////////////////////////
        // Publish the names so the dashboard can see them
        ///////////////////////////////////////////////////////

        NetworkTableEntry commandListEntry = NetworkTableInstance.getDefault().getEntry("autonomous/command_list") ;
        String[] names = autonomousCommands.keySet().toArray(new String[0]) ;
        commandListEntry.forceSetStringArray(names);

    }
}
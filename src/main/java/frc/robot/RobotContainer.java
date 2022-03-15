// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;

import frc.robot.Constants.*;

import frc.robot.subsystems.*;
import frc.robot.commands.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import java.util.ArrayList;
import edu.wpi.first.networktables.*;

import com.pathplanner.lib.*;
import com.pathplanner.lib.PathPlannerTrajectory.*;

// import java.io.IOException;
// import java.nio.file.Path;
// import java.util.List;
// import frc.robot.Robot;

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
    // private final UnderGlow underGlow = new UnderGlow() ;

    // The driver's controller
    Joystick m_driverController;

    // Gamepad
    Joystick m_gamepad;

    private JoystickButton intakeButton;
    private JoystickButton liftToShootingButton;
    private JoystickButton liftToIntakeButton;
    private JoystickButton shooterButton;
    private JoystickButton climbButton;
    private JoystickButton ejectButton;
    private JoystickButton moveToTargetButton;
    private JoystickButton switchCameraButton;

    private int Forward_Speed_Axis;
    private int Lateral_Speed_Axis;
    private int Yaw_Axis;

    private double Forward_Speed_Attenuation = 0.5;
    private double Lateral_Speed_Attenuation = 0.5;
    private double Yaw_Attenuation = 0.5;

    private boolean initiateSequence = false;

    // PathPlannerTrajectory trajectory;
    ArrayList<PathPlannerTrajectory> trajectories;

    // Commands

    private final StartIntake startIntakeCommand = new StartIntake(m_shooterIndex, m_intake);
    private final StopIntake stopIntakeCommand = new StopIntake(m_intake, m_shooterIndex);

    private final GotoShootingPosition shootingPositionCommand = new GotoShootingPosition(m_shooterIndex);
    private final GotoIntakePosition intakePositionCommand = new GotoIntakePosition(m_shooterIndex);

    private final StartShooting startShootingCommand = new StartShooting(m_shooterIndex);
    private final StopShooting stopShootingCommand = new StopShooting(m_shooterIndex);

    private Climb climbCommand;

    private final StartEjecting startEjectingCommand = new StartEjecting(m_intake, m_shooterIndex);
    private final StopEjecting stopEjectingCommand = new StopEjecting(m_intake, m_shooterIndex);

    private final MoveToTarget moveToTargetCommand = new MoveToTarget(m_robotDrive);
    private final SwitchCamera switchCameraCommand = new SwitchCamera();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        // Configure the button bindings
        configureButtonBindings();

        // // Configure default commands
        // // Set the default drive command to split-stick arcade drive
        // trajectory = PathPlanner.loadPath("2_TarmacB2_to_BBallB", 1.0, 0.5);

        trajectories = new ArrayList<PathPlannerTrajectory>();
        trajectories.add(PathPlanner.loadPath("0_TarmacB1_to_BBallD",
                Constants.DriveConstants.pPMaxVel, Constants.DriveConstants.pPMaxAcc));
        trajectories.add(PathPlanner.loadPath("1_TarmacB1_to_BBallD_BBallC",
                Constants.DriveConstants.pPMaxVel, Constants.DriveConstants.pPMaxAcc));
        trajectories.add(PathPlanner.loadPath("2_TarmacB2_to_BBallB",
                Constants.DriveConstants.pPMaxVel, Constants.DriveConstants.pPMaxAcc));
        trajectories.add(PathPlanner.loadPath("3_TarmacB2_to_BBallB_BBallC",
                Constants.DriveConstants.pPMaxVel, Constants.DriveConstants.pPMaxAcc));
        trajectories.add(PathPlanner.loadPath("4_TarmacB2_to_BBallC",
                Constants.DriveConstants.pPMaxVel, Constants.DriveConstants.pPMaxAcc));
        trajectories.add(PathPlanner.loadPath("5_TarmacB2_to_BBallC_BBallB",
                Constants.DriveConstants.pPMaxVel, Constants.DriveConstants.pPMaxAcc));
        trajectories.add(PathPlanner.loadPath("6_TarmacB2_to_BBallC_BBallD",
                Constants.DriveConstants.pPMaxVel, Constants.DriveConstants.pPMaxAcc));
        trajectories.add(PathPlanner.loadPath("7_TarmacR1_to_RBallD",
                Constants.DriveConstants.pPMaxVel, Constants.DriveConstants.pPMaxAcc));
        trajectories.add(PathPlanner.loadPath("8_TarmacR1_to_RBallD_RBallE",
                Constants.DriveConstants.pPMaxVel, Constants.DriveConstants.pPMaxAcc));
        trajectories.add(PathPlanner.loadPath("9_TarmacR1_to_RBallE",
                Constants.DriveConstants.pPMaxVel, Constants.DriveConstants.pPMaxAcc));
        trajectories.add(PathPlanner.loadPath("10_TarmacR1_to_RBallE_RBallF",
                Constants.DriveConstants.pPMaxVel, Constants.DriveConstants.pPMaxAcc));
        trajectories.add(PathPlanner.loadPath("11_TarmacR2_to_RBallF",
                Constants.DriveConstants.pPMaxVel, Constants.DriveConstants.pPMaxAcc));
        trajectories.add(PathPlanner.loadPath("12_TarmacR2_to_RBallF_RBallE",
                Constants.DriveConstants.pPMaxVel, Constants.DriveConstants.pPMaxAcc));
        trajectories.add(PathPlanner.loadPath("13_Test_Constant_x",
                Constants.DriveConstants.pPMaxVel, Constants.DriveConstants.pPMaxAcc));
        trajectories.add(PathPlanner.loadPath("14_Test_Constant_y",
                Constants.DriveConstants.pPMaxVel, Constants.DriveConstants.pPMaxAcc));
        trajectories.add(PathPlanner.loadPath("15_Test_Diagonal",
                Constants.DriveConstants.pPMaxVel, Constants.DriveConstants.pPMaxAcc));
        trajectories.add(PathPlanner.loadPath("16_Test_Loop",
                Constants.DriveConstants.pPMaxVel, Constants.DriveConstants.pPMaxAcc));
        trajectories.add(PathPlanner.loadPath("17_Test_Sideways",
                Constants.DriveConstants.pPMaxVel, Constants.DriveConstants.pPMaxAcc));
        trajectories.add(PathPlanner.loadPath("18_Test_U_Shape_Dif_Angle",
                Constants.DriveConstants.pPMaxVel, Constants.DriveConstants.pPMaxAcc));
        trajectories.add(PathPlanner.loadPath("19_Test_U_Shape_Same_Angle",
                Constants.DriveConstants.pPMaxVel, Constants.DriveConstants.pPMaxAcc));
        // trajectories.add(PathPlanner.loadPath("20_Back_Off_Tarmac",
        // Constants.DriveConstants.pPMaxVel, Constants.DriveConstants.pPMaxAcc));
        trajectories.add(PathPlanner.loadPath("20_TarmacB1_Back_Off_Tarmac",
                Constants.DriveConstants.pPMaxVel, Constants.DriveConstants.pPMaxAcc));
        trajectories.add(PathPlanner.loadPath("21_TarmacB1_to_BBallD_Vicinity",
                Constants.DriveConstants.pPMaxVel, Constants.DriveConstants.pPMaxAcc));
        trajectories.add(PathPlanner.loadPath("22_TarmacR1_Back_Off_Tarmac",
                Constants.DriveConstants.pPMaxVel, Constants.DriveConstants.pPMaxAcc));
        trajectories.add(PathPlanner.loadPath("23_TarmacB2_to_BBallB_Vicinity",
                Constants.DriveConstants.pPMaxVel, Constants.DriveConstants.pPMaxAcc));
        trajectories.add(PathPlanner.loadPath("24_TarmacB2_to_BBallC_Vicinity",
                Constants.DriveConstants.pPMaxVel, Constants.DriveConstants.pPMaxAcc));
        trajectories.add(PathPlanner.loadPath("25_TarmacR1_to_RBallD_Vicinity",
                Constants.DriveConstants.pPMaxVel, Constants.DriveConstants.pPMaxAcc));
        trajectories.add(PathPlanner.loadPath("26_TarmacR1_to_RBallE_Vicinity",
                Constants.DriveConstants.pPMaxVel, Constants.DriveConstants.pPMaxAcc));
        trajectories.add(PathPlanner.loadPath("27_TarmacR2_to_RBallF_Vicinity",
                Constants.DriveConstants.pPMaxVel, Constants.DriveConstants.pPMaxAcc));


        m_armBar.neutralGripperA();;
        m_armBar.neutralGripperB();
        m_robotDrive.setDefaultCommand(
                // A split-stick arcade command, with forward/backward controlled by the left
                // hand, and turning controlled by the right.

                new RunCommand(() -> {
                    m_robotDrive.drive(
                            m_driverController.getRawAxis(Forward_Speed_Axis)
                                    * Forward_Speed_Attenuation,
                            m_driverController.getRawAxis(Lateral_Speed_Axis)
                                    * Lateral_Speed_Attenuation,
                            m_driverController.getRawAxis(Yaw_Axis) * Yaw_Attenuation,
                            false);
                }, m_robotDrive));

        
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * calling passing it to a {@link JoystickButton}.
     */
    private void configureButtonBindings() {

        // The driver's controller
        // m_driverController = new Joystick(2); // change
        // Gamepad
        // m_gamepad = new Joystick(OIConstants.gamepadPort);

        // for (int i = 0; i < 5; i++) {
        //     Joystick js = new Joystick(i);
        //     System.out.format("Joystick %d is %s%n", i, js.getName());
        //     if (js.isConnected()) {
        //         if (js.getName().contains("RadioMas")) {
        //             m_driverController = js;
        //             System.out.println("Found Radio Master Joystick");
        //         } else if (js.getName().contains("F310")) {
        //             m_driverController = js;
        //             System.out.println("Found F310 Joystick");
        //         } else if (js.getName().contains("FightStick")) {
        //             m_gamepad = js;
        //             System.out.println("Found FightStick Joystick");
        //         } else {
        //             System.out.println("*** Found un expected joystick");
        //         }
        //     } else {
        //         System.out.format("   Joystick %d is %s is not connected %n", i, js.getName());
        //     }
        // }

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

//        if (controllerType.equals("RadioMas TX16S Joystick")) {
            Forward_Speed_Axis = Constants.RadioMaster_Controller.Right_Gimbal_Y;
            Lateral_Speed_Axis = Constants.RadioMaster_Controller.Right_Gimbal_X;
            Yaw_Axis = Constants.RadioMaster_Controller.Left_Gimbal_X;

            Forward_Speed_Attenuation = Constants.RadioMaster_Controller.Forward_Axis_Attenuation;
            Lateral_Speed_Attenuation = Constants.RadioMaster_Controller.Lateral_Axis_Attenuation;
            Yaw_Attenuation = Constants.RadioMaster_Controller.Yaw_Axis_Attenuation;

            // these all need to be updated.

            // wpk need to test this out.

            intakeButton = new JoystickAnalogButton(m_driverController, Constants.RadioMaster_Controller.SE_Axis, -1.0,
                    -0.5);
            // intakeButton = new JoystickButton(m_driverController, 1 ) ;
            // Constants.Logitech_F310_Controller.Right_Bumper);

            liftToShootingButton = new JoystickAnalogButton(m_driverController,
                    Constants.RadioMaster_Controller.SF_Axis, 0.75, 1.0);
            liftToIntakeButton = new JoystickAnalogButton(m_driverController, Constants.RadioMaster_Controller.SF_Axis,
                    -1.0, -0.5);
            // liftToShootingButton = new JoystickButton(m_driverController, 0 );
            // liftToIntakeButton = new JoystickButton(m_driverController, 0 );

            shooterButton = new JoystickButton(m_driverController, Constants.RadioMaster_Controller.SH_Momentary);
            // climbButton = new JoystickButton(m_driverController,
            // Constants.Logitech_F310_Controller.Back_Button);
            // ejectButton = new JoystickButton(m_driverController,
            // Constants.Logitech_F310_Controller.Button_X);
            // moveToTargetButton = new JoystickButton(m_driverController,
            // Constants.Logitech_F310_Controller.Start_Button); ***what to do w/ this?
            controllerFound = true;

        // } else if (controllerType.equals("Controller (Gamepad F310)")) {

        //     Forward_Speed_Axis = Constants.Logitech_F310_Controller.Right_Stick_Y;
        //     Lateral_Speed_Axis = Constants.Logitech_F310_Controller.Right_Stick_X;
        //     Yaw_Axis = Constants.Logitech_F310_Controller.Left_Stick_X;
        //     Forward_Speed_Attenuation = -Constants.Logitech_F310_Controller.Forward_Axis_Attenuation;
        //     Lateral_Speed_Attenuation = -Constants.Logitech_F310_Controller.Lateral_Axis_Attenuation;
        //     Yaw_Attenuation = -Constants.Logitech_F310_Controller.Yaw_Axis_Attenuation;

        //     intakeButton = new JoystickButton(m_driverController,
        //             Constants.Logitech_F310_Controller.Right_Bumper);
        //     liftToShootingButton = new JoystickButton(m_driverController,
        //             Constants.Logitech_F310_Controller.Button_Y);
        //     liftToIntakeButton = new JoystickButton(m_driverController,
        //             Constants.Logitech_F310_Controller.Button_B);
        //     shooterButton = new JoystickButton(m_driverController,
        //             Constants.Logitech_F310_Controller.Button_A);
        //     // climbButton = new JoystickButton(m_driverController,
        //     // Constants.Logitech_F310_Controller.Back_Button);
        //     // ejectButton = new JoystickButton(m_driverController,
        //     // Constants.Logitech_F310_Controller.Button_X);
        //     moveToTargetButton = new JoystickButton(m_driverController,
        //             Constants.Logitech_F310_Controller.Start_Button);
        //     controllerFound = true;

        // } else if (controllerType.equals("Xbox Controller")) {

        //     // sometimes the logi-tech controller shows up this way when the X/D switch is
        //     // in the "X" position
        //     Forward_Speed_Axis = Constants.Xbox_Controller.Right_Stick_Y;
        //     Lateral_Speed_Axis = Constants.Xbox_Controller.Right_Stick_X;
        //     Yaw_Axis = Constants.Xbox_Controller.Left_Stick_X;
        //     Forward_Speed_Attenuation = Constants.Xbox_Controller.Forward_Axis_Attenuation;
        //     Lateral_Speed_Attenuation = Constants.Xbox_Controller.Lateral_Axis_Attenuation;
        //     Yaw_Attenuation = Constants.Xbox_Controller.Yaw_Axis_Attenuation;

        //     intakeButton = new JoystickButton(m_driverController, Constants.Xbox_Controller.Right_Bumper);
        //     liftToShootingButton = new JoystickButton(m_driverController,
        //             Constants.Xbox_Controller.Button_Y);
        //     liftToIntakeButton = new JoystickButton(m_driverController, Constants.Xbox_Controller.Button_B);
        //     shooterButton = new JoystickButton(m_driverController, Constants.Xbox_Controller.Button_A);
        //     // climbButton = new JoystickButton(m_driverController,
        //     // Constants.Xbox_Controller.Back_Button);
        //     // ejectButton = new JoystickButton(m_driverController,
        //     // Constants.Xbox_Controller.Button_X);
        //     moveToTargetButton = new JoystickButton(m_driverController, Constants.Xbox_Controller.Start_Button);
        //     controllerFound = true;

        // } else if (controllerType.equals("Logitech Dual Action")) {
        //     Forward_Speed_Axis = Constants.Logitech_Dual_Action.Right_Stick_Y;
        //     Lateral_Speed_Axis = Constants.Logitech_Dual_Action.Right_Stick_X;
        //     Yaw_Axis = Constants.Logitech_Dual_Action.Left_Stick_X;
        //     Forward_Speed_Attenuation = Constants.Logitech_Dual_Action.Forward_Axis_Attenuation;
        //     Lateral_Speed_Attenuation = Constants.Logitech_Dual_Action.Lateral_Axis_Attenuation;
        //     Yaw_Attenuation = Constants.Logitech_Dual_Action.Yaw_Axis_Attenuation;

        //     intakeButton = new JoystickButton(m_driverController,
        //             Constants.Logitech_Dual_Action.Right_Bumper);
        //     liftToShootingButton = new JoystickButton(m_driverController,
        //             Constants.Logitech_Dual_Action.Button_Y);
        //     liftToIntakeButton = new JoystickButton(m_driverController,
        //             Constants.Logitech_Dual_Action.Button_B);
        //     shooterButton = new JoystickButton(m_driverController, Constants.Logitech_Dual_Action.Button_A);
        //     // climbButton = new JoystickButton(m_driverController,
        //     // Constants.Logitech_Dual_Action.Back_Button);
        //     // ejectButton = new JoystickButton(m_driverController,
        //     // Constants.Logitech_Dual_Action.Button_X);
        //     moveToTargetButton = new JoystickButton(m_driverController,
        //             Constants.Logitech_Dual_Action.Start_Button);
        //     controllerFound = true;

        // } else {

        //     // no buttons will be configured and we'll crash shortly after. That's OK, we
        //     // want to crash so we can find this issue.
        // }

        // Drive at half speed when the right bumper is held
        // new JoystickButton(m_driverController,
        // Constants.OIConstants.halfSpeedButton);
        // .whenPressed(() -> m_robotDrive.setMaxOutput(0.5));
        // .whenReleased(() -> m_robotDrive.setMaxOutput(1));

        // climbCommand = new Climb(m_armBar, m_robotDrive, m_driverController,
        // m_gamepad,
        // Forward_Speed_Axis, Lateral_Speed_Axis, Yaw_Axis,
        // Forward_Speed_Attenuation, Lateral_Speed_Attenuation, Yaw_Attenuation
        // ) ;
        climbCommand = new Climb(m_armBar, m_driverController, m_gamepad);
        // Forward_Speed_Axis, Lateral_Speed_Axis, Yaw_Axis,
        // Forward_Speed_Attenuation, Lateral_Speed_Attenuation, Yaw_Attenuation
        // ) ;

//        if (m_gamepad != null && !m_gamepad.getName().equals("")) {

            // are these the right constants?????

            // game pad
            System.out.println("setting up climb button") ;
            climbButton = new JoystickButton(m_gamepad, Constants.Fight_Stick.Button_Y);
            ejectButton = new JoystickAnalogButton(m_gamepad, Constants.Fight_Stick.Left_Trigger, 0.5, 1.0);
            switchCameraButton = new JoystickAnalogButton(m_gamepad, Constants.Fight_Stick.Right_Trigger, 0.5, 1.0);

            climbButton.whenPressed(climbCommand);
            ejectButton.whenPressed(startEjectingCommand).whenReleased(stopEjectingCommand);
            switchCameraButton.whenPressed(switchCameraCommand);
        // } else {

        // }

        if (controllerFound) {
            intakeButton.whenPressed(startIntakeCommand).whenReleased(stopIntakeCommand);
            // liftToShootingButton.whenPressed(shootingPositionCommand);
            liftToIntakeButton.whenPressed(intakePositionCommand);
            liftToShootingButton.whenPressed(shootingPositionCommand);
            shooterButton.whenPressed(startShootingCommand).whenReleased(stopShootingCommand);
            // moveToTargetButton.whenPressed(moveToTargetCommand);
        }

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */

    public class AutonomousSequential extends SequentialCommandGroup {
        /**
         * Creates a new AutonomousSequential.
         */
    }

    public Command getAutonomousCommand() {

        PathPlannerTrajectory trajectory = trajectories.get(20);
        // PathPlannerTrajectory trajectory =
        // trajectories.get((int)NetworkTableInstance.getDefault().getEntry("pathSelected").getDouble(20.0));

        // use network tables to get value from driver station

        PPMecanumControllerCommand pathCommand = new PPMecanumControllerCommand(
                trajectory,
                m_robotDrive::getPose,
                DriveConstants.kDriveKinematics,
                new PIDController(AutoConstants.kPXController, 0, 0),
                new PIDController(AutoConstants.kPYController, 0, 0),
                new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0,
                        AutoConstants.kThetaControllerConstraints),
                AutoConstants.kMaxSpeedMetersPerSecond,
                m_robotDrive::setWheelSpeeds,
                m_robotDrive);

        // Reset odometry to the starting pose of the trajectory.
        Pose2d temp = trajectory.getInitialPose();
        PathPlannerState s = (PathPlannerState) trajectory.getStates().get(0);
        Pose2d temp2 = new Pose2d(temp.getTranslation(), s.holonomicRotation);
        m_robotDrive.resetOdometry(temp2);

        // if (initiateSequence == false) {
        // return pathCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
        // } else {

        // SequentialCommandGroup autoCommand = new SequentialCommandGroup(
        // // new DriveBackwards(m_robotDrive),
        // // new GotoShootingPosition(m_shooterIndex), // Should already be up
        // new StartShooting(m_shooterIndex).withTimeout(1.0),
        // new StopShooting(m_shooterIndex),
        // new GotoIntakePosition(m_shooterIndex),
        // new StartIntake(m_shooterIndex, m_intake),
        // pathCommand,
        // new StopIntake(m_intake, m_shooterIndex),
        // new GotoShootingPosition(m_shooterIndex),
        // new StartShooting(m_shooterIndex).withTimeout(1.0),
        // new StopShooting(m_shooterIndex),
        // new GotoIntakePosition(m_shooterIndex));

        // SequentialCommandGroup autoCommand = new SequentialCommandGroup(
        // // new DriveBackwards(m_robotDrive),
        // // new GotoShootingPosition(m_shooterIndex), // Should already be up
        // new
        // StartShooting(m_shooterIndex).withTimeout(Constants.AutoConstants.AutoShootingTimeout),
        // new StopShooting(m_shooterIndex),
        // new GotoIntakePosition(m_shooterIndex),
        // new StartIntake(m_shooterIndex, m_intake),
        // pathCommand,
        // new MoveToTarget(m_robotDrive),
        // new StopIntake(m_intake, m_shooterIndex),
        // //new DriveToPose(temp2, m_robotDrive),
        // new GotoShootingPosition(m_shooterIndex),
        // new
        // StartShooting(m_shooterIndex).withTimeout(Constants.AutoConstants.AutoShootingTimeout),
        // new StopShooting(m_shooterIndex),
        // new GotoIntakePosition(m_shooterIndex));

        SequentialCommandGroup autoCommand = new SequentialCommandGroup(
                new StartShooting(m_shooterIndex).withTimeout(Constants.AutoConstants.AutoShootingTimeout),
                new StopShooting(m_shooterIndex),
                new GotoIntakePosition(m_shooterIndex),
                // new StartIntake(m_shooterIndex, m_intake),
                pathCommand
        // new MoveToTarget(m_robotDrive),
        // new StopIntake(m_intake, m_shooterIndex)
        // new DriveToPose(temp2, m_robotDrive),
        // new GotoShootingPosition(m_shooterIndex),
        // new
        // StartShooting(m_shooterIndex).withTimeout(Constants.AutoConstants.AutoShootingTimeout),
        // new StopShooting(m_shooterIndex),
        // new GotoIntakePosition(m_shooterIndex)
        );

        // Run path following command, then stop at the end.
        return autoCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
        // }
    }
}
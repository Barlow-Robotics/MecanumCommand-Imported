// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.SerialPort;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants {
        public static final int ID_frontLeftMotor = 4;
        public static final int ID_backLeftMotor = 1;
        public static final int ID_frontRightMotor = 5;
        public static final int ID_backRightMotor = 2;

        //public static final double countsPerRevolution = 8192.0;
        public static final double circumferenceOfWheel = 6.0 * Math.PI;
        public static final double distanceGoal = 120.0;

        public static final double kTrackWidth = 0.5;
        // Distance between centers of right and left wheels on robot
        public static final double kWheelBase = 0.7;
        // Distance between centers of front and back wheels on robot

        public static final MecanumDriveKinematics kDriveKinematics 
            = new MecanumDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2), 
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
                );

        public static final int kEncoderCPR = 1024;
        public static final double kWheelDiameterMeters = 0.15;
        public static final double kEncoderDistancePerPulse =
                // Assumes the encoders are directly mounted on the wheel shafts
                (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or
        // theoretically
        // for *your* robot's drive.
        // The RobotPy Characterization Toolsuite provides a convenient tool for
        // obtaining these
        // values for your robot.
        public static final SimpleMotorFeedforward kFeedforward = new SimpleMotorFeedforward(1, 0.8, 0.15);

        // Example value only - as above, this must be tuned for your drive!
        public static final double kPFrontLeftVel = 0.5;
        public static final double kPRearLeftVel = 0.5;
        public static final double kPFrontRightVel = 0.5;
        public static final double kPRearRightVel = 0.5;

        // Drivetrain
        public static final double drivetrainMinPower = 0.05;
        public static final double drivetrainMaxPower = 1.0;
        public static final double manualVoltageRampingConstant = 0.21;
        public static final double closedVoltageRampingConstant = 0.21;

        public static final int PID_id = 0;
        public static final double PID_Period = 1.0 / 20.0;
        public static final double DrivetrainKf = 1.8; // 0.1797
        public static final double DrivetrainkP = 0.02;

        public static final double unitsPerRotation = 1024;
        public static final double RPMsToUnitsPerHundredMilliseconds = 1.0 / 600.0;
        public static final double desiredRPMsForDrive = 560.0;
        public static final double maxDriveVelocity = 6000.0;
        public static final double VelocityInputConversionFactor = desiredRPMsForDrive * unitsPerRotation
                * RPMsToUnitsPerHundredMilliseconds;

        public static final int encoderTimeout = 30;
        public static final int mainFeedbackLoop = 0;

        public static final double autoBackingDistance = 3.5; // 3.5 rotations of the wheel ~ 65"
        public static final double pathFollowingThreshold = 20;
        public static final int autonomousDriveTime = 2500;
        public static final double autonomousDriveSpeed = 0.7;
        public static final double autonomousTurnRate = 0.7;

        public static final double speedConstantForBallChase = 0.3;
        public static final double maxAngleChangeForAlignFinish = 0.5;
        public static final double maxAngleDifferenceBetweenNavXAndVision = 0.01;
        public static final double alignTimeoutTime = 1000;
        public static final double alignMemorySize = 3;
    }




    public static final class OIConstants {
        public static final int kDriverControllerPort = 1;
        public static final int halfSpeedButton = 2;
        public static final int leftYAxis = 1;
        public static final int leftXAxis = 0;
        public static final int rightXAxis = 4;
        public static final int rightYAxis = 5;

        public static final String LogitechF310Name = "Controller (Gamepad F310)" ;
    }

    // For Brian's Controller

    // public static final class OIConstants {
    //     public static final int kDriverControllerPort = 1;
    //     public static final int halfSpeedButton = 2;
    //     public static final int leftYAxis = 0;
    //     public static final int leftXAxis = 1;
    //     public static final int rightXAxis = 3;
    //     public static final int rightYAxis = 5;
    // }



    public static final class UnderGlowConstants {
        public static final SerialPort.Port port = SerialPort.Port.kUSB1 ;
        public static final int BlueAliance = 1 ;
        public static final int RedAliance = 2 ;
        public static final int NeonGreen = 3 ;
    }

    public static final class IntakeConstants {
        public static final int ID_intakeMotor = 3;
        public static final double intakeMotorSpeed = 0.3;
        public static final int mainFeedbackLoop = 0;
        public static final int encoderTimeout = 0;
        public static final int Extend_Solenoid = 0;
        public static final int Retract_Solenoid = 1;
        public static final double closedVoltageRampingConstant = 0;
        public static final double manualVoltageRampingConstant = 0;
        public static final double DrivetrainKf = 0;
        public static final double DrivetrainkP = 0;
        public static final int PID_id = 0;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 0.5;
        public static final double kPYController = 0.5;
        public static final double kPThetaController = 0.5;

        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class ArmBarConstants {
        //these are all temp values!!!
        
        public static final int ID_armBarMotor = 10;
        
        public static final int ID_hallEffectsA1 = 11;
        public static final int ID_hallEffectsA2 = 12;
        public static final int ID_hallEffectsB1 = 13;
        public static final int ID_hallEffectsB2 = 14;

        public static final int ID_solenoidA1 = 15;
        public static final int ID_solenoidA2 = 16;
        public static final int ID_solenoidB1 = 17;
        public static final int ID_solenoidB2 = 18;

        public static final int unitsPerRotation = 1024;
        public static final double gearRatio = 48/1;

        public static final double armBarMotorSpeed = 0.0;

        public static final double manualVoltageRampingConstant = 0.21;
        public static final double closedVoltageRampingConstant = 0.21;

        public static final int encoderTimeout = 30;
        public static final int mainFeedbackLoop = 0;

        public static final int PID_id = 0;

        public static final double DrivetrainKf = 1.8;
        public static final double DrivetrainkP = 0.02;

        public static final double firstRotationAngle = 90;
        public static final double consistentRotationAngle = 90;
        


    }

    public final class Logitech_F310_Controller {

        // Constants for Axes
        public static final int Left_Stick_X = 0 ;
        public static final int Left_Stick_Y = 1 ;
        public static final int Left_Trigger = 2 ;
        public static final int Right_Trigger = 3 ;
        public static final int Right_Stick_X = 4 ;
        public static final int Right_Stick_Y = 5 ;

        // Constants for buttons
        public static final int Button_A = 1 ;
        public static final int Button_B = 2 ;
        public static final int Button_X = 3 ;
        public static final int Button_Y = 4 ;
        public static final int Left_Bumper = 5 ;
        public static final int Right_Bumper = 6 ;
        public static final int Back_Button = 7 ;
        public static final int Start_Button = 8 ;
        public static final int Left_Stick = 9 ;
        public static final int Right_Stick = 10 ;
        
    }

    public static final class ShooterConstants {
        public static final int ID_shooterMotor = 7;
        // public static final int ID_receiverMotor = 8;
        // public static final double receiverMotorVelocity = 0.3;
        public static final double shooterMotorVelocity = 0.3;
    }


    public static final String Feeder = null;
}

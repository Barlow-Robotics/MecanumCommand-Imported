// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.FilenameFilter;

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

    public static final double RadiansToDegrees = 180.0 / Math.PI;
    public static final double DegreesToRadians = Math.PI / 180.0;
    public static final double Inches_Per_Foot = 12.0;
    public static final double InchesToMeters = 0.0254;

    public static final class DriveConstants {
        public static final int ID_frontLeftMotor = 4;
        public static final int ID_backLeftMotor = 5;
        public static final int ID_frontRightMotor = 1;
        public static final int ID_backRightMotor = 2;

        // public static final double countsPerRevolution = 8192.0;
        public static final double circumferenceOfWheel = 6.0 * Math.PI;
        public static final double distanceGoal = 120.0;

        public static final double kTrackWidth = 0.5;
        // Distance between centers of right and left wheels on robot
        public static final double kWheelBase = 0.7;
        // Distance between centers of front and back wheels on robot

        public static final MecanumDriveKinematics kDriveKinematics = new MecanumDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

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
        public static final double kPFrontLeftVel = 0.75;
        public static final double kPRearLeftVel = 0.75;
        public static final double kPFrontRightVel = 0.75;
        public static final double kPRearRightVel = 0.75;

        // Drivetrain
        public static final double GearRatio = 48 / 1;

        public static final double drivetrainMinPower = 0.05;
        public static final double drivetrainMaxPower = 1.0;
        public static final double manualVoltageRampingConstant = 0.21;
        public static final double closedVoltageRampingConstant = 0.21;

        public static final int PID_id = 0;
        public static final double PID_Period = 1.0 / 20.0;
        public static final double DrivetrainKf = 1.8; // 0.1797
        public static final double DrivetrainkP = 0.02;

        public static final double UnitsPerRotation = 1024;
        public static final double RPMsToUnitsPerHundredMilliseconds = 1.0 / 600.0;
        public static final double DesiredRPMsForDrive = 560.0;
        public static final double MaxDriveVelocity = 6000.0;
        public static final double VelocityInputConversionFactor = DesiredRPMsForDrive * UnitsPerRotation
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

        public static final double kFrontLeft_x = 3; // feet (change)
        public static final double kFrontLeft_y = 3; // feet (change)
        public static final double kFrontRight_x = 3; // feet (change)
        public static final double kFrontRight_y = 3; // feet (change)
        public static final double kBackLeft_x = 3; // feet (change)
        public static final double kBackLeft_y = 3; // feet (change)
        public static final double kBackRight_x = 3; // feet (change)
        public static final double kBackRight_y = 3; // feet (change)

        // wpk need to measure and update.
        public static final int Counts_Per_Revolution = 21300;
        public static final double Wheel_Diameter = 6.0;
        public static final double Inches_Per_Revolution = Math.PI * Wheel_Diameter;
        public static final double Meters_Per_Revolution = Inches_Per_Revolution * InchesToMeters;
        public static final double Meters_Per_Count = Meters_Per_Revolution / Counts_Per_Revolution;

    }


    public static final class OIConstants {
        public static final int kDriverControllerPort = 1;
        public static final int halfSpeedButton = 2;
        public static final int leftYAxis = 1;
        public static final int leftXAxis = 0;
        public static final int rightXAxis = 4;
        public static final int rightYAxis = 5;

        public static final String LogitechF310Name = "Controller (Gamepad F310)";
    }


    public final class Logitech_F310_Controller {

        // Constants for Axes
        public static final int Left_Stick_X = 0;
        public static final int Left_Stick_Y = 1;
        public static final int Left_Trigger = 2;
        public static final int Right_Trigger = 3;
        public static final int Right_Stick_X = 4;
        public static final int Right_Stick_Y = 5;

        // Constants for buttons
        public static final int Button_A = 2;
        public static final int Button_B = 3;
        public static final int Button_X = 1;
        public static final int Button_Y = 4;
        public static final int Left_Bumper = 5;
        public static final int Right_Bumper = 6;
        public static final int Back_Button = 7;
        public static final int Start_Button = 8;
        public static final int Left_Stick = 9;
        public static final int Right_Stick = 10;

    }



    // For Brian's Controller

    // public static final class OIConstants {
    // public static final int kDriverControllerPort = 1;
    // public static final int halfSpeedButton = 2;
    // public static final int leftYAxis = 0;
    // public static final int leftXAxis = 1;
    // public static final int rightXAxis = 3;
    // public static final int rightYAxis = 5;
    // }

    public static final class UnderGlowConstants {
        public static final SerialPort.Port port = SerialPort.Port.kUSB1;
        public static final int BlueAliance = 1;
        public static final int RedAliance = 2;
        public static final int NeonGreen = 3;
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

        public static final int ID_ArmBarMotor = 10;

        public static final int ID_HallEffectsA1 = 1;
        public static final int ID_HallEffectsA2 = 2;
        public static final int ID_HallEffectsB1 = 3;
        public static final int ID_HallEffectsB2 = 4;

        public static final int ID_SolenoidA1 = 5;
        public static final int ID_SolenoidA2 = 6;
        public static final int ID_SolenoidB1 = 7;
        public static final int ID_SolenoidB2 = 8;

        public static final int UnitsPerMotorRotation = 2048;
        public static final double GearboxGearRatio = 100 / 1; // farther gear to axel gear
        public static final double ChainGearRatio = 12 / 12; // (or 15/12) upper gear to lower gear
        public static final double UnitsPerArmRotation = UnitsPerMotorRotation * GearboxGearRatio * ChainGearRatio ;
        public static final double UnitsPerArmDegree = UnitsPerArmRotation / 360.0 ;

        public static final double AngleToNextArm = 33 ;
        public static final double MidBarRotationAngle = 90;
        public static final double HighBarRotationAngle = MidBarRotationAngle + 180 + AngleToNextArm ;
        public static final double TraverseBarRotationAngle = HighBarRotationAngle + 180 ;
        public static final double FinalRestingAngle = TraverseBarRotationAngle + ( 90 - AngleToNextArm) ; // angle to hang straight down.
        public static final double AngleTolerance = 1;
//        public static final double ConsistentRotationAngleTolerance = 6;

        public static final double DesiredArmVelocity = 2* (180 - 2*AngleToNextArm) / (Math.PI * Math.sqrt(14.25/386)) ; // degrees persecond
        public static final double CruiseVelocity = (UnitsPerArmDegree * DesiredArmVelocity ) / 10.0 ; // divided by 10 because Falcon Velocities are in 100 mSec units
        public static final double MaxAcceleration = CruiseVelocity * 0.75 ;
        public static final int AccelerationSmoothing = 2 ;

        public static final double SlowArmVelocity = 10 ;
        public static final double SlowCruiseVelocity = (UnitsPerArmDegree * SlowArmVelocity ) / 10.0 ; // divided by 10 because Falcon Velocities are in 100 mSec units
        public static final double SlowMaxAcceleration = SlowCruiseVelocity * 0.75 ;


        public static final int Position_PID_id = 0;
        public static final double Position_kF = 0.4;  // wpk need to give this some thought
        public static final double Position_kP = 0.1;  // wpk need to give this some thought
        public static final double Position_kD = 0.0;  // wpk need to give this some thought
        public static final double Position_kI = 0.0;  // wpk need to give this some thought


        public static final double ArmBarMotorSpeed = 0.1;

        public static final double manualVoltageRampingConstant = 0.05;
        public static final double closedVoltageRampingConstant = 0.05;


        public static final int Velocity_PID_id = 1;
        public static final double Velocity_kF = 0.0485;
        public static final double Velocity_kP = 0.01;
        public static final double Velocity_kD = 0.00;

    }


    public static final class IntakeConstants {
        public static final int ID_IntakeMotor = 3;
        public static final double IntakeMotorSpeed = 250.0;
        public static final int mainFeedbackLoop = 0;
        public static final int encoderTimeout = 0;
        public static final int Extend_Solenoid = 0;
        public static final int Retract_Solenoid = 1;

        public static final double closedVoltageRampingConstant = 0.1;
        public static final double manualVoltageRampingConstant = 0.1;
        public static final double kF = 1.0;
        public static final double kP = 0.0;

        public static final int PID_id = 0;
    }

    public static final class ShooterConstants {
        public static final int ID_ShooterMotor = 7;
        public static final int ID_FlyWheelMotor = 8;
        public static final double BeltMotorIntakeVelocity = 200;
        public static final double BeltMotorShootingVelocity = -5700;
        public static final double FlyWheelMotorIntakeVelocity = -500;
        public static final double FlyWheelMotorShootingVelocity = 850;
        public static final double FlyWheelMinShootingSpeed = 18000;

        public static final double BeltMotor_kF = 1.0;
        public static final double BeltMotor_kP = 0.0;

        public static final double FlyWheelMotor_kP = 0.0;
        public static final double FlyWheelMotor_kF = 1.0;

        public static final class Lift {
            public static final int ID_Motor = 11;
            public static final int MotorUnitsPerRotation = 2048;
            public static final double GearboxGearRatio = 48 / 1; 
            public static final double ChainGearRatio = 18.0 / 15.0;
            public static final double UnitsPerRotation = MotorUnitsPerRotation * GearboxGearRatio * ChainGearRatio ;
            public static final double UnitsPerDegree = (UnitsPerRotation) / 360.0 ;

            // public static final double MotorIntakeAngle = 0.0 * UnitsPerDegree ; 
            public static final double MotorIntakeAngle = 0.0 * UnitsPerDegree ; // wpk temporary for testing
            public static final double MotorShootingAngle = 58.0 * UnitsPerDegree ; 
            public static final double MotorStartingAngle = 0.0 * UnitsPerDegree ; 

            public static final double DesiredArmVelocity = 60 ; // degrees persecond
            public static final double CruiseVelocity = (UnitsPerDegree * DesiredArmVelocity ) / 10.0 ; // divided by 10 because Falcon Velocities are in 100 mSec units
            public static final double MaxAcceleration = CruiseVelocity / 2.0 ;
            public static final int AccelerationSmoothing = 2 ;

            //solenoid piston stuff - temp values
            public static final int ID_Extend_Solenoid = 1;    
            public static final int ID_Retract_Solenoid = 0;

            public static final int PID_ID = 0 ;
            public static final double kF = 0.4;
            public static final double kP = 0.2;
            public static final double kD = 0.0;
            public static final double kI = 0.0;
   
        }


    }

    public static final String Feeder = null;
}

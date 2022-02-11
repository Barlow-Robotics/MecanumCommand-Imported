// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList ;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;

//import edu.wpi.first.wpilibj.drive.MecanumDrive;
import frc.robot.subsystems.MecanumDrive;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PIDConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.ErrorCode;
//import com.kauailabs.navx.frc.AHRS; //where 2 get this library ?
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
//import edu.wpi.first.wpilibj.SpeedControllerGroup;
import frc.robot.Constants;
// import frc.robot.sim.PhysicsSim; //where 2 get this library ?
// import edu.wpi.first.wpilibj.RobotBase;
// import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.Robot;
import edu.wpi.first.networktables.*;



public class DriveSubsystem extends SubsystemBase {

    WPI_TalonFX m_frontLeft;
    WPI_TalonFX m_backLeft;
    WPI_TalonFX m_frontRight;
    WPI_TalonFX m_backRight;

    MecanumDriveOdometry odometry;

    public final MecanumDrive m_drive;

    // Constructs a new SwerveControllerCommand that when executed will follow the provided trajectory
    // public SwerveControllerCommand(
    //     Trajectory trajectory,
    //     Supplier<Pose2d> pose,
    //     SwerveDriveKinematics kinematics,
    //     PIDController xController,
    //     PIDController yController,
    //     ProfiledPIDController thetaController,
    //     Consumer<SwerveModuleState[]> outputModuleStates,
    //     Subsystem... requirements
    // );

    // The gyro sensor
    private final Gyro m_gyro = new ADXRS450_Gyro();

    // Mechanum kinematics setup (wheel position in relation to centre)
    static Translation2d m_frontLeftLocation = new Translation2d(Units.feetToMeters(DriveConstants.kFrontLeft_x), Units.feetToMeters(DriveConstants.kFrontLeft_y));
    static Translation2d m_frontRightLocation = new Translation2d(Units.feetToMeters(DriveConstants.kFrontRight_x), Units.feetToMeters(DriveConstants.kFrontRight_y));
    static Translation2d m_backLeftLocation = new Translation2d(Units.feetToMeters(DriveConstants.kBackLeft_x), Units.feetToMeters(DriveConstants.kBackLeft_y));
    static Translation2d m_backRightLocation = new Translation2d(Units.feetToMeters(DriveConstants.kBackRight_x), Units.feetToMeters(DriveConstants.kBackRight_y));

    static MecanumDriveKinematics kinematics = new MecanumDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

    // Odometry class for tracking robot pose
    public MecanumDriveOdometry m_odometry = new MecanumDriveOdometry(DriveConstants.kDriveKinematics, m_gyro.getRotation2d());

    // Adjust based on how well the robot tracks the trajectory. NOT TUNED
    PIDController xController = new PIDController(1, 0, 0);
    PIDController yController = new PIDController(1.1, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(1.1, 0, 0,  new TrapezoidProfile.Constraints(Math.PI, Math.PI));

    // PID for each wheel CHARACTERIZATION GETS YOU PRETTY CLOSE BUT MAKE SURE TO TUNE.. i think
    PIDController frontLeftPID = new PIDController(PIDConstants.fl_kP, 0, 0);
    PIDController frontRightPID = new PIDController(PIDConstants.fr_kP, 0, 0);
    PIDController backLeftPID = new PIDController(PIDConstants.bl_kP, 0, 0);
    PIDController backRightPID = new PIDController(PIDConstants.br_kP, 0, 0);

    // Robot pose object
    Pose2d pose = new Pose2d();

    /**
     * Gets gyro heading from -180 to 180. CCW Positive
     * @return Gyro Heading [-180, 180]
     */
    public double getGyroHeading(){
        return Math.IEEEremainder(m_gyro.getAngle(), 360) * -1;
    }

    ArrayList<WPI_TalonFX> motors = new ArrayList<WPI_TalonFX>() ;
    public Object requirements;
    

    /** Creates a new DriveSubsystem. */
    public DriveSubsystem() {

        m_frontLeft = new WPI_TalonFX(DriveConstants.ID_frontLeftMotor);
        m_frontRight = new WPI_TalonFX(DriveConstants.ID_frontRightMotor);
        m_backLeft = new WPI_TalonFX(DriveConstants.ID_backLeftMotor);
        m_backRight = new WPI_TalonFX(DriveConstants.ID_backRightMotor);

        motors.add(m_frontLeft);
        motors.add(m_backLeft);
        motors.add(m_frontRight);
        motors.add(m_backRight);

        // // These are OK forward/back, but not good with rot
        // m_backLeft.setInverted(true);
        // m_frontLeft.setInverted(false);
        // m_backRight.setInverted(true);
        // m_frontRight.setInverted(false);

        m_backLeft.setInverted(false);
        m_frontLeft.setInverted(false);
        m_backRight.setInverted(true);
        m_frontRight.setInverted(true);

        setMotorConfig(m_backLeft);
        setMotorConfig(m_frontLeft);
        setMotorConfig(m_backRight);
        setMotorConfig(m_frontRight);



        m_drive = new MecanumDrive(m_frontLeft, m_backLeft, m_frontRight, m_backRight);



        // initializePIDConfig(rightFrontSide); what 2 do abt this ??

        // Sets the distance per pulse for the encoders

        // m_frontLeftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
        // m_backLeftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
        // m_frontRightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
        // m_backRightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);

    }


    private void report() {
        // Report various parameters out to network tables for monitoring purposes
        NetworkTableInstance.getDefault().getEntry("drive/back_left_position").setDouble(m_backLeft.getSelectedSensorPosition());
        NetworkTableInstance.getDefault().getEntry("drive/back_left_velocity").setDouble(m_backLeft.getSelectedSensorVelocity());

        NetworkTableInstance.getDefault().getEntry("drive/back_right_position").setDouble(m_backRight.getSelectedSensorPosition());
        NetworkTableInstance.getDefault().getEntry("drive/back_right_velocity").setDouble(m_backRight.getSelectedSensorVelocity());

        NetworkTableInstance.getDefault().getEntry("drive/front_left_position").setDouble(m_frontLeft.getSelectedSensorPosition());
        NetworkTableInstance.getDefault().getEntry("drive/front_left_velocity").setDouble(m_frontLeft.getSelectedSensorVelocity());

        NetworkTableInstance.getDefault().getEntry("drive/front_right_position").setDouble(m_frontRight.getSelectedSensorPosition());
        NetworkTableInstance.getDefault().getEntry("drive/front_right_velocity").setDouble(m_frontRight.getSelectedSensorVelocity());
    }


    @Override
    public void periodic() {

        report() ;

        // Update the odometry in the periodic block
        m_odometry.update( m_gyro.getRotation2d(), getCurrentWheelSpeeds() );
    }



    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }
    
    public MecanumDriveKinematics getKinematics() {
        return kinematics;
    }

    public PIDController getXController(){
        return xController;
    }

    public PIDController getYController(){
        return yController;
    }

    public ProfiledPIDController getThetaController(){
        return thetaController;
    }

    public PIDController getFrontLeftPIDController() {
        return frontLeftPID;
    }

    public PIDController getFrontRightPIDController() {
        return frontRightPID;
    }

    public PIDController getBackLeftPIDController() {
        return backLeftPID;
    }

    public PIDController getBackRightPidController() {
        return backRightPID;
    }

    public Rotation2d getDesiredRotation(){
        return Robot.selectedTrajectory[1].sample(Robot.m_autoTimer.get()).poseMeters.getRotation();
    }

    /**
     * Returns the current wheel speeds of the robot.
     *
     * @return The current wheel speeds.
     */
    public MecanumDriveWheelSpeeds getWheelSpeeds() {
        return new MecanumDriveWheelSpeeds(m_frontLeft.getSelectedSensorVelocity(),
                m_frontRight.getSelectedSensorVelocity(), m_backLeft.getSelectedSensorVelocity(),
                m_backRight.getSelectedSensorVelocity());
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(pose, m_gyro.getRotation2d());
    }

    /**
     * Drives the robot at given x, y and theta speeds. Speeds range from [-1, 1]
     * and the linear speeds have no effect on the angular speed.
     *
     * @param xSpeed        Speed of the robot in the x direction
     *                      (forward/backwards).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     */
    @SuppressWarnings("ParameterName")
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        if (fieldRelative) {
            m_drive.driveCartesian(ySpeed, xSpeed, rot, -m_gyro.getAngle());
        } else {
            if ( rot > 0.1) {
                int wpk = 1;
            }
            if ( ySpeed > 0.1 ) {
                int wpk = 1 ;
            }
            if ( xSpeed > 0.1 ) {
                int wpk = 1 ;
            }
            m_drive.driveCartesian(ySpeed, xSpeed, rot);
        }
    }

    /** Sets the front left drive SpeedController to a voltage. */
    public void setDriveSpeedControllersVolts(MecanumDriveMotorVoltages volts) {
        m_frontLeft.setVoltage(volts.frontLeftVoltage);
        m_backLeft.setVoltage(volts.rearLeftVoltage);
        m_frontRight.setVoltage(volts.frontRightVoltage);
        m_backRight.setVoltage(volts.rearRightVoltage);
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        m_backRight.setSelectedSensorPosition(0, 0, 0);
        m_backLeft.setSelectedSensorPosition(0, 0, 0);
    }

    /**
     * Gets the average distance of the two encoders.
     *
     * @return the average of the two encoder readings
     */
    public double getAverageEncoderDistance() {
        return (m_backLeft.getSelectedSensorPosition() + m_backRight.getSelectedSensorPosition()) / 2.0;
    }




    /**
     * Gets the current wheel speeds.
     *
     * @return the current wheel speeds in a MecanumDriveWheelSpeeds object.
     */
    public MecanumDriveWheelSpeeds getCurrentWheelSpeeds() {

        // Multiplying ny 10.0 because Talon reports velocity as counts per 100 mSec.
        return new MecanumDriveWheelSpeeds(
            m_frontLeft.getSelectedSensorVelocity() * 10.0,
            m_backLeft.getSelectedSensorVelocity() * 10.0, 
            m_frontRight.getSelectedSensorVelocity() * 10.0,
            m_backRight.getSelectedSensorVelocity() * 10.0
            );
    }

    /**
     * Sets the max output of the drive. Useful for scaling the drive to drive more
     * slowly.
     * 
     * 
     * /** Sets the max output of the drive. Useful for scaling the drive to drive
     * more slowly.
     *
     * @param maxOutput the maximum output to which the drive will be constrained
     */
    public void setMaxOutput(double maxOutput) {
        m_drive.setMaxOutput(maxOutput);
    }

    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
        m_gyro.reset();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return m_gyro.getRotation2d().getDegrees();
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return -m_gyro.getRate();
    }

    private void setMotorConfig(WPI_TalonFX motor) { //changed to TalonFX for intake
        motor.configFactoryDefault() ;
        // motor.configSelectedFeedbackSensor(
        //     FeedbackDevice.QuadEncoder, 
        //     Constants.DriveConstants.mainFeedbackLoop,
        //     Constants.DriveConstants.encoderTimeout
        //     ); 
        motor.configClosedloopRamp(Constants.DriveConstants.closedVoltageRampingConstant) ;
        motor.configOpenloopRamp(Constants.DriveConstants.manualVoltageRampingConstant) ;
        // motor.configNominalOutputForward(0);
        // motor.configNominalOutputReverse(0);
        // motor.configPeakOutputForward(1.0);
        // motor.configPeakOutputReverse(-1.0);
        //motor.configMotionCruiseVelocity( (int) (Constants.DriveConstants.unitsPerRotation * Constants.DriveConstants.desiredRPMsForDrive));
        motor.config_kF(Constants.DriveConstants.PID_id, Constants.DriveConstants.DrivetrainKf);
        motor.config_kP(Constants.DriveConstants.PID_id, Constants.DriveConstants.DrivetrainkP);
        motor.config_kI(Constants.DriveConstants.PID_id, 0);
        motor.config_kD(Constants.DriveConstants.PID_id, 0);
        motor.setNeutralMode(NeutralMode.Brake);
    }


    // public void setDefaultNeutralMode() {
    //     setBrakeMode();
    // }

    // public void setCoastMode() {
    //     for ( WPI_TalonSRX m : motors ) {
    //         m.setNeutralMode(NeutralMode.Coast);
    //     }
    // }

    // public void setBrakeMode() {
    //     for ( WPI_TalonSRX m : motors ) {
    //         m.setNeutralMode(NeutralMode.Brake);
    //     }
    // }



    public double getDistance() {
        double leftPos = m_frontLeft.getSelectedSensorPosition();
        double rightPos = m_frontRight.getSelectedSensorPosition();
        double averagePos = (leftPos + rightPos) / 2;
        return averagePos;
    }

    public void resetDistance() {
        m_frontLeft.setSelectedSensorPosition(0.0);
        m_frontRight.setSelectedSensorPosition(0.0);
    }





    // private void simulationInit() {
    //     PhysicsSim.getInstance().addTalonFX(m_frontRight, 0.75, 4000, true);
    //     PhysicsSim.getInstance().addTalonFX(m_frontLeft, 0.75, 4000, true);
    //     PhysicsSim.getInstance().addTalonFX(m_backRight, 0.75, 4000);
    //     PhysicsSim.getInstance().addTalonFX(m_backLeft, 0.75, 4000);
    // }


    boolean simulationInitialized = false ;

  public void simulationInit() {
        //PhysicsSim.getInstance().addTalonFX(m_frontRight, 0.75, 6800, true);
        //PhysicsSim.getInstance().addTalonFX(m_frontLeft, 0.75, 6800, true);
        //PhysicsSim.getInstance().addTalonFX(m_backRight, 0.75, 6800);
        //PhysicsSim.getInstance().addTalonFX(m_backLeft, 0.75, 6800);
}


  @Override
  public void simulationPeriodic() {
    if (! simulationInitialized) {
      simulationInit();
      simulationInitialized = true ;
    }

    // do sim stuff



  }





}

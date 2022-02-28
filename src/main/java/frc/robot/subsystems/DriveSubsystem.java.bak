// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import frc.robot.Constants;
import edu.wpi.first.networktables.*;
import frc.robot.sim.*;

public class DriveSubsystem extends SubsystemBase {

    WPI_TalonFX m_frontLeft;
    WPI_TalonFX m_backLeft;
    WPI_TalonFX m_frontRight;
    WPI_TalonFX m_backRight;

    ArrayList<WPI_TalonFX> motors = new ArrayList<WPI_TalonFX>();

    MecanumDriveOdometry odometry;

    public final MecanumDrive m_drive;

    // The gyro sensor
    private final Gyro m_gyro = new ADXRS450_Gyro();

    // Odometry class for tracking robot pose
    public MecanumDriveOdometry m_odometry = new MecanumDriveOdometry(DriveConstants.kDriveKinematics,
            m_gyro.getRotation2d());

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

        m_frontRight.setInverted(false);
        m_backRight.setInverted(false);
        m_frontLeft.setInverted(true);
        m_backLeft.setInverted(true);

        // m_backLeft.setInverted(false);
        // m_frontLeft.setInverted(false);
        // m_backRight.setInverted(true);
        // m_frontRight.setInverted(true);

        setMotorConfig(m_backLeft);
        setMotorConfig(m_frontLeft);
        setMotorConfig(m_backRight);
        setMotorConfig(m_frontRight);

        m_drive = new MecanumDrive(m_frontLeft, m_backLeft, m_frontRight, m_backRight);

    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        m_odometry.update(m_gyro.getRotation2d(), getCurrentWheelSpeeds());

        report();
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(pose, m_gyro.getRotation2d());
    }

    private double getSpeed(WPI_TalonFX motor) {
        // multiplied by 10 because velocity reported as counts per 1/10th second
        double s = motor.getSelectedSensorVelocity() * 10.0 * Constants.DriveConstants.Meters_Per_Count;
        return (s);
    }

    /**
     * Gets the current wheel speeds.
     *
     * @return the current wheel speeds in a MecanumDriveWheelSpeeds object.
     */
    public MecanumDriveWheelSpeeds getCurrentWheelSpeeds() {
        return new MecanumDriveWheelSpeeds(
                getSpeed(m_frontLeft),
                getSpeed(m_backLeft),
                getSpeed(m_frontRight),
                getSpeed(m_backRight));
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
            m_drive.driveCartesian(ySpeed, xSpeed, rot);
        }
    }

    /** Sets the front left drive MotorController to a voltage. */
    public void setDriveMotorControllersVolts(MecanumDriveMotorVoltages volts) {
        m_frontLeft.setVoltage(volts.frontLeftVoltage);
        m_backLeft.setVoltage(volts.rearLeftVoltage);
        m_frontRight.setVoltage(volts.frontRightVoltage);
        m_backRight.setVoltage(volts.rearRightVoltage);
        NetworkTableInstance.getDefault().getEntry("drive/front_left_volts").setDouble(volts.frontLeftVoltage);
        NetworkTableInstance.getDefault().getEntry("drive/front_right_volts").setDouble(volts.frontRightVoltage);
        NetworkTableInstance.getDefault().getEntry("drive/rear_left_volts").setDouble(volts.rearLeftVoltage);
        NetworkTableInstance.getDefault().getEntry("drive/rear_right_volts").setDouble(volts.rearRightVoltage);
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        m_frontLeft.setSelectedSensorPosition(0);
        m_backLeft.setSelectedSensorPosition(0);
        m_frontRight.setSelectedSensorPosition(0);
        m_backRight.setSelectedSensorPosition(0);
    }

    /**
     * Gets gyro heading from -180 to 180. CCW Positive
     * 
     * @return Gyro Heading [-180, 180]
     */
    public double getGyroHeading() {
        return Math.IEEEremainder(m_gyro.getAngle(), 360) * 1;
    }

    private void report() {
        // Report various parameters out to network tables for monitoring purposes
        NetworkTableInstance.getDefault().getEntry("drive/back_left_position")
                .setDouble(m_backLeft.getSelectedSensorPosition());
        NetworkTableInstance.getDefault().getEntry("drive/back_left_velocity")
                .setDouble(m_backLeft.getSelectedSensorVelocity());

        NetworkTableInstance.getDefault().getEntry("drive/back_right_position")
                .setDouble(m_backRight.getSelectedSensorPosition());
        NetworkTableInstance.getDefault().getEntry("drive/back_right_velocity")
                .setDouble(m_backRight.getSelectedSensorVelocity());

        NetworkTableInstance.getDefault().getEntry("drive/front_left_position")
                .setDouble(m_frontLeft.getSelectedSensorPosition());
        NetworkTableInstance.getDefault().getEntry("drive/front_left_velocity")
                .setDouble(m_frontLeft.getSelectedSensorVelocity());

        NetworkTableInstance.getDefault().getEntry("drive/front_right_position")
                .setDouble(m_frontRight.getSelectedSensorPosition());
        NetworkTableInstance.getDefault().getEntry("drive/front_right_velocity")
                .setDouble(m_frontRight.getSelectedSensorVelocity());
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

    private void setMotorConfig(WPI_TalonFX motor) { // changed to TalonFX for intake
        motor.configFactoryDefault();
        motor.configClosedloopRamp(Constants.DriveConstants.closedVoltageRampingConstant);
        motor.configOpenloopRamp(Constants.DriveConstants.manualVoltageRampingConstant);
        motor.config_kF(Constants.DriveConstants.PID_id, Constants.DriveConstants.DrivetrainKf);
        motor.config_kP(Constants.DriveConstants.PID_id, Constants.DriveConstants.DrivetrainkP);
        motor.config_kI(Constants.DriveConstants.PID_id, 0);
        motor.config_kD(Constants.DriveConstants.PID_id, 0);
        motor.setNeutralMode(NeutralMode.Brake);
    }

    boolean simulationInitialized = false;

    public void simulationInit() {
        PhysicsSim.getInstance().addTalonFX(m_frontRight, 0.75, 6800, true);
        PhysicsSim.getInstance().addTalonFX(m_frontLeft, 0.75, 6800);
        PhysicsSim.getInstance().addTalonFX(m_backRight, 0.75, 6800);
        PhysicsSim.getInstance().addTalonFX(m_backLeft, 0.75, 6800, true);
    }

    @Override
    public void simulationPeriodic() {
        if (!simulationInitialized) {
            simulationInit();
            simulationInitialized = true;
        }
        PhysicsSim.getInstance().run();

        // do sim stuff

    }

}

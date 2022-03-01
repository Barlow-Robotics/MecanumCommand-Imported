// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//No copyright? Nothing was in here when I opened it

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmBar;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.ArmBarConstants;

import edu.wpi.first.networktables.*;

public class Climb extends CommandBase {

    private ArmBar m_armBar;
    private DriveSubsystem m_drive;

    public enum ArmCommandState {
        ResettingPosition,
        WaitingForArmToBeStraightUp,
        DrivingBackward,
        MovingToHighBar,
        LettingGoMidBar,
        MovingToTraversalBar,
        LettingGoHighBar,
        OnTraversalBar,
        GoingToRestingPosition ,
        Finished
    };

    ArmCommandState currentState = ArmCommandState.WaitingForArmToBeStraightUp;

    /** Creates a new Climb. */
    public Climb(ArmBar a, DriveSubsystem d) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_armBar = a;
        addRequirements(m_armBar);
        m_drive = d;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        currentState = ArmCommandState.WaitingForArmToBeStraightUp;
    }

    // For brake mode: talon.setNeutralMode(NeutralMode.Brake),
    // For coast mode: talon.setNeutralMode(NeutralMode.Coast).

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        switch (currentState) {
            case ResettingPosition:
                //m_armBar.ResetPosition();
                currentState = ArmCommandState.WaitingForArmToBeStraightUp;
                break ;
            case WaitingForArmToBeStraightUp:
                if (Math.abs(m_armBar.getArmAngle() - ArmBarConstants.MidBarRotationAngle) > ArmBarConstants.AngleTolerance) {
                    m_armBar.rotateGripperArmDegree(ArmBarConstants.MidBarRotationAngle);
                } else {
                    currentState = ArmCommandState.DrivingBackward;
                }
                break;

            case DrivingBackward:
                if (m_armBar.gripperAIsClosed()) {
                    m_drive.drive(0.0, 0.0, 0, false); // how incorporate drivetrain?
                    currentState = ArmCommandState.MovingToHighBar;
                } else {
                    m_drive.drive(-0.1, 0.0, 0, false);
                }
                break;

            case MovingToHighBar:
                if (!m_armBar.gripperBIsClosed()) {
                    // wpk Need tthink about whether this is the correct angle angle from mid to high might be more than 180
                    m_armBar.rotateGripperArmDegree(ArmBarConstants.HighBarRotationAngle);
                } else {
                    currentState = ArmCommandState.LettingGoMidBar;
                }
                break;

            case LettingGoMidBar:
                m_armBar.releaseGripperA();
                if(m_armBar.gripperAIsOpen()){
                    m_armBar.neutralGripperA();
                    currentState = ArmCommandState.MovingToTraversalBar;
                }
                break;

            case MovingToTraversalBar:
                if (!m_armBar.gripperAIsClosed()) {
                    // wpk is this the angle we want?
                    m_armBar.rotateGripperArmDegree(ArmBarConstants.TraverseBarRotationAngle);
                } else {
                    m_armBar.setSlowMotionConfig();  // slow thngs down so we don't drop like a stone
                    currentState = ArmCommandState.LettingGoHighBar;
                }
                break;

            case LettingGoHighBar:
                m_armBar.releaseGripperB();
                if(m_armBar.gripperBIsOpen()){
                    m_armBar.neutralGripperB();
                    currentState = ArmCommandState.OnTraversalBar;
                }
                break;

            case OnTraversalBar:
                // do we let go or do we rotate it down using the member function?
                if (m_armBar.gripperAIsClosed() && m_armBar.gripperBIsOpen() ) {
                    currentState = ArmCommandState.GoingToRestingPosition;
                }
                break;

            case GoingToRestingPosition:
                if (Math.abs(m_armBar.getArmAngle() - ArmBarConstants.FinalRestingAngle) > ArmBarConstants.AngleTolerance) {
                    m_armBar.rotateGripperArmDegree(ArmBarConstants.FinalRestingAngle);
                } else {
                    currentState = ArmCommandState.Finished;
                }
                break ;

            case Finished:
                // Bot is immobile - nothing else needs to go here
                m_armBar.stopMotor();
                break;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_armBar.stopMotor();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return currentState == ArmCommandState.Finished ;
    }

    void report() {
        NetworkTableInstance.getDefault().getEntry("driverStation/climb_state").setString(currentState.name());
    }

}
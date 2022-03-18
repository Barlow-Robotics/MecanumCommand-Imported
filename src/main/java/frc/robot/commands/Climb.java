// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//No copyright? Nothing was in here when I opened it

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmBar;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.ArmBarConstants;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.Joystick;

public class Climb extends CommandBase {

    private ArmBar m_armBar;
    private DriveSubsystem m_drive;

    public enum ArmCommandState {
        ResettingPosition,
        FreeingBClaws ,
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
    Joystick controller ;
    Joystick gamePad ;


    /** Creates a new Climb. */
    public Climb(ArmBar a, Joystick c, Joystick gp) { 

        // Use addRequirements() here to declare subsystem dependencies.
        m_armBar = a;
        addRequirements(m_armBar);
        // m_drive = d;
        controller = c ;
        gamePad = gp ;

        updateHallStatesForDashboard() ;        
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_armBar.neutralGripperA();
        m_armBar.neutralGripperB();

        currentState = ArmCommandState.FreeingBClaws;
    }



    private void updateHallStatesForDashboard() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault() ;

        if ( currentState == ArmCommandState.DrivingBackward) {
            inst.getEntry("hall_effects/A1_At_Desired_State").setBoolean(!m_armBar.A1HallOpen()) ;
            inst.getEntry("hall_effects/A2_At_Desired_State").setBoolean(!m_armBar.A2HallOpen()) ;
            inst.getEntry("hall_effects/B1_At_Desired_State").setBoolean(m_armBar.B1HallOpen()) ;
            inst.getEntry("hall_effects/B2_At_Desired_State").setBoolean(m_armBar.B2HallOpen()) ;
        } else if ( currentState == ArmCommandState.MovingToHighBar) {
            inst.getEntry("hall_effects/A1_At_Desired_State").setBoolean(!m_armBar.A1HallOpen()) ;
            inst.getEntry("hall_effects/A2_At_Desired_State").setBoolean(!m_armBar.A2HallOpen()) ;
            inst.getEntry("hall_effects/B1_At_Desired_State").setBoolean(!m_armBar.B1HallOpen()) ;
            inst.getEntry("hall_effects/B2_At_Desired_State").setBoolean(!m_armBar.B2HallOpen()) ;
        } else if ( currentState == ArmCommandState.LettingGoMidBar) {
            inst.getEntry("hall_effects/A1_At_Desired_State").setBoolean(m_armBar.A1HallOpen()) ;
            inst.getEntry("hall_effects/A2_At_Desired_State").setBoolean(m_armBar.A2HallOpen()) ;
            inst.getEntry("hall_effects/B1_At_Desired_State").setBoolean(!m_armBar.B1HallOpen()) ;
            inst.getEntry("hall_effects/B2_At_Desired_State").setBoolean(!m_armBar.B2HallOpen()) ;
        } else if ( currentState == ArmCommandState.MovingToTraversalBar) {
            inst.getEntry("hall_effects/A1_At_Desired_State").setBoolean(!m_armBar.A1HallOpen()) ;
            inst.getEntry("hall_effects/A2_At_Desired_State").setBoolean(!m_armBar.A2HallOpen()) ;
            inst.getEntry("hall_effects/B1_At_Desired_State").setBoolean(!m_armBar.B1HallOpen()) ;
            inst.getEntry("hall_effects/B2_At_Desired_State").setBoolean(!m_armBar.B2HallOpen()) ;
        } else if ( currentState == ArmCommandState.LettingGoHighBar) {
            inst.getEntry("hall_effects/A1_At_Desired_State").setBoolean(!m_armBar.A1HallOpen()) ;
            inst.getEntry("hall_effects/A2_At_Desired_State").setBoolean(!m_armBar.A2HallOpen()) ;
            inst.getEntry("hall_effects/B1_At_Desired_State").setBoolean(m_armBar.B1HallOpen()) ;
            inst.getEntry("hall_effects/B2_At_Desired_State").setBoolean(m_armBar.B2HallOpen()) ;
        } else {
            inst.getEntry("hall_effects/A1_At_Desired_State").setBoolean(true) ;
            inst.getEntry("hall_effects/A2_At_Desired_State").setBoolean(true) ;
            inst.getEntry("hall_effects/B1_At_Desired_State").setBoolean(true) ;
            inst.getEntry("hall_effects/B2_At_Desired_State").setBoolean(true) ;
        }

    }


    double latchedAngle ;


    // For brake mode: talon.setNeutralMode(NeutralMode.Brake),
    // For coast mode: talon.setNeutralMode(NeutralMode.Coast).

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        NetworkTableInstance inst = NetworkTableInstance.getDefault() ;

        inst.getEntry("climb_command/climb_button").setBoolean(gamePad.getRawButton(Constants.Fight_Stick.Button_Y)) ;


        switch (currentState) {
            case ResettingPosition:
                m_armBar.ResetPosition(0.0);
                currentState = ArmCommandState.FreeingBClaws;
                m_armBar.neutralGripperA();
                m_armBar.neutralGripperA();
                break ;
            case FreeingBClaws:
                m_armBar.neutralGripperA();
                m_armBar.neutralGripperA();
                if (m_armBar.getArmAngle() > -43.0) {
                    m_armBar.rotateGripperArmDegree(-45.0);
                } else {

                    currentState = ArmCommandState.WaitingForArmToBeStraightUp;
                }
                break;
            case WaitingForArmToBeStraightUp:
                if (Math.abs(m_armBar.getArmAngle() - ArmBarConstants.MidBarRotationAngle) > ArmBarConstants.AngleTolerance) {
                    m_armBar.rotateGripperArmDegree(ArmBarConstants.MidBarRotationAngle);
                } else {
                    currentState = ArmCommandState.DrivingBackward;
                }
                break;

            case DrivingBackward:
                if (m_armBar.gripperAIsClosed()) {
                    //m_drive.setWheelSpeeds( new MecanumDriveWheelSpeeds( 0.0, 0.0, 0.0, 0.) );
                    // wpk test
                    
                    // if (controller.getRawButton(5)) {
                    if (gamePad.getRawButton(Constants.Fight_Stick.Button_Y)) {

                        currentState = ArmCommandState.MovingToHighBar;
                    }
                } else {
                    if ( gamePad.getPOV(0) == 0.0) {
                        m_armBar.rotateGripperArmDegree(ArmBarConstants.MidBarRotationAngle + 4.0);
                    } else if ( gamePad.getPOV(0) == 180.0) {
                        m_armBar.rotateGripperArmDegree(ArmBarConstants.MidBarRotationAngle - 4.0) ;
                    } else if ( gamePad.getPOV(0) == -1.0) {
                        m_armBar.rotateGripperArmDegree(ArmBarConstants.MidBarRotationAngle) ;
                    }
                    //m_drive.setWheelSpeeds( new MecanumDriveWheelSpeeds( -1.0, -1.0, -1.0, -1.0) );
                }
                break;

            case MovingToHighBar:
                if (!m_armBar.gripperBIsClosed()) {
                    // wpk Need tthink about whether this is the correct angle angle from mid to high might be more than 180
                    m_armBar.rotateGripperArmDegree(ArmBarConstants.HighBarRotationAngle);
                } else {
                    if (gamePad.getRawButton(Constants.Fight_Stick.Button_Y)) {
                        latchedAngle = m_armBar.getArmAngle() ;
                        currentState = ArmCommandState.LettingGoMidBar;
                        //m_armBar.rotateGripperArmDegree(latchedAngle);
                    }
                    m_armBar.stopMotor();
                }
                break;

            case LettingGoMidBar:
                m_armBar.releaseGripperA();
                if(m_armBar.gripperAIsOpen()){
                    m_armBar.neutralGripperA();
                    currentState = ArmCommandState.MovingToTraversalBar;
                } else {
                    // if ( gamePad.getPOV(0) == 180.0) {
                    //     m_armBar.rotateGripperArmDegree(latchedAngle + 5.0);
                    // } else if ( gamePad.getPOV(0) == 0.0)  {
                    //     m_armBar.rotateGripperArmDegree(latchedAngle - 5.0);
                    // } else {
                    //     m_armBar.rotateGripperArmDegree(latchedAngle);
                    // }
                    if ( gamePad.getPOV(0) == 180.0)  {
                        // rotate back to reduce pressure on claw so that servos can release.
                        m_armBar.rotateGripperArmDegree(latchedAngle -5.0);
                    } else {
                        m_armBar.stopMotor();
                    }
                }
                break;

            case MovingToTraversalBar:
                if (!m_armBar.gripperAIsClosed()) {
                    // wpk is this the angle we want?
                    m_armBar.rotateGripperArmDegree(ArmBarConstants.TraverseBarRotationAngle);
                } else {
                    // if (gamePad.getRawButton(Constants.Fight_Stick.Button_Y)) {
                    //     m_armBar.setSlowMotionConfig();  // slow thngs down so we don't drop like a stone
                    //     currentState = ArmCommandState.LettingGoHighBar;
                    //     latchedAngle = m_armBar.getArmAngle() ;
                    //     m_armBar.rotateGripperArmDegree(latchedAngle);
                    // }

                    if (gamePad.getRawButton(Constants.Fight_Stick.Button_Y)) {
                        m_armBar.setSlowMotionConfig();  // slow thngs down so we don't drop like a stone
                        latchedAngle = m_armBar.getArmAngle() ;
                        currentState = ArmCommandState.LettingGoHighBar;
                        //m_armBar.rotateGripperArmDegree(latchedAngle);
                    }
                    m_armBar.stopMotor();
                }
                break;

            case LettingGoHighBar:
                m_armBar.releaseGripperB();
                if (m_armBar.gripperBIsOpen()) {
                    m_armBar.neutralGripperB();
                    currentState = ArmCommandState.OnTraversalBar;
                } else {
                    // if ( gamePad.getPOV(0) == 180.0) {
                    //     m_armBar.rotateGripperArmDegree(latchedAngle + 5.0);
                    // } else if ( gamePad.getPOV(0) == 0.0)  {
                    //     m_armBar.rotateGripperArmDegree(latchedAngle - 5.0);
                    // } else {
                    //     m_armBar.rotateGripperArmDegree(latchedAngle);
                    // }

                    if ( gamePad.getPOV(0) == 180.0)  {
                        // rotate back to reduce pressure on claw so that servos can release.
                        m_armBar.rotateGripperArmDegree(latchedAngle -5.0);
                    } else {
                        m_armBar.stopMotor();
                    }

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
        updateHallStatesForDashboard();
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
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//No copyright? Nothing was in here when I opened it

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmBar;
import frc.robot.Constants;
import frc.robot.Constants.ArmBarConstants;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.Joystick;

public class Climb extends CommandBase {

    private ArmBar m_armBar;

    public enum ArmCommandState {
        ResettingPosition,
        FreeingBClaws ,
        WaitingForArmToBeStraightUp,
        AttachingToMidBar,
        MovingToHighBar,
        ReleavingLoadOnMidBar ,
        LettingGoMidBar,
        MovingToTraversalBar,
        ReleavingLoadOnHighBar ,
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

        report() ;        
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_armBar.neutralGripperA();
        m_armBar.neutralGripperB();

        //currentState = ArmCommandState.FreeingBClaws;
        currentState = ArmCommandState.ResettingPosition;

    }

    double latchedAngle ;

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        NetworkTableInstance inst = NetworkTableInstance.getDefault() ;

        inst.getEntry("climb_command/climb_button").setBoolean(gamePad.getRawButton(Constants.Fight_Stick.Button_Y)) ;

        switch (currentState) {
            case ResettingPosition:
                m_armBar.resetPosition(0.0);
                currentState = ArmCommandState.FreeingBClaws;
                m_armBar.neutralGripperA();
                m_armBar.neutralGripperA();
                break ;

            case FreeingBClaws:
                m_armBar.neutralGripperA();
                m_armBar.neutralGripperA();
                if (m_armBar.getArmAngle() > -(ArmBarConstants.FreeingBClawAngle - ArmBarConstants.AngleTolerance) ) {
                    m_armBar.rotateGripperArmDegree(-ArmBarConstants.FreeingBClawAngle);
                } else {
                    currentState = ArmCommandState.WaitingForArmToBeStraightUp;
                }
                break;

            case WaitingForArmToBeStraightUp:
                if (Math.abs(m_armBar.getArmAngle() - ArmBarConstants.MidBarRotationAngle) > ArmBarConstants.AngleTolerance) {
                    m_armBar.rotateGripperArmDegree(ArmBarConstants.MidBarRotationAngle);
                } else {
                    currentState = ArmCommandState.AttachingToMidBar;
                }
                break;

            case AttachingToMidBar:
                if (m_armBar.gripperAIsClosed()) {
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
                }
                break;




            case MovingToHighBar:
                if (!m_armBar.gripperBIsClosed()) {
                    // wpk Need tthink about whether this is the correct angle angle from mid to high might be more than 180
                    m_armBar.rotateGripperArmDegree(ArmBarConstants.HighBarRotationAngle);
                } else {
                    if (gamePad.getRawButton(Constants.Fight_Stick.Button_Y)) {
                        latchedAngle = m_armBar.getArmAngle() ;
                        currentState = ArmCommandState.ReleavingLoadOnMidBar;
                    }
                    m_armBar.stopMotor();
                }
                break;

            case ReleavingLoadOnMidBar:
                // rotate the arm backward to reduce load on claws
                m_armBar.rotateGripperArmDegree(latchedAngle - ArmBarConstants.LoadReducingAngle );
                if ( m_armBar.getArmAngle() <= latchedAngle - (ArmBarConstants.LoadReducingAngle - 1.0) ) {
                    // then start moving forward again and try to release from the bar
                    m_armBar.rotateGripperArmDegree(latchedAngle + ArmBarConstants.LoadReducingAngle);
                    currentState = ArmCommandState.LettingGoMidBar;
                }
                break ;

            case LettingGoMidBar:
                m_armBar.releaseGripperA();
                if(m_armBar.gripperAIsOpen()){
                    m_armBar.neutralGripperA();
                    currentState = ArmCommandState.MovingToTraversalBar;
                } else {
                    // if ( gamePad.getPOV(0) == 180.0)  {
                    //     // rotate back to reduce pressure on claw so that servos can release.
                    //     m_armBar.rotateGripperArmDegree(latchedAngle -5.0);
                    // } else {
                    //     m_armBar.stopMotor();
                    // }
                }
                break;



            case MovingToTraversalBar:
                if (!m_armBar.gripperAIsClosed()) {
                    m_armBar.rotateGripperArmDegree(ArmBarConstants.TraverseBarRotationAngle);
                } else {
                    if (gamePad.getRawButton(Constants.Fight_Stick.Button_Y)) {
//                        m_armBar.setSlowMotionConfig();  // slow thngs down so we don't drop like a stone
                        latchedAngle = m_armBar.getArmAngle() ;
                        currentState = ArmCommandState.ReleavingLoadOnHighBar;
                    }
                    m_armBar.stopMotor();
                }
                break;

            case ReleavingLoadOnHighBar:
                // rotate the arm backward to reduce load on claws
                m_armBar.rotateGripperArmDegree(latchedAngle - ArmBarConstants.LoadReducingAngle );
                if ( m_armBar.getArmAngle() <= latchedAngle - (ArmBarConstants.LoadReducingAngle - 1.0) ) {
                    // then start moving forward again and try to release from the bar
                    m_armBar.rotateGripperArmDegree(latchedAngle + ArmBarConstants.LoadReducingAngle);
                    currentState = ArmCommandState.LettingGoHighBar;
                }
                break ;

            case LettingGoHighBar:
                m_armBar.releaseGripperB();
                if (m_armBar.gripperBIsOpen()) {
                    m_armBar.neutralGripperB();
                    currentState = ArmCommandState.OnTraversalBar;
                } else {
                    // if ( gamePad.getPOV(0) == 180.0)  {
                    //     // rotate back to reduce pressure on claw so that servos can release.
                    //     m_armBar.rotateGripperArmDegree(latchedAngle -5.0);
                    // } else {
                    //     m_armBar.stopMotor();
                    // }
                }
                break;



            case OnTraversalBar:
                if (m_armBar.gripperAIsClosed() && m_armBar.gripperBIsOpen() ) {
                    currentState = ArmCommandState.GoingToRestingPosition;
                }
                break;

            case GoingToRestingPosition:
                // if (Math.abs(m_armBar.getArmAngle() - ArmBarConstants.FinalRestingAngle) > ArmBarConstants.AngleTolerance) {
                //     m_armBar.rotateGripperArmDegree(ArmBarConstants.FinalRestingAngle);
                // } else {
                    currentState = ArmCommandState.Finished;
                // }
                break ;


            case Finished:
                // Bot is immobile - nothing else needs to go here
                m_armBar.stopMotor();
                break;
        }

        report();

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
        NetworkTableInstance inst = NetworkTableInstance.getDefault() ;

        inst.getEntry("driverStation/climb_state").setString(currentState.name());

        if ( currentState == ArmCommandState.AttachingToMidBar) {
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

}
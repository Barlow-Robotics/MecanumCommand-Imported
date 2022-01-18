package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.UnderGlowConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class UnderGlow extends SubsystemBase {

    SerialPort port = null ;

    int currentMode = 0 ;
    
    public UnderGlow() {
        try {
            port = new SerialPort(9600, UnderGlowConstants.port ) ;
        } catch (Exception ex) {

        }
    }

    @Override
    public void periodic() {
        int desiredMode = UnderGlowConstants.NeonGreen ;

        if ( DriverStation.isEnabled() ) {
            if (DriverStation.getAlliance() == DriverStation.Alliance.Blue ) {
                desiredMode = UnderGlowConstants.BlueAliance ;
            } else if (DriverStation.getAlliance() == DriverStation.Alliance.Red ) {
                desiredMode = UnderGlowConstants.RedAliance ;
            }
        } else {
            desiredMode = UnderGlowConstants.NeonGreen ;
        }

        if ( currentMode != desiredMode && port != null) {
            try { 
                port.write(new byte[] {(byte)desiredMode}, 1) ;
            } catch (Exception ex) {

            }
            currentMode = desiredMode ;
        }


    }


    
}

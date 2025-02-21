package frc.lib.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class XboxController2 extends CommandXboxController {
    private int port;

    public XboxController2(int port) {
        super(port);
        this.port = port;
        //TODO Auto-generated constructor stub
    }

    public boolean getRawButtonPressed(int button) {
        return DriverStation.getStickButtonPressed(port, (byte) button);
    }
    
}

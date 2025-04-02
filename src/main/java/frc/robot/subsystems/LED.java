package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Limelight.LimelightData;
import edu.wpi.first.wpilibj2.command.Command;

public class LED extends SubsystemBase {
    private CANdle lights;
    public Boolean shutoff = false;

    public LED() {
        lights = new CANdle(20);

        configLEDs();
    }

    public void configLEDs() {
        
        //This is how you implement animations if you want to:
        // RainbowAnimation anim = new RainbowAnimation(1, 0.1, -1);
        // lights.animate(anim);

        lights.clearAnimation(0);
        
        //Auto sets to Red, maybe autoset to rainbow?
        lights.setLEDs(255, 0, 0);
        
    }

    public Command start() {
        return new Command() {
            @Override
            public void execute() {
                LimelightData.update();

                //Change LEDs based on whether or not recognizes a target
                if (LimelightData.isValidTarget()) {
                    lights.setLEDs(0, 255, 0);
                } else {
                    lights.setLEDs(255, 0, 0);
                }
            }
            
            @Override
            public boolean isFinished() {
                return shutoff;
            }

            @Override
            public void end(boolean interrupted) {
                lights.clearAnimation(0);
                lights.setLEDs(0, 0, 0);
            }
        };
    }
}

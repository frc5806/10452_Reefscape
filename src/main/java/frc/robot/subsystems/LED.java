package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
        // RainbowAnimation anim = new RainbowAnimation(1, 0.1, -1);
        // lights.animate(anim);
        lights.clearAnimation(0);
        // lights.setLEDs(80, 100, 170);
        // lights.animate(anim);
        // lights.setLEDs(255, 0, 0);
        // lights.modulateVBatOutput(0.9);
        // lights.configBrightnessScalar(0.1);
    }

    public Command start() {
        return new Command() {
            @Override
            public void execute() {
                LimelightData.update();
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

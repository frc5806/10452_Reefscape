package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Limelight.LimelightData;

public class LED extends SubsystemBase {
    private CANdle lights;

    public LED() {
        lights = new CANdle(20);

        // RainbowAnimation anim = new RainbowAnimation(1, 0.1, -1);
        // lights.animate(anim);
        lights.clearAnimation(0);
        // lights.setLEDs(80, 100, 170);
        // lights.animate(anim);
        // lights.setLEDs(255, 0, 0);
        // lights.modulateVBatOutput(0.9);
        // lights.configBrightnessScalar(0.1);
    }

    @Override
    public void periodic() {
        new InstantCommand(() -> System.out.println("checking"));
        LimelightData.update();
        if (LimelightData.isValidTargets()) {
            new InstantCommand(() -> lights.setLEDs(0, 255, 0));
        } else {
            new InstantCommand(() -> lights.setLEDs(255, 0, 0));
        }
    }
}

package frc.robot.commands.Intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class AutoIntake extends Command {
    private Intake intake;
    private DoubleSupplier powerSupplier;

    public AutoIntake(
            Intake intake,
            DoubleSupplier powerSupplier) {
        this.intake = intake;
        addRequirements(intake);

        this.powerSupplier = powerSupplier;
    }

    @Override
    public void execute() {
        /* Get Values */
        double power = MathUtil.applyDeadband(powerSupplier.getAsDouble(), 0);

        SmartDashboard.putNumber("power", power);
        // if (!intake.isOrange()) {
        //     intake.set(power);
        // }
        /* Intake */
    
    }
    
}

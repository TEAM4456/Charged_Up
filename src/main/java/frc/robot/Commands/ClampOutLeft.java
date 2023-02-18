package frc.robot.Commands;



import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Arm;

public class ClampOutLeft extends CommandBase{
    public final Arm arm;
    public ClampOutLeft(Arm arm){
        this.arm = arm;
    }
    public void execute() {
        arm.clampOutLeft();
    }
    public void end(boolean interrupted) {
        arm.clampStopLeft();

    }
}

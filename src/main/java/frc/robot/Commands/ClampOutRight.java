package frc.robot.Commands;



import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Arm;

public class ClampOutRight extends CommandBase{
    public final Arm arm;
    public ClampOutRight(Arm arm){
        this.arm = arm;
    }
    public void execute() {
        arm.clampOutRight();
    }
    public void end(boolean interrupted) {
        arm.clampStopRight();

    }
}

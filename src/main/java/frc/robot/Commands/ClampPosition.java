package frc.robot.Commands;



import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Arm;

public class ClampPosition extends CommandBase{
    public final Arm arm;
    public ClampPosition(Arm arm){
        this.arm = arm;
    }
    public void execute() {
        arm.clampInPosition();
    }
    public void end(boolean interrupted){
        arm.clampOutPosition();
    }
}

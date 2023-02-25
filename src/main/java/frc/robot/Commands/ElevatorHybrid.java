package frc.robot.Commands;



import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Arm;

public class ElevatorHybrid extends CommandBase{
    public final Arm arm;
    public ElevatorHybrid(Arm arm) {
        this.arm = arm;
    }
    public void execute() {
        arm.elevatorPosition(100);
    }

}
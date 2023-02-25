package frc.robot.Commands;



import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Arm;

public class ElevatorHighCube extends CommandBase{
    public final Arm arm;
    public ElevatorHighCube(Arm arm) {
        this.arm = arm;
    }
    public void execute() {
        arm.elevatorPosition(100);
    }

}

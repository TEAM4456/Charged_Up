package frc.robot.Commands;
import frc.robot.Constants;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Arm;

public class ElevatorHighCone extends CommandBase{
    public final Arm arm;
    public ElevatorHighCone(Arm arm) {
        this.arm = arm;
    }
    public void execute() {
        arm.elevatorPosition(Constants.armConstants.elevatorHighCone);
    }

}

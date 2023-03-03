package frc.robot.Commands;
import frc.robot.Constants;
import frc.robot.Subsystems.Arm;

public class ElevatorLowCone {
    public final Arm arm;
    public ElevatorLowCone(Arm arm) {
        this.arm = arm;
    }
    public void execute() {
        arm.elevatorPosition(Constants.armConstants.elevatorLowCube);
    }
}

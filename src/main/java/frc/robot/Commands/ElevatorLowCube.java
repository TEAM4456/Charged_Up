package frc.robot.Commands;
import frc.robot.Constants;
import frc.robot.Subsystems.Arm;

public class ElevatorLowCube {
    public final Arm arm;
    public ElevatorLowCube(Arm arm) {
        this.arm = arm;
    }
    public void execute() {
        arm.elevatorPosition(Constants.armConstants.elevatorLowCube);
    }
}

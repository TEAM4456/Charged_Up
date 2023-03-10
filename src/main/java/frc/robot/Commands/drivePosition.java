package frc.robot.Commands;
import frc.robot.Constants;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Arm;

public class drivePosition extends CommandBase{
    public final Arm arm;
    public drivePosition(Arm arm) {
        this.arm = arm;
    }
    public void execute() {
        arm.elevatorPosition(Constants.armConstants.elevatorDrive);
        arm.rotatePosition(Constants.armConstants.rotateDrive);
    }
    public boolean isFinished() {
        return 
        (Math.abs(arm.elevatorEncoderRight.getPosition() - Constants.armConstants.elevatorDrive) < 1)
        &&
        (Math.abs(arm.rotateEncoder.getPosition() - Constants.armConstants.rotateDrive) < 1);
      }

}

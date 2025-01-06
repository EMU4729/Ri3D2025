package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.constants.AlgaeArmConstants;
import frc.robot.constants.CoralArmConstants;
import frc.robot.constants.ElevatorConstants;

public class ActivateAlgaeUnload extends ActivateBase{
    public ActivateAlgaeUnload(){
    }

    @Override
    public void initialize() {
        super.initialize();
        Subsystems.elevator.setTargetHeight(ElevatorConstants.HEIGHTS.ALGAE_PROCESSOR);
        Subsystems.algaeArm.setTargetAngle(AlgaeArmConstants.ANGLES.UNLOAD);
    }
}

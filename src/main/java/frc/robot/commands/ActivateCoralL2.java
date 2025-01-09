package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.constants.AlgaeArmConstants;
import frc.robot.constants.CoralArmConstants;
import frc.robot.constants.ElevatorConstants;

public class ActivateCoralL2 extends ActivateBase{
    public ActivateCoralL2(){this(false);}
    public ActivateCoralL2(boolean StayUnstowed){
        super(StayUnstowed);
    }

    @Override
    public void initialize() {
        System.out.println("Moving to unload coral at L1");
        super.initialize();
        Subsystems.elevator.setTargetHeight(ElevatorConstants.HEIGHTS.CORAL_L2);
        Subsystems.coralArm.setTargetAngle(CoralArmConstants.ANGLES.UNLOAD);
    }
}

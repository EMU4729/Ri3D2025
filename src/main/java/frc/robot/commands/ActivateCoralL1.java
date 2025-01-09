package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.constants.AlgaeArmConstants;
import frc.robot.constants.CoralArmConstants;
import frc.robot.constants.ElevatorConstants;

public class ActivateCoralL1 extends ActivateBase{
    public ActivateCoralL1(){this(false);}
    public ActivateCoralL1(boolean StayUnstowed){
        super(StayUnstowed);
    }


    @Override
    public void initialize() {
        System.out.println("Moving to unload coral at L1");
        super.initialize();
        Subsystems.elevator.setTargetHeight(ElevatorConstants.HEIGHTS.CORAL_L1);
        Subsystems.coralArm.setTargetAngle(CoralArmConstants.ANGLES.UNLOAD);
    }
}

package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.constants.AlgaeArmConstants;
import frc.robot.constants.CoralArmConstants;
import frc.robot.constants.ElevatorConstants;

public class ActivateAlgaeL2 extends ActivateBase{
    public ActivateAlgaeL2(){
        addRequirements(/*Subsystems.coralArm,*/ Subsystems.elevator);
    }

    @Override
    public void initialize() {
        super.initialize();
        Subsystems.elevator.setTargetHeight(ElevatorConstants.HEIGHTS.ALGAE_L2);
        Subsystems.algaeArm.setTargetAngle(AlgaeArmConstants.ANGLES.LOAD_START);
        //start wheels
    }

    @Override
    public void end(boolean interrupted) {
        // stop wheels
        super.end(interrupted);
    }
}

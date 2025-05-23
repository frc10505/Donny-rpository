package frc.team10505.robot;

import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.team10505.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotContainer {
private void configBindings(){
if (Utils.isSimulation()) {
    Joystick.button(1).onTrue(ElevatorSubsystem.setHeight(5));
    
} else {
    Joystick.button(1).onTrue(ElevatorSubsystem.setHeight(5));

}
}

private SendableChooser<Command> autonChooser;
public RobotContainer() {
    autonChooser = AutoBuilder.buildAutoChooser();
    configBindings();
    SmartDashboard.putData("autonChooser", autonChooser);
}
    
}

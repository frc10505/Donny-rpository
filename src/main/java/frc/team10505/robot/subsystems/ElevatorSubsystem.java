package frc.team10505.robot.subsystems;

import java.util.Queue;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ElevatorSubsystem extends SubsystemBase {
    // Motors
    public int kElevatorMotorId = (0);
    public final TalonFX elevatorMotor = new TalonFX(kElevatorMotorId, "KingKan");
    public int kElevatorFollowerMotorId = 9;
    public final TalonFX elevatorFollowerMotor = new TalonFX(kElevatorFollowerMotorId, "KingKan");

    // Encoders, Real and simulated
    private double elevatorEncoderValue = 0.0;

    private double totalEffort;
    // operator.interface

    private double height = 0.0;
    
    // Controls, Actual
    private final PIDController elevatorController = new PIDController(0, 0, 0);
    private final ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(0, 0, 0.1, 0.1);

    public boolean usePID = true;
    public double kElevatorMotorCurrentLimit = 30;

    /* Constructor, runs everthing inside during initialization */
    public ElevatorSubsystem() {
        elevatorMotor.setPosition(0.0);

        // set current limits
        var limitConfigs = new CurrentLimitsConfigs();
        limitConfigs.StatorCurrentLimit = kElevatorMotorCurrentLimit;
        limitConfigs.StatorCurrentLimitEnable = true;

       // motorConfig = MotorOutputConfigs.NeutralModeValue.Brake;
       var motorConfig = new MotorOutputConfigs();
       motorConfig.NeutralMode = NeutralModeValue.Brake;

        elevatorMotor.getConfigurator().apply(motorConfig);
        elevatorMotor.getConfigurator().apply(limitConfigs);

        elevatorFollowerMotor.getConfigurator().apply(motorConfig);
        elevatorFollowerMotor.getConfigurator().apply(limitConfigs);
        elevatorFollowerMotor.setControl(new Follower(elevatorMotor.getDeviceID(), false));
    }
    

    /* commands to referense */
    // changes our setpoint, which changes our pid calcuations therefore effort to
    // the motor, which happens periodically
    public Command setHeight(double newHeight) {
        return Commands.runOnce(() -> {
            height = newHeight;
        });
    }

    public Command setMotor(double voltage){
        return Commands.runEnd(() -> {
                    usePID = false;
                    elevatorMotor.setVoltage(voltage);
            },
            () -> {
                elevatorMotor.setVoltage(0);
                usePID = true;

            });
    }       
    // ONLY to use for testing motor direction
    // public Command testElevator(double voltage){
    // return runEnd(() -> {
    // elevatorMotor.setVoltage(voltage);()
    // }, () -> {
    // elevatorMotor.setVoltage(0.0);
    // });
            // }

    /* Calculations */
    public double getElevatorEncoder() {
        return (elevatorMotor.getRotorPosition().getValueAsDouble() * (Math.PI * 1.751 * 2) / 12.0) * -1.0;
    }

     public boolean isNearGoal() {
        return MathUtil.isNear(height, getElevatorEncoder(), 2);
    }

    public boolean issGigh () {
        return getElevatorEncoder() > 30;
    }

    public boolean isAbove(double heightOfChoice){
        return getElevatorEncoder() > heightOfChoice;
    }

    public double getEffort() {
        return totalEffort = ((elevatorFeedforward.calculate(0, 0))
                + (elevatorController.calculate(getElevatorEncoder(), height)));
    }





 public void periodic() {
        elevatorEncoderValue = getElevatorEncoder();
        totalEffort = getEffort();

       // if(usePID){
        elevatorMotor.setVoltage(totalEffort * -1.0);
      //  }

        SmartDashboard.putNumber("Elevator Encoder", elevatorEncoderValue);
        SmartDashboard.putNumber("Elevator Effort", totalEffort);
        SmartDashboard.putNumber("Elevator Height", height);
        SmartDashboard.putBoolean("issGigh", issGigh());

    }

}



        


        


    
    



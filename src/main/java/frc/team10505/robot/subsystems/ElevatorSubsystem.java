package frc.team10505.robot.subsystems;

import java.util.Queue;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@SuppressWarnings("unused")
public class ElevatorSubsystem extends SubsystemBase {
    // Motors
    // public final TalonFx elevatorMotor = new TalonFx(kElevatorMotorId);
    public int kElevatorMotorId;
    public final TalonFX elevator = new TalonFX(kElevatorMotorId, "KingKan");
    public int kElevatorFollowerMotorId;

    // public final TalonFx elevatorFollowerMotor = new
    // TalonFx(kElevatorFollowerMotorId);
    public final TalonFX elevatorFollowerMotor = new TalonFX(kElevatorFollowerMotorId, "KingKan");

    // Encoders, Real and simulated
    private double elevatorEncoderValue = 0.0;

    private double totalEffort;
    // operator.interface
    public final SendableChooser<Double> elevatorHeight = new SendableChooser<>();
    private double height = 0.0;
    public double KP;
    public double KI;
    public double KD;

    // Controls, Actual
    private final PIDController elevatorController = new PIDController(KP, KI, KD);
    public double KS;
    public double KG;
    public double KV;
    private double KA;
    private final ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(KS, KG, KV, KA);

    public boolean usePID = true;
    public CoreTalonFX elevatorMotor;
    public double kElevatorMotorCurrentLimit;
    private TalonFXConfiguration limitConfigs;
    public double heght;
    private double newHeight;
    public double MathUntil;

    /* Constructor, runs everthing inside during initialization */
    public ElevatorSubsystem() {
        elevatorMotor.setPosition(0.0);
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();

        // set current limits
        limitConfigs = new TalonFXConfiguration();
        limitConfigs.CurrentLimits.StatorCurrentLimit = kElevatorMotorCurrentLimit;
        limitConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        motorConfig.NeutralMode = NeutralModeValue.Brake;
        elevatorMotor.getConfigurator().apply(motorConfig);
        elevatorMotor.getConfigurator().apply(limitConfigs);

        motorConfig.NeutralMode = NeutralModeValue.Brake;
        elevatorFollowerMotor.getConfigurator().apply(motorConfig);
        elevatorFollowerMotor.getConfigurator().apply(limitConfigs);
        elevatorFollowerMotor.setControl(new Follower(elevatorMotor.getDeviceID(), false));

    

    /* commands to referense */
    // changes our setpoint, which changes our pid calcuations therefore effort to
    // the motor, which happens periodically
    public edu.wpi.first.wpilibj2.command.Command setHeight(double newHeight) {
        return runOnce(() -> {
            height = newHeight;
        });
    }

    public Command setMotor(double voltage){
        return runEnd(() -> {
                    usePID = false;
                    elevator.setVoltage(voltage);
                });

            }
            private Command runEnd(Runnable runnable) {
                // TODO Auto-generated method stub
                throw new UnsupportedOperationException("Unimplemented method 'runEnd'");

            // ONLY to use for testing motor direction
            // public Command testElevator(double voltage){
            // return runEnd(() -> {
            // elevatorMotor.setVoltage(voltage);()
            // }, () -> {
            // elevatorMotor.setVoltage(0.0);
            // });
            // }

            /* Calculations */
            public double getElevatorEncoder () {
                return (elevatorMotor.getRotorPosition().getValueAsDouble());  
            }


        


    
    



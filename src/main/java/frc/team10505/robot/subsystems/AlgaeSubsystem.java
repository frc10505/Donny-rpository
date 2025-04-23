package frc.team10505.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.team10505.robot.Constants.AlgaeConstants.*;

public class AlgaeSubsystem extends SubsystemBase {
    public final int kAlgaePivotMotorId = 8;
    public final int kPivotMotorCurrentLimit = 15;
    public final int kAlgaeIntakeMotorID = 7;
    public final int kAlgaeIntakeMotorCurrentLimit = 25;

    private final SparkMax intakeMotor = new SparkMax(kAlgaeIntakeMotorID, MotorType.kBrushless);
    private SparkMaxConfig pivotMotorConfig = new SparkMaxConfig();
    private final SparkMax pivotMotor = new SparkMax(kAlgaePivotMotorId, MotorType.kBrushless);
    private SparkMaxConfig intakeMotorConfig = new SparkMaxConfig();

    private final SparkAbsoluteEncoder pivotEncoder = pivotMotor.getAbsoluteEncoder();
    private double encoderValue;
    private double AbsoluteOffset = 180.0;

    private PIDController pivotController = new PIDController(0, 0, 0);
    private ArmFeedforward pivotFeedForward = new ArmFeedforward(0, 0, 0, 0);

    private double pivotSetPoint = -90;

    public boolean coasting = false;

    public AlgaeSubsystem() {
        configAlgaeSubsys();
        SmartDashboard.putNumber("pivotEncoder", encoderValue);
    }

    public double getPivotEncoder() {
        return -pivotEncoder.getPosition() + AbsoluteOffset;
    }

    public double GetEffort() {
        return pivotFeedForward.calculate(Units.degreesToRadians(getPivotEncoder()), 0)
        + pivotController.calculate(getPivotEncoder(), pivotSetPoint);
    }

    public Command setVoltage(double voltage) {
        return run(() -> {
            pivotSetPoint = voltage;
        });
    }

    public Command holdAngle() {
        return run(() -> {
            if (!coasting) {
                pivotMotor.setVoltage(GetEffort());
            }
        });
    }

    public Command stopPivot() {

        return runOnce(() -> {
            pivotMotor.stopMotor();
        });
    }

    public Command coastPivot() {
        return run(() -> {
            pivotMotorConfig.idleMode(IdleMode.kCoast);
            coasting = true;
        });
    }

    public Command brakePivot() {
        return run(() -> {
            pivotMotorConfig.idleMode(IdleMode.kBrake);
            pivotSetPoint = getPivotEncoder();
            coasting = false;
        });
    }

    public Command intakeFoward() {
        return runOnce(() -> {
            intakeMotor.set(intakeSpeed);
        });
    }

    public Command intakeFowardSlowest() {
        return runOnce(() -> {
            intakeMotor.set(intakeSlowSpeed);
        });
    }

    public Command intakeReverse() {
        return runOnce(() -> {
            intakeMotor.set(-intakeSpeed);
        });
    }

    public Command intakeStop() {
        return runOnce(() -> {
            intakeMotor.set(0);
        });
    }

    public void perodic() {
        encoderValue = getPivotEncoder();
        SmartDashboard.putNumber("pivotEncoder", encoderValue);
        SmartDashboard.putNumber("Intake Motor Output", intakeMotor.getAppliedOutput());
        SmartDashboard.putNumber("Pivot Motor Output", pivotMotor.getAppliedOutput());
    }

    private void configAlgaeSubsys() {
        pivotMotorConfig.idleMode(IdleMode.kBrake);
        pivotMotorConfig.smartCurrentLimit(kPivotMotorCurrentLimit, kPivotMotorCurrentLimit);
        pivotMotorConfig.absoluteEncoder.positionConversionFactor(pivotEncoderScale);
        pivotMotorConfig.absoluteEncoder.zeroOffset(pivotEncoderOffset);
        pivotMotor.configure(pivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        intakeMotorConfig.idleMode(IdleMode.kBrake);
        intakeMotorConfig.smartCurrentLimit(kAlgaeIntakeMotorCurrentLimit, kAlgaeIntakeMotorCurrentLimit);
        intakeMotor.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

}

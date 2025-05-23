package frc.team10505.robot.subsystems;

import static frc.team10505.robot.Constants.AlgaeConstants.intakeSlowSpeed;
import static frc.team10505.robot.Constants.AlgaeConstants.intakeSpeed;
import static frc.team10505.robot.Constants.AlgaeConstants.kAlgaePivotMotorId;
import static frc.team10505.robot.Constants.AlgaeConstants.pivotEncoderOffset;

import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeSubsystem extends SubsystemBase {
public final double SmartCurrentLimit = 20;
public final double pivotEncoderScale = 30;
public final double kResetSafeParameters = 34;
public final double kPivotMotorCurrentLimit = 15;
public final double kAlgaeIntakeMotorID = 1;
public int kpivotMotorID = 0;
private final static SparkMax intakeMotor = new SparkMax(kAlgaeIntakeMotorID, MotorType.kBrushless);
private SparkMaxConfig pivotMotorConfig = new SparkMaxConfig();
private final SparkMax pivotMotor = new SparkMax(kAlgaePivotMotorId, MotorType.kBrushless);
private SparkMaxConfig pivotMotorConfig2 = new SparkMaxConfig();


private final SparkAbsoluteEncoder pivotEncoder = pivotMotor.getAbsoluteEncoder();
private double encoderValue;
private double absoluteOffset = 180;


private double pivotSetPoint = -90;

public boolean coasting = false;

public AlgaeSubsystem() {
    configAlgaeSubsys();
    SmartDashboard.putNumber("pivotEncoder", encoderValue);
}

public double getPivotEncoder() {
    return -pivotEncoder.getPosition() + absoluteOffset; }


public Command setVoltage(double voltage) {
    return runOnce (() -> {
        pivotSetPoint = 0;
    });
}
}

public Command holdAngle() {
    return runOnce (() -> {
        if (!coastng) {
            pivotMotor.setVoltage(PIDEffort)();
        }
    });
}

public Command stopPivot
    return runOnce = (() -> {
        pivotMotor.stopMotor();
    });

    public Command coastPivot() {
        return run(() -> {
            pivotMotorConfig.idleMode(idleMode.kCoast);
            coasting = true;
        });
    }

    public Command brakePivot() {
        return run(() -> {
            pivotMotorConfig.idleMode(idleMode.kBrake);
            pivotSetPoint = getPivotEncoder();
            coasting = false;
        });
    }

public Command intakeFoward() {
    return runOnce(() -> {
        intakeMotor.set(intakeSpeed);
    });
}

public Command intakeFowardSlower() {
    return runOnce(() -> {
        intakeMotor.Set(intakeSlowSpeed);
    });
}

public Command intakeFowardSlowest() {
    return runOnce(() -> {
        intakeMotor.Set(intakeFowardSlowest());
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
    SmartDashboard.putNumber("Intake motor Output", intakeMotor.getAppliedOutput());
    SmartDashboard.putNumber("pivot Motor Output", pivotMotor.getAppliedOutput());}


private void configAlgaeSubsys() {
    pivotMotorConfig.idleMode(idleMode.kBrake);
    pivotMotorConfig.smartCurrentLimit(kPivotMotorCurrentLimit, kPivotMotorCurrentLimit);
    pivotMotorConfig.absoluteEncoder.positionConversionFactor(pivotEncoderScale);
    pivotMotorConfig.absoluteEncoder.zeroOffset(pivotEncoderOffset);
    pivotMotorConfig.configure(pivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    intakeMotorConfig.idlemode(idleMode.kBrake);
    intakeMotorConfig.SmartCurrentLimit(SmartCurrentLimit, 
        kAlgaeIntakeMotorCurrentLimit);
    intakeMotor.configure("intakeMotorConfig", PersistMode.kResetSafeParameters, PersistMode.kPersistParameters);
}










    
}

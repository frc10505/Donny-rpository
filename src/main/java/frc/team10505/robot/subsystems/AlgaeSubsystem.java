package frc.team10505.robot.subsystems;

import static frc.team10505.robot.Constants.AlgaeConstants.intakeSlowSpeed;
import static frc.team10505.robot.Constants.AlgaeConstants.intakeSpeed;
import static frc.team10505.robot.Constants.AlgaeConstants.kAlgaeIntakeMotorCurrentLimit;
import static frc.team10505.robot.Constants.AlgaeConstants.kAlgaePivotMotorId;
import static frc.team10505.robot.Constants.AlgaeConstants.pivotEncoderOffset;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeSubsystem extends SubsystemBase {
public final int SmartCurrentLimit = 20;
public final int pivotEncoderScale = 30;
public final int kResetSafeParameters = 34;
public final int kPivotMotorCurrentLimit = 15;
public final int kAlgaeIntakeMotorID = 1;
public int kpivotMotorID = 0;
private double simSpeed = 0;
private final SparkMax intakeMotor = new SparkMax(kAlgaeIntakeMotorID, MotorType.kBrushless);
private SparkMaxConfig pivotMotorConfig = new SparkMaxConfig();
private final SparkMax pivotMotor = new SparkMax(kAlgaePivotMotorId, MotorType.kBrushless);
private SparkMaxConfig intakeMotorConfig = new SparkMaxConfig();

private PIDController pivotController;
private ArmFeedforward pivotFeedforward;

private final SparkAbsoluteEncoder pivotEncoder = pivotMotor.getAbsoluteEncoder();
private double encoderValue;
private double absoluteOffset = 180;


private double pivotSetPoint = -90;

public boolean coasting = false;

public AlgaeSubsystem() {
    // configAlgaeSubsys();
    // SmartDashboard.putNumber("pivotEncoder", encoderValue);
    if (Utils.isSimulation()) {
        pivotController = new PIDController(0, 0, 0);
        pivotFeedforward = new ArmFeedforward(0, 0, 0, 0);
    }
}

public double getPivotEncoder() {
    return -pivotEncoder.getPosition() + absoluteOffset; }


public Command setVoltage(double voltage) {
    return runOnce (() -> {
        pivotSetPoint = 0;
    });
}


// public Command holdAngle() {
//     return run (() -> {
//         if (!coastng) {
//             pivotMotor.setVoltage(PIDEffort);
//         }
//     });
// }

public Command stopPivot () {
    return run (() -> {
        pivotMotor.stopMotor();
     } );
    };

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

public Command intakeFoward(double speed) {
    return runOnce(() -> {
        simSpeed = speed;
    });
}

// public Command intakeFowardSlower() {
//     return runOnce(() -> {
//         intakeMotor.set(intakeSlowSpeed);
//     });
// }

// public Command intakeFowardSlowest() {
//     return runOnce(() -> {
//         intakeMotor.set(intakeSlow);
//     });
// }

// public Command intakeReverse() {
//     return runOnce(() -> {
//         intakeMotor.set(-intakeSpeed);
//     });
// }

// public Command intakeStop() {
//     return runOnce(() -> {
//         intakeMotor.set(0);
//     });
// }

public void perodic() {
    encoderValue = getPivotEncoder();
    SmartDashboard.putNumber("pivotEncoder", encoderValue);
    SmartDashboard.putNumber("Intake motor Output", intakeMotor.getAppliedOutput());
    SmartDashboard.putNumber("pivot Motor Output", pivotMotor.getAppliedOutput());}


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

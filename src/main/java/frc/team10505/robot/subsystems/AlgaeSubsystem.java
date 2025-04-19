package frc.team10505.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

public class AlgaeSubsystem extends SubsystemBase {

    //motor controllers

    private final static SparkMax intakeMotor = new SparkMax(kAlgaeIntakeMotorID, MotorType.kBrushless);
}   private SparkMaxConfig pivotMotorConfig = new SparkMaxConfig();

//Encodor
private final SparkAbsoluteEncoder pivotEncoder = pivotMotor.getAbsoluteEncoder();
private double encoderValue;
private double absoluteOffset = 180.0;

//Controller
private


    
        
    


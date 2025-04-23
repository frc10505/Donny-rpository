package frc.team10505.robot;

public class Constants {
    public final class AlgaeConstants {
        public final static int kAlgaePivotMotorId = 8;
        public final static int kPivotMotorCurrentLimit = 15;
        public final static int kAlgaeIntakeMotorID = 7;
        public final static int kAlgaeIntakeMotorCurrentLimit = 25;

        // Intake speed
        public final static double intakeSpeed = 0.5;// 0.5
        public final static double intakeSlowSpeed = 0.3; // 0.3
        public final static double bunsSpeed = 0.2;

        public final static double pivotEncoderOffset = 0;
        public final static double pivotEncoderScale = 360;

        // PID Contants
        public final static double KP = 0.1;
        public final static double KI = 0.0;
        public final static double KD = 0.0;

        // Simulation Constants
        public final static int gearing = 36;
        public final static double lenthMeters = 0.5;
        public final static double massKg = 3.0;

        // sim pivot PID constants
        public final static double simPivotKP = 0.3; // $$0.4
        public final static double simPivotKI = 0;
        public final static double simPivotKD = 0.001;

        // sim pivot ffe contants
        public final static double simPivotKS = 4.0;
    public final static double simPivotKG = 1.315;
        public final static double simPivotKV = 0.4;
        public final static double simPivotKA = 0.1;
    }

    public final class CoralConstants {
        public final static int kLeftMotorId = 2;
        public final static int kLeftMotorCurrentLimit = 15;
        public final static int kRightMotorID = 3;
        public final static int kRightMotorCurrentLimit = 15;
        public final static int kIntakeInId = 60;
        public final static int kIntakeOutId = 61;
        public final static double kIntakeSpeed = 0.37;// .4
        public final static double kLeftL1Speed = 0.5;// .2
        public final static double kRightL1Speed = 0.25;// .4
        public final static double kOutakeSpeed = 0.25;
        public final static double kOutakeTopSpeed = 0.2;// .1
        public final static double kTroughSpeed = 0.30;
        public final static double kTroughRIghtMotorPercentage = 0.9;
    }
}
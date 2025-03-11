package frc.robot.math;

import edu.wpi.first.math.controller.ArmFeedforward;

import frc.robot.Constants.ArmConstants;

public class ArmMath {
    
    public static ArmFeedforward createShoulderFeedforward() {

        double ks = 0.01;
        double kg = ArmConstants.SHOULDER_KG;
        double kv = ArmConstants.SHOULDER_KV;
        double ka = ArmConstants.SHOULDER_KA;


        return new ArmFeedforward(ks, kg, kv, ka);
    };

    public static ArmFeedforward createWristFeedforward() {

        double ks = 0.01;
        double kg = ArmConstants.WRIST_KG;
        double kv = ArmConstants.WRIST_KV;
        double ka = ArmConstants.WRIST_KA;


        return new ArmFeedforward(ks, kg, kv, ka);
    };

}


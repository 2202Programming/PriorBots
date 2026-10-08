// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot2026.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot2026.Constants.CAN;

/** Add your docs here. */
public class Hopper extends SubsystemBase {
    final SparkFlex belts;
    final SparkFlexConfig beltsCfg;

    final RelativeEncoder encoder;

    boolean inverted = true;

    int stallAmp = 60; // [AMP]
    int freeAmp = 10; // [AMP]

    public Hopper() {
        belts = new SparkFlex(CAN.BeltID, MotorType.kBrushless);
        beltsCfg = new SparkFlexConfig();
        encoder = belts.getEncoder();

        beltsCfg
            .inverted(inverted)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(stallAmp, freeAmp);

        belts.configure(beltsCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setBeltPct(double pct) {
        belts.set(pct);
    }

    public double getBeltPct() {
        return belts.get();
    }

    public Command cmdBeltPct(double pct) {
        return runOnce(() -> {
            setBeltPct(pct);
        });
    }
}

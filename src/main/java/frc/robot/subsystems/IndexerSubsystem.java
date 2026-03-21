package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IndexerSubsystem extends SubsystemBase {
    public final SparkMax mHopper = new SparkMax(Constants.CANConstants.INDEXER_HOPPER, MotorType.kBrushless);
    public final SparkMax mKicker = new SparkMax(Constants.CANConstants.INDEXER_KICKER, MotorType.kBrushless);

    public IndexerSubsystem() {
        var hopperConfig = new SparkMaxConfig()
                .idleMode(IdleMode.kCoast)
                .inverted(true);

        mHopper.configure(hopperConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        var kickerConfig = new SparkMaxConfig()
                .idleMode(IdleMode.kCoast)
                .inverted(true);

        mKicker.configure(kickerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void reverseIndexer() {
        mHopper.set(-1.0);
        mKicker.set(-1.0);
    }

    public void runIndexer() {
        mHopper.set(1.0);
        mKicker.set(1.0);
    }

    public void stopIndexer() {
        mHopper.stopMotor();
        mKicker.stopMotor();
    }

    public Command cRun(BooleanSupplier reverse) {
        return runEnd(() -> {
            if (reverse.getAsBoolean()) {
                this.reverseIndexer();
            } else {
                this.runIndexer();
            }
        }, this::stopIndexer);
    }

}

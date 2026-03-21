package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@Logged
public class ArmSubsystem extends SubsystemBase {
    public final SparkMax mArm = new SparkMax(Constants.CANConstants.INTAKE_ARM, MotorType.kBrushless);

    public ArmSubsystem() {
        var softLimit = new SoftLimitConfig()
                .forwardSoftLimitEnabled(true).forwardSoftLimit(40)
                .reverseSoftLimitEnabled(true).reverseSoftLimit(0);

        var closedLoop = new ClosedLoopConfig()
                .pid(0.05, 0.00001, 0);

        var armConfig = new SparkMaxConfig()
                .inverted(false)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(20)
                .apply(softLimit)
                .apply(closedLoop);

        mArm.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public double getPosition() {
        return mArm.getEncoder().getPosition();
    }

    public double getCurrent() {
        return mArm.getOutputCurrent();
    }

    public double getVelocity() {
        return mArm.getEncoder().getVelocity();
    }

    public enum Target {
        Home(0.0),
        Half(13.0),
        Extended(40.0);

        public double position;

        private Target(double position) {
            this.position = position;
        }
    }

    public Command cRun(Target target) {
        return run(() -> {
            mArm.getClosedLoopController().setSetpoint(target.position, ControlType.kPosition);
        });
    }
}

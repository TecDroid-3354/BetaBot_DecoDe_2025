package org.firstinspires.ftc.teamcode.shooter

import AngularVelocity
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.seattlesolvers.solverslib.hardware.motors.Motor
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.utils.velocityMotorEx.VelocityMotorConfig
import org.firstinspires.ftc.teamcode.utils.velocityMotorEx.VelocityMotorEx

/**
 * Creates a [VelocityMotorConfig] in order to store the desired configurations for the
 * shooter's velocity motor
 */
val shooterMotorConfig = VelocityMotorConfig(
    ShooterConstants.zeroPowerBehavior,
    ShooterConstants.direction,
    ShooterConstants.ticksPerRevolution,
    ShooterConstants.pidController
)
/**
 * This is the code for controlling the shooter wheels on our robot.
 *
 */
@Suppress("JoinDeclarationAndAssignment")
class Shooter(hw: HardwareMap, val telemetry: Telemetry): SubsystemBase() {
    // This is where the motor intended to control the shooter is declared
    val motor: VelocityMotorEx

    // This is the code that will execute when the class is initialized
    init {
        // The shooter motor controller is initialized by creating an instance of Motor and passing a velocity motor config
        motor = VelocityMotorEx(Motor(hw, ShooterConstants.shooterMotorId), shooterMotorConfig)
        // Calls motorConfig()
        motorConfig()
    }

    /**
     * Sets the motor's velocity to a desired angular velocity
     */
    fun setVelocity(velocity: AngularVelocity) {
        motor.setVelocity(velocity)
    }

    /**
     * Calls the super class method for stopping the motor
      */
    fun stop() {
        motor.stopMotor()
    }

    /**
     * Gets the current motor's velocity in degrees / second
     * @return the current velocity in deg / sec
     */
    fun getVelocityInDegPerSec(): Double {
        return motor.getVelocity().degPerSec
    }

    /**
     * Checks if the shooter motor is active and running, useful for logic control
     * @return true if the motor is running
     */
    fun isActive(): Boolean {
        return motor.getVelocity().rotPerSec >= 0.5
    }

    /**
     * Useful method giving the motor desired configurations
     */
    fun motorConfig() {
        // Gives the motor an inverted value
        motor.setInverted(ShooterConstants.isMotorInverted)
    }
}
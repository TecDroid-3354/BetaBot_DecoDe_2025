package org.firstinspires.ftc.teamcode.utils.positionMotorSimple

import Angle
import AngularVelocity
import Distance
import com.seattlesolvers.solverslib.hardware.motors.Motor
import kotlin.math.abs


class PositionMotor(
    private val motor: Motor,
    override var config: PositionMotorConfig
) : IPositionMotor {

    // Defining a few, useful variables
    private var lastPower = 0.0

    /* ! CONFIG METHODS ! */

    init {
        applyConfig()
    }

    // The following functions, applyConfig() allow us to apply a custom configuration in case of needed
    override fun applyConfig() {
        // Setting up the motor & encoder default behavior
        motor.setZeroPowerBehavior(config.zeroPowerBehavior)
        motor.encoder.setDirection(config.direction)

        /// Arranging a P coefficient to achieve desired positions
        motor.positionCoefficient = config.pCoefficient
    }

    override fun applyConfig(config: PositionMotorConfig) {
        // Takes a custom configuration & applies it
        this.config = config
        applyConfig()
    }

    /* ! FUNCTIONAL METHODS ! */

    // Sets raw power using a number between [-1.0, 1.0]
    override fun setPower(power: Double) {
        // Basically, a condition that evaluates that the power passed isn't too low or equal
        // to the past power used
        if ((abs(this.lastPower - power) > config.powerThreshold) || (power == 0.0 && lastPower != 0.0)) {
            lastPower = power
            motor.set(power)
        }
    }

    // The following two methods allow us to set a position based on a given angle according to the parameters
    override fun setPosition(angle: Angle) {
        motor.setTargetPosition((angle.rotations * config.ticksPerRevolution * config.gearRatio).toInt())
    }

    override fun setPosition(distance: Distance, circumference: Distance) {
        // Transforms the distance to rotations
        val rotations = distance.meters / circumference.meters
        setPosition(Angle.fromRotations(rotations))
    }

    // Stops the motor using two instructions
    override fun stopMotor() {
        motor.set(0.0)
        motor.stopMotor()
    }

    /* ! SETTER METHODS ! */

    // This method takes a custom gearRatio & applies it
    override fun setGearRatio(gearRatio: Double) {
        config.gearRatio = gearRatio
    }

    // Takes a custom P coefficient & applies it
    override fun setPCoefficient(p: Double) {
        motor.positionCoefficient = p
    }

    // Changes the direction of a motor
    override fun setDirection(direction: Motor.Direction) {
        motor.encoder.setDirection(direction)
    }

    // The following two functions change the PID tolerance according to the given parameters
    override fun setTolerance(tolerance: Angle) {
        motor.setPositionTolerance(tolerance.rotations * config.ticksPerRevolution * config.gearRatio)
    }

    override fun setTolerance(tolerance: Distance, circumference: Distance) {
        val rotations = tolerance.meters / circumference.meters
        setTolerance(Angle.fromRotations(rotations))
    }

    // Sets the run mode
    override fun setMode(mode: Motor.RunMode) = motor.setRunMode(mode)


    /* ! GETTER METHODS ! */

    // Returns the current position
    override fun getPosition(): Angle =
        Angle.fromRotations(motor.currentPosition / config.ticksPerRevolution * config.gearRatio)

    // Returns the current velocity
    override fun getVelocity(): AngularVelocity =
        AngularVelocity.fromRps(motor.get() / config.ticksPerRevolution * config.gearRatio)

}
package org.firstinspires.ftc.teamcode.utils.positionMotorPENDING

import Angle
import AngularVelocity
import Distance
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import kotlin.math.abs


class PositionMotor(
    private val motor: DcMotorEx,
    override var config: PositionMotorConfig
) : IPositionMotor {

    private var lastPower = 0.0
    private var gearRatio = 1.0

    init {
        applyConfig()
    }

    override fun applyConfig() {
        val p = config.pCoefficient
        motor.zeroPowerBehavior = config.zeroPowerBehavior
        motor.direction = config.direction
        motor.setPositionPIDFCoefficients(p)
    }

    override fun applyConfig(config: PositionMotorConfig) {
        this.config = config
        applyConfig()
    }

    override fun setGearRatio(gearRatio: Double) {
        this.gearRatio = gearRatio
    }

    override fun setPCoefficient(p: Double) {
        motor.setPositionPIDFCoefficients(p)
    }

    override fun setPower(power: Double) {
        if ((abs(this.lastPower - power) > config.powerThreshold) || (power == 0.0 && lastPower != 0.0)) {
            lastPower = power
            motor.power = power
        }
    }

    override fun setPosition(angle: Angle) {
        motor.targetPosition = (angle.rotations * config.ticksPerRevolution * gearRatio).toInt()
    }

    override fun setPosition(distance: Distance, circumference: Distance) {
        val rotations = distance.meters / circumference.meters

        setPosition(Angle.fromRotations(rotations))
    }

    override fun setDirection(direction: DcMotorSimple.Direction) {
        motor.direction = direction
    }

    override fun setTolerance(tolerance: Angle) {
        motor.targetPositionTolerance = (tolerance.rotations * config.ticksPerRevolution * gearRatio).toInt()
    }

    override fun setTolerance(tolerance: Distance, circumference: Distance) {
        val rotations = tolerance.meters / circumference.meters

        setTolerance(Angle.fromRotations(rotations))
    }

    override fun getPosition(): Angle {
        return Angle.fromRotations(motor.currentPosition / config.ticksPerRevolution * gearRatio)
    }

    override fun getVelocity(): AngularVelocity = AngularVelocity.fromRps(motor.velocity / config.ticksPerRevolution * gearRatio)

    override fun setMode(mode: DcMotor.RunMode) {
        motor.setMode(mode)
    }
}
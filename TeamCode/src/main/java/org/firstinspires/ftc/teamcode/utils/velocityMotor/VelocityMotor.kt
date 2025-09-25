package org.firstinspires.ftc.teamcode.utils.velocityMotorEx

import Angle
import AngularVelocity
import Distance
import LinearVelocity
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import kotlin.math.abs


class VelocityMotorEx(
    private val motor: DcMotorEx,
    override var config: VelocityMotorConfig
) : IVelocityMotorEx {

    private var lastPower = 0.0
    private var gearRatio = 1.0

    private var wheelCircumference = Distance.fromCm(7.5)

    init {
        applyConfig()
    }

    override fun applyConfig() {
        val coefficients = config.pidfCoefficients
        motor.zeroPowerBehavior = config.zeroPowerBehavior
        motor.direction = config.direction
        motor.setVelocityPIDFCoefficients(coefficients.p, coefficients.i, coefficients.d, coefficients.f)
    }

    override fun applyConfig(config: VelocityMotorConfig) {
        this.config = config
        applyConfig()
    }

    override fun setGearRatio(gearRatio: Double) {
        this.gearRatio = gearRatio
    }

    override fun setPIDFCoefficients(pidfCoefficients: PIDFCoefficients) {
        motor.setVelocityPIDFCoefficients(pidfCoefficients.p, pidfCoefficients.i, pidfCoefficients.d, pidfCoefficients.f)
    }

    override fun setPower(power: Double) {
        if ((abs(this.lastPower - power) > config.powerThreshold) || (power == 0.0 && lastPower != 0.0)) {
            lastPower = power
            motor.power = power
        }
    }

    override fun setVelocity(angularVelocity: AngularVelocity) {
        motor.velocity = angularVelocity.rps * config.ticksPerRevolution * gearRatio
    }

    override fun setVelocity(linearVelocity: LinearVelocity) {
        val rps = linearVelocity.mps / wheelCircumference.meters

        setVelocity(AngularVelocity.fromRps(rps))
    }

    override fun setVelocity(linearVelocity: LinearVelocity, circumference: Distance) {
        val rps = linearVelocity.mps / circumference.meters

        setVelocity(AngularVelocity.fromRps(rps))
    }

    override fun setDirection(direction: DcMotorSimple.Direction) {
        motor.direction = direction
    }

    override fun setCircumference(circumference: Distance) {
        wheelCircumference = circumference
    }

    override fun getPosition(): Angle {
        return Angle.fromRotations(motor.currentPosition / config.ticksPerRevolution * gearRatio)
    }

    override fun getVelocity(): AngularVelocity = AngularVelocity.fromRps(motor.velocity / config.ticksPerRevolution * gearRatio)

    override fun getLinearVelocity(): LinearVelocity =
        LinearVelocity.fromMps(wheelCircumference.meters * (motor.velocity / config.ticksPerRevolution * gearRatio))

    override fun getLinearVelocity(circumference: Distance): LinearVelocity =
        LinearVelocity.fromMps(circumference.meters * (motor.velocity / config.ticksPerRevolution * gearRatio))

    override fun setMode(mode: DcMotor.RunMode) {
        motor.setMode(mode)
    }
}
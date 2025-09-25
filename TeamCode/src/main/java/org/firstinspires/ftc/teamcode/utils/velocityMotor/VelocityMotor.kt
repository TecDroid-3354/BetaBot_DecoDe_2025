package org.firstinspires.ftc.teamcode.utils.velocityMotor

import Angle
import AngularVelocity
import Distance
import LinearVelocity
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.seattlesolvers.solverslib.controller.wpilibcontroller.SimpleMotorFeedforward
import kotlin.math.abs


class VelocityMotor(
    private val motor: DcMotorEx,
    override var config: VelocityMotorConfig
) : IVelocityMotor {

    private var lastPower = 0.0
    private var gearRatio = 1.0

    private var wheelCircumference = Distance.fromCm(7.5)

    private val feedforward: SimpleMotorFeedforward = SimpleMotorFeedforward(config.svaCoefficients.kS, config.svaCoefficients.kV, config.svaCoefficients.kA)

    init {
        applyConfig()
    }

    override fun applyConfig() {
        val coefficients = config.svaCoefficients
        motor.zeroPowerBehavior = config.zeroPowerBehavior
        motor.direction = config.direction
        setSVACoefficients(config.svaCoefficients)
    }

    override fun applyConfig(config: VelocityMotorConfig) {
        this.config = config
        applyConfig()
    }

    override fun setGearRatio(gearRatio: Double) {
        this.gearRatio = gearRatio
    }

    override fun setSVACoefficients(svaCoefficients: SVACoefficients) {
        feedforward.ks = svaCoefficients.kS
        feedforward.kv = svaCoefficients.kV
        feedforward.ka = svaCoefficients.kA
    }

    override fun setPower(power: Double) {
        if ((abs(this.lastPower - power) > config.powerThreshold) || (power == 0.0 && lastPower != 0.0)) {
            lastPower = power
            motor.power = power
        }
    }

    override fun setVelocity(angularVelocity: AngularVelocity) {
        setPower(feedforward.calculate(angularVelocity.rps))
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
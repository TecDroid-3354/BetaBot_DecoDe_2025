package org.firstinspires.ftc.teamcode.utils.positionMotorEx

import Angle
import AngularVelocity
import Distance
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.seattlesolvers.solverslib.controller.PIDFController
import kotlin.math.abs


class PositionMotorEx(
    private val motor: DcMotorEx,
    override var config: PositionMotorExConfig
) : IPositionMotorEx {

    private var pidfController: PIDFController = PIDFController(config.pidfCoefficients)
    private var lastPower = 0.0
    private var gearRatio = 1.0

    init {
        applyConfig()
    }

    override fun applyConfig() {
        motor.zeroPowerBehavior = config.zeroPowerBehavior
        motor.direction = config.direction
        setPIDFCoefficients(config.pidfCoefficients)
    }

    override fun applyConfig(config: PositionMotorExConfig) {
        this.config = config
        applyConfig()
    }

    override fun setGearRatio(gearRatio: Double) {
        this.gearRatio = gearRatio
    }

    override fun setPIDFCoefficients(pidfCoefficients: PIDFCoefficients) {
        pidfController.setPIDF(pidfCoefficients.p, pidfCoefficients.i, pidfCoefficients.d, pidfCoefficients.f)
    }

    override fun setPower(power: Double) {
        if ((abs(this.lastPower - power) > config.powerThreshold) || (power == 0.0 && lastPower != 0.0)) {
            lastPower = power
            motor.power = power
        }
    }

    override fun setPosition(setPoint: Angle) {
        pidfController.setPoint = setPoint.rotations

        while (!pidfController.atSetPoint()) {
            val voltage = pidfController.calculate(getPosition().rotations)
            setPower(voltage)
        }

        setPower(0.0)
    }

    override fun setPosition(distance: Distance, circumference: Distance) {
        val rotations = distance.meters / circumference.meters

        setPosition(Angle.fromRotations(rotations))
    }

    override fun setDirection(direction: DcMotorSimple.Direction) {
        motor.direction = direction
    }

    override fun setPidfTolerance(tolerance: Angle) {
        pidfController.setTolerance(tolerance.rotations)
    }

    override fun setPidfTolerance(tolerance: Distance, circumference: Distance) {
        val rotations = tolerance.meters / circumference.meters
        setPidfTolerance(Angle.fromRotations(rotations))
    }

    override fun getPosition(): Angle {
        return Angle.fromRotations(motor.currentPosition / config.ticksPerRevolution * gearRatio)
    }

    override fun getVelocity(): AngularVelocity = AngularVelocity.fromRps(motor.velocity / config.ticksPerRevolution * gearRatio)

    override fun setMode(mode: DcMotor.RunMode) {
        motor.setMode(mode)
    }
}
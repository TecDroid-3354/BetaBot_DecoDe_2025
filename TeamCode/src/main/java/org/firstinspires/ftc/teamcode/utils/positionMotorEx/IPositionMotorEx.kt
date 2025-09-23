package org.firstinspires.ftc.teamcode.utils.positionMotorEx

import Angle
import AngularVelocity
import Distance
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.PIDFCoefficients

data class PositionMotorExConfig(
    val zeroPowerBehavior: DcMotor.ZeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT,
    val direction: DcMotorSimple.Direction = DcMotorSimple.Direction.FORWARD,
    val ticksPerRevolution: Double = 1.0,
    val pidfCoefficients: PIDFCoefficients,
    val powerThreshold: Double = 0.01
)

interface IPositionMotorEx {
    var config: PositionMotorExConfig

    fun setPower(power: Double)
    fun setPosition(setPoint: Angle)
    fun setPosition(distance: Distance, circumference: Distance)
    fun setDirection(direction: DcMotorSimple.Direction)
    fun setPidfTolerance(tolerance: Angle)
    fun setPidfTolerance(tolerance: Distance, circumference: Distance)
    fun getPosition(): Angle
    fun getVelocity(): AngularVelocity
    fun setMode(mode: DcMotor.RunMode)
    fun applyConfig()
    fun applyConfig(config: PositionMotorExConfig)

    fun setGearRatio(gearRatio: Double)

    fun setPIDFCoefficients(pidfCoefficients: PIDFCoefficients)
}
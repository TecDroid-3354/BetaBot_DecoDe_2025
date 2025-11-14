package org.firstinspires.ftc.teamcode.utils.positionMotorPENDING

import Angle
import AngularVelocity
import Distance
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple

data class PositionMotorConfig(
    val zeroPowerBehavior: DcMotor.ZeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT,
    val direction: DcMotorSimple.Direction = DcMotorSimple.Direction.FORWARD,
    val ticksPerRevolution: Double = 1.0,
    val pCoefficient: Double,
    val powerThreshold: Double = 0.01
)

interface IPositionMotor {
    var config: PositionMotorConfig

    fun setPower(power: Double)
    fun setPosition(angle: Angle)
    fun setPosition(distance: Distance, circumference: Distance)
    fun setDirection(direction: DcMotorSimple.Direction)
    fun setTolerance(tolerance: Angle)
    fun setTolerance(tolerance: Distance, circumference: Distance)
    fun getPosition(): Angle
    fun getVelocity(): AngularVelocity
    fun setMode(mode: DcMotor.RunMode)
    fun applyConfig()
    fun applyConfig(config: PositionMotorConfig)

    fun setGearRatio(gearRatio: Double)

    fun setPCoefficient(p: Double)
}
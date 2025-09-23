package org.firstinspires.ftc.teamcode.utils

import Angle
import AngularVelocity
import Distance
import LinearVelocity
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.seattlesolvers.solverslib.hardware.motors.Motor

data class MotorConfig(
    val zeroPowerBehavior: DcMotor.ZeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT,
    val direction: DcMotorSimple.Direction = DcMotorSimple.Direction.FORWARD,
    val ticksPerRevolution: Double = 1.0,
    val pidfCoefficients: PIDFCoefficients = PIDFCoefficients(1.0, 0.0, 0.0, 0.0),
    val powerThreshold: Double = 0.01
)

interface IMotorEx {
    var config: MotorConfig

    fun setPower(power: Double)
    fun setVelocity(angularVelocity: AngularVelocity)
    fun setVelocity(linearVelocity: LinearVelocity, circumference: Distance)
    fun setDirection(direction: DcMotorSimple.Direction)
    fun getPosition(): Angle
    fun getVelocity(): AngularVelocity
    fun setMode(mode: DcMotor.RunMode)
    fun applyConfig()
    fun applyConfig(config: MotorConfig)

    fun setGearRatio(gearRatio: Double)

    fun setPIDFCoefficients(pidfCoefficients: PIDFCoefficients)
}
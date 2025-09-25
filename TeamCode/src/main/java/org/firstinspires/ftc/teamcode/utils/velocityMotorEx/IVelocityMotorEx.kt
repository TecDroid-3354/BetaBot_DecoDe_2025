package org.firstinspires.ftc.teamcode.utils.velocityMotorEx

import Angle
import AngularVelocity
import Distance
import LinearVelocity
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.PIDFCoefficients

data class VelocityMotorExConfig(
    val zeroPowerBehavior: DcMotor.ZeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT,
    val direction: DcMotorSimple.Direction = DcMotorSimple.Direction.FORWARD,
    val ticksPerRevolution: Double = 1.0,
    val pidfCoefficients: PIDFCoefficients = PIDFCoefficients(1.0, 0.0, 0.0, 0.0),
    val powerThreshold: Double = 0.01
)

interface IVelocityMotorEx {
    var config: VelocityMotorExConfig

    fun setPower(power: Double)
    fun setVelocity(angularVelocity: AngularVelocity)
    fun setVelocity(linearVelocity: LinearVelocity)
    fun setVelocity(linearVelocity: LinearVelocity, circumference: Distance)
    fun setDirection(direction: DcMotorSimple.Direction)
    fun setMode(mode: DcMotor.RunMode)
    fun setCircumference(circumference: Distance)
    fun getPosition(): Angle
    fun getVelocity(): AngularVelocity
    fun getLinearVelocity(): LinearVelocity
    fun getLinearVelocity(circumference: Distance): LinearVelocity
    fun applyConfig()
    fun applyConfig(config: VelocityMotorExConfig)

    fun setGearRatio(gearRatio: Double)

    fun setPIDFCoefficients(pidfCoefficients: PIDFCoefficients)
}
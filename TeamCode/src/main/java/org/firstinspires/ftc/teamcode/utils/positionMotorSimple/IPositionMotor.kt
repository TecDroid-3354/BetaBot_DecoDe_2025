package org.firstinspires.ftc.teamcode.utils.positionMotorSimple

import Angle
import AngularVelocity
import Distance
import com.seattlesolvers.solverslib.hardware.motors.Motor

data class PositionMotorConfig(
    val zeroPowerBehavior: Motor.ZeroPowerBehavior = Motor.ZeroPowerBehavior.FLOAT,
    val direction: Motor.Direction = Motor.Direction.FORWARD,
    val ticksPerRevolution: Double = 1.0,
    val pCoefficient: Double,
    var gearRatio: Double = 1.0,
    val powerThreshold: Double = 0.01
)

interface IPositionMotor {
    /* ! CONFIG METHODS ! */
    var config: PositionMotorConfig
    fun applyConfig()
    fun applyConfig(config: PositionMotorConfig)

    /* ! FUNCTIONAL METHODS ! */
    fun setPower(power: Double)
    fun setPosition(angle: Angle)
    fun setPosition(distance: Distance, circumference: Distance)
    fun stopMotor()

    /* ! SETTER METHODS ! */
    fun setGearRatio(gearRatio: Double)
    fun setPCoefficient(p: Double)
    fun setDirection(direction: Motor.Direction)
    fun setTolerance(tolerance: Angle)
    fun setTolerance(tolerance: Distance, circumference: Distance)
    fun setMode(mode: Motor.RunMode)

    /* ! GETTER METHODS !*/
    fun getPosition(): Angle
    fun getVelocity(): AngularVelocity
}
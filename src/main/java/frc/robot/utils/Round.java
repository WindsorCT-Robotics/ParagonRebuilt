package frc.robot.utils;

public class Round {
    public static double round(double value, int decimalPlace) {
        double decimal = Math.pow(10, decimalPlace);
        return ((double) Math.round(value * decimal)) / decimal;
    }   
}
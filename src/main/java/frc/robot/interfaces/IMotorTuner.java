package frc.robot.interfaces;

import edu.wpi.first.units.measure.Voltage;

public interface IMotorTuner {
    public void updateP(double value);

    public double getP();

    public void updateI(double value);

    public double getI();

    public void updateD(double value);

    public double getD();

    public void updateKS(double value);

    public Voltage getKS();

    public void updateKV(double value);

    public Voltage getKV();

    public void updateKA(double value);

    public Voltage getKA();

    public void updateKG(double value);

    public Voltage getKG();

    public void updateKCos(double value);

    public Voltage getKCos();

    public void updateKCosRatio(double value);

    public double getKCosRatio();
}
package frc.robot.Util;

public class Gains {
    public double P = 0;
	public double I = 0;
	public double D = 0;
	public double F = 0;
	public double S = 0;
	public double V = 0;
	public int Izone = 0;
	public double peakOutput = 0;

	/** 
     * Class for PID gains
     * @param P P gain
     * @param I I gain
     * @param D D gain
     * @param F FF gain
     * @param Izone I zone
     * @param PeakOutput Peak Output
     */
	public Gains(double P, double I, double D, double F, int Izone, double PeakOutput){
		this.P = P;
		this.I = I;
		this.D = D;
		this.F = F;
		this.Izone = Izone;
		this.peakOutput = PeakOutput;
	}

	public Gains(double P, double D, double S, double V, double peakOutput){
		this.P = P;
		this.D = D;
		this.S = S;
		this.V = V;
		this.peakOutput = peakOutput;
	}

	public Gains(double P, double peakOutput){
		this.P = P;
		this.peakOutput = peakOutput;
	}
}

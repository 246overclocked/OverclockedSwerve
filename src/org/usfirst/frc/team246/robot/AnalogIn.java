package org.usfirst.frc.team246.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.livewindow.LiveWindowSendable;
import edu.wpi.first.wpilibj.tables.ITable;

public class AnalogIn implements PIDSource, LiveWindowSendable {
	
	public boolean onRIO;
	public AnalogInput input;
	public double currentVal = 0;
	
	public AnalogIn(int channel, boolean onRIO)
	{
		this.onRIO = onRIO;
		if(onRIO)
		{
			input = new AnalogInput(channel);
			input.setAverageBits(3);
		}
//		Used to be used for BBBAnalogs
//		else
//		{
//
//		}
	}
	
	public double get()
	{
		if(onRIO)
		{
			return input.getAverageVoltage();
		}
		else
		{
			return currentVal;
		}
	}
	
	public void updateVal(double val)
	{
		currentVal = val;
	}

	private ITable m_table;

	/**
	 * {@inheritDoc}
	 */
	public void initTable(ITable subtable) {
		m_table = subtable;
		updateTable();
	}

	/**
	 * {@inheritDoc}
	 */
	public void updateTable() {
		if (m_table != null) {
			m_table.putNumber("Value", get());
		}
	}

	/**
	 * {@inheritDoc}
	 */
	public ITable getTable() {
		return m_table;
	}

	/**
	 * Analog Channels don't have to do anything special when entering the
	 * LiveWindow. {@inheritDoc}
	 */
	public void startLiveWindowMode() {
	}

	/**
	 * Analog Channels don't have to do anything special when exiting the
	 * LiveWindow. {@inheritDoc}
	 */
	public void stopLiveWindowMode() {
	}

	@Override
	public double pidGet() {
		return get();
	}

	@Override
	public String getSmartDashboardType() {
		return "Analog Input";
	}

}

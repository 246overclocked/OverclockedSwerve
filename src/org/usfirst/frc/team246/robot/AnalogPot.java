/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008-2014. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package org.usfirst.frc.team246.robot;

import edu.wpi.first.wpilibj.ControllerPower;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import edu.wpi.first.wpilibj.livewindow.LiveWindowSendable;
import edu.wpi.first.wpilibj.tables.ITable;

/**
 * Class for reading analog potentiometers. Analog potentiometers read
 * in an analog voltage that corresponds to a position. The position is
 * in whichever units you choose, by way of the scaling and offset
 * constants passed to the constructor. 
 *
 * @author Alex Henning
 * @author Colby Skeggs (rail voltage)
 */
public class AnalogPot implements Potentiometer, LiveWindowSendable {
    private double m_fullRange, m_offset;
    private AnalogIn m_analog_input;
    private boolean m_init_analog_input;

    /**
     * Common initialization code called by all constructors.
     * @param input The {@link AnalogInput} this potentiometer is plugged into.
     * @param fullRange The scaling to multiply the voltage by to get a meaningful unit.
     * @param offset The offset to add to the scaled value for controlling the zero value
     */
    private void initPot(final AnalogIn input, double fullRange, double offset) {
        this.m_fullRange = fullRange;
        this.m_offset = offset;
        m_analog_input = input;
    }

    /**
     * AnalogPotentiometer constructor.
     *
     * Use the fullRange and offset values so that the output produces
     * meaningful values. I.E: you have a 270 degree potentiometer and
     * you want the output to be degrees with the halfway point as 0
     * degrees. The fullRange value is 270.0(degrees) and the offset is
     * -135.0 since the halfway point after scaling is 135 degrees.
     * 
     * This will calculate the result from the fullRange times the
     * fraction of the supply voltage, plus the offset.
     *
     * @param channel The analog channel this potentiometer is plugged into.
     * @param fullRange The scaling to multiply the fraction by to get a meaningful unit.
     * @param offset The offset to add to the scaled value for controlling the zero value
     */
    public AnalogPot(final int channel, double fullRange, double offset, boolean onRIO) {
    	AnalogIn input = new AnalogIn(channel, onRIO);
    	m_init_analog_input = true;
        initPot(input, fullRange, offset);
    }

    /**
     * AnalogPotentiometer constructor.
     *
     * Use the fullRange and offset values so that the output produces
     * meaningful values. I.E: you have a 270 degree potentiometer and
     * you want the output to be degrees with the halfway point as 0
     * degrees. The fullRange value is 270.0(degrees) and the offset is
     * -135.0 since the halfway point after scaling is 135 degrees.
     * 
     * This will calculate the result from the fullRange times the
     * fraction of the supply voltage, plus the offset.
     *
     * @param input The {@link AnalogInput} this potentiometer is plugged into.
     * @param fullRange The scaling to multiply the fraction by to get a meaningful unit.
     * @param offset The offset to add to the scaled value for controlling the zero value
     */
    public AnalogPot(final AnalogIn input, double fullRange, double offset) {
    	m_init_analog_input = false;
    	initPot(input, fullRange, offset);
    }

    /**
     * AnalogPotentiometer constructor.
     *
     * Use the fullRange and offset values so that the output produces
     * meaningful values. I.E: you have a 270 degree potentiometer and
     * you want the output to be degrees with the halfway point as 0
     * degrees. The fullRange value is 270.0(degrees) and the offset is
     * -135.0 since the halfway point after scaling is 135 degrees.
     *
     * @param channel The analog channel this potentiometer is plugged into.
     * @param scale The scaling to multiply the voltage by to get a meaningful unit.
     */
    public AnalogPot(final int channel, double scale, boolean onRIO) {
        this(channel, scale, 0, onRIO);
    }
    
    /**
     * AnalogPotentiometer constructor.
     *
     * Use the fullRange and offset values so that the output produces
     * meaningful values. I.E: you have a 270 degree potentiometer and
     * you want the output to be degrees with the halfway point as 0
     * degrees. The fullRange value is 270.0(degrees) and the offset is
     * -135.0 since the halfway point after scaling is 135 degrees.
     *
     * @param input The {@link AnalogInput} this potentiometer is plugged into.
     * @param scale The scaling to multiply the voltage by to get a meaningful unit.
     */
    public AnalogPot(final AnalogIn input, double scale) {
        this(input, scale, 0);
    }

    /**
     * AnalogPotentiometer constructor.
     *
     * @param channel The analog channel this potentiometer is plugged into.
     */
    public AnalogPot(final int channel, boolean onRIO) {
    	this(channel, 1, 0, onRIO);
    }

    /**
     * AnalogPotentiometer constructor.
     *
     * @param input The {@link AnalogInput} this potentiometer is plugged into.
     */
    public AnalogPot(final AnalogIn input) {
    	this(input, 1, 0);
    }

    /**
     * Get the current reading of the potentiometer.
     *
     * @return The current position of the potentiometer.
     */
    public double get() {
    	if(m_analog_input.onRIO)
    	{
    		return (m_analog_input.get() / ControllerPower.getVoltage5V()) * m_fullRange + m_offset;
    	}
    	else
    	{
    		return m_analog_input.get() * m_fullRange + m_offset;
    	}
    }
    
    public double getRaw() {
    	if(m_analog_input.onRIO)
    	{
    		return m_analog_input.get() / ControllerPower.getVoltage5V();
    	}
    	else
    	{
    		return m_analog_input.get();
    	}
    }

    /**
     * Implement the PIDSource interface.
     *
     * @return The current reading.
     */
    public double pidGet() {
        return get();
    }


    /**
     * Live Window code, only does anything if live window is activated.
     */
    public String getSmartDashboardType(){
        return "Analog Input";
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
    public ITable getTable(){
        return m_table;
    }

    /**
     * Analog Channels don't have to do anything special when entering the LiveWindow.
     * {@inheritDoc}
     */
    public void startLiveWindowMode() {}

    /**
     * Analog Channels don't have to do anything special when exiting the LiveWindow.
     * {@inheritDoc}
     */
    public void stopLiveWindowMode() {}
}

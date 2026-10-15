/*
 * *****************************************************************************
 * Copyright (C) 2014-2023 Dennis Sheirer
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 * ****************************************************************************
 */
package io.github.dsheirer.source.tuner.rtl.r8x.r820t;

import io.github.dsheirer.source.SourceException;
import io.github.dsheirer.source.tuner.TunerType;
import io.github.dsheirer.source.tuner.rtl.RTL2832TunerController;
import io.github.dsheirer.source.tuner.rtl.r8x.R8xEmbeddedTuner;
import javax.usb.UsbException;

/**
 * Rafael Micro R820T and R820T2 Embedded Tuner implementation,
 * with dynamic support for RTL-SDR Blog V4L and standard R820T devices.
 */
public class R820TEmbeddedTuner extends R8xEmbeddedTuner
{
    private static final byte I2C_WRITE_ADDRESS = (byte) 0x34;
    private static final byte I2C_READ_ADDRESS = (byte) 0x35;

    private static final long UPCONVERTER_OFFSET_HZ = 28800000L;
    private static final int BAND_HF = 1;
    private static final int BAND_UHF = 3;
    private int currentInputBand = 0;

    /**
     * Constructs an instance.
     * Dynamic VCO power reference: 1 for V4L, 2 for standard R820T/R820T2.
     *
     * @param adapter for accessing RTL2832USBController interfaces
     */
    public R820TEmbeddedTuner(RTL2832TunerController.ControllerAdapter adapter)
    {
        super(adapter, adapter.isV4LDongle() ? 1 : 2);
    }

    @Override
    public TunerType getTunerType()
    {
        return TunerType.RAFAELMICRO_R820T;
    }

    @Override
    public byte getI2CWriteAddress()
    {
        return I2C_WRITE_ADDRESS;
    }

    @Override
    public byte getI2CReadAddress()
    {
        return I2C_READ_ADDRESS;
    }

    @Override
    protected void setPLL(long frequency, boolean controlI2C) throws UsbException
    {
        try
        {
            super.setPLL(frequency, controlI2C);
        }
        catch (UsbException e)
        {
            if (!getAdapter().isV4LDongle())
            {
                throw e;
            }
        }

        // Apply V4L VCO max current adjustment if this is a V4L
        if (getAdapter().isV4LDongle())
        {
            getAdapter().writeI2CRegister(getI2CWriteAddress(), (byte) 0x12, (byte) 0x06, controlI2C);
        }
    }

    /**
     * Sets the center frequency. Setting the frequency is a two-part process
     * of setting the multiplexer and then setting the Oscillator (PLL).
     */
    @Override
    public synchronized void setTunedFrequency(long frequency) throws SourceException
    {
        getAdapter().getLock().lock();

        try
        {
            getAdapter().enableI2CRepeater();
            boolean controlI2C = false;

            long tuneFrequency = frequency;
            boolean isV4L = getAdapter().isV4LDongle();

            // V4L automatic HF upconversion
            if (isV4L && frequency < UPCONVERTER_OFFSET_HZ)
            {
                tuneFrequency += UPCONVERTER_OFFSET_HZ;
            }

            long offsetFrequency = tuneFrequency + IF_FREQUENCY;
            setMux(offsetFrequency, controlI2C);
            setPLL(offsetFrequency, controlI2C);

            // V4L RF path and GPIO 5 switching
            if (isV4L)
            {
                int band = (frequency <= UPCONVERTER_OFFSET_HZ) ? BAND_HF : BAND_UHF;

                if (band == BAND_HF)
                {
                    writeRegister(Register.RF_POLY_MUX, (byte) 0x40, controlI2C);
                    writeRegister(Register.TF_BAND, (byte) 0x00, controlI2C);
                }

                if (band != currentInputBand)
                {
                    currentInputBand = band;

                    byte cable1In = (byte) ((band == BAND_HF) ? 0x40 : 0x00);
                    getAdapter().setGpioPin(5, (cable1In == 0));
                    writeRegister(Register.INPUT_SELECTOR_CABLE_1, cable1In, controlI2C);

                    byte airIn = (byte) ((band == BAND_UHF) ? 0x00 : 0x20);
                    writeRegister(Register.INPUT_SELECTOR_AIR, airIn, controlI2C);
                }
            }

            getAdapter().disableI2CRepeater();
        }
        catch(Exception e)
        {
            throw new SourceException("R820TEmbeddedTuner error", e);
        }
        finally
        {
            getAdapter().getLock().unlock();
        }
    }
}

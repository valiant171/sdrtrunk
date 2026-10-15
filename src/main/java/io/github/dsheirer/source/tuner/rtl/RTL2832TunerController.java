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
package io.github.dsheirer.source.tuner.rtl;

import io.github.dsheirer.buffer.ByteNativeBufferFactory;
import io.github.dsheirer.buffer.INativeBufferFactory;
import io.github.dsheirer.source.SourceException;
import io.github.dsheirer.source.tuner.ITunerErrorListener;
import io.github.dsheirer.source.tuner.TunerClass;
import io.github.dsheirer.source.tuner.TunerFactory;
import io.github.dsheirer.source.tuner.TunerType;
import io.github.dsheirer.source.tuner.configuration.TunerConfiguration;
import io.github.dsheirer.source.tuner.usb.USBTunerController;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.Charset;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.concurrent.locks.ReentrantLock;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.usb4java.DeviceHandle;
import org.usb4java.LibUsb;
import org.usb4java.LibUsbException;

import javax.usb.UsbDisconnectedException;
import javax.usb.UsbException;

/**
 * RTL-2832 tuner controller implementation.
 */
public class RTL2832TunerController extends USBTunerController
{
    private static final Logger mLog = LoggerFactory.getLogger(RTL2832TunerController.class);

    public static final int TWO_TO_22_POWER = 4194304;
    public static final int USB_TRANSFER_BUFFER_SIZE = 65_536;
    public static final byte CONTROL_ENDPOINT_IN = (byte) (LibUsb.ENDPOINT_IN | LibUsb.REQUEST_TYPE_VENDOR);
    public static final byte CONTROL_ENDPOINT_OUT = (byte) (LibUsb.ENDPOINT_OUT | LibUsb.REQUEST_TYPE_VENDOR);
    public static final long TIMEOUT_US = 1000000l; //uSeconds
    public static final byte REQUEST_ZERO = (byte) 0;
    public static final byte EEPROM_ADDRESS = (byte) 0xA0;
    public static final byte[] FIR_FILTER_COEFFICIENTS = {(byte) 0xCA, (byte) 0xDC, (byte) 0xD7, (byte) 0xD8,
            (byte) 0xE0, (byte) 0xF2, (byte) 0x0E, (byte) 0x35, (byte) 0x06, (byte) 0x50, (byte) 0x9C, (byte) 0x0D,
            (byte) 0x71, (byte) 0x11, (byte) 0x14, (byte) 0x71, (byte) 0x74, (byte) 0x19, (byte) 0x41, (byte) 0xA5};
    public static final SampleRate DEFAULT_SAMPLE_RATE = SampleRate.RATE_2_400MHZ;
    private SampleRate mSampleRate = DEFAULT_SAMPLE_RATE;
    private INativeBufferFactory mNativeBufferFactory = new ByteNativeBufferFactory();
    private int mOscillatorFrequency = 28800000; //28.8 MHz
    private Descriptor mDescriptor;
    private EmbeddedTuner mEmbeddedTuner;
    private long mTunedFrequency = 0;
    private boolean mBiasTEnabled = false;

    public RTL2832TunerController(int bus, String portAddress, ITunerErrorListener tunerErrorListener)
    {
        super(bus, portAddress, tunerErrorListener);
    }

    @Override
    protected INativeBufferFactory getNativeBufferFactory()
    {
        return mNativeBufferFactory;
    }

    @Override
    protected int getTransferBufferSize()
    {
        return USB_TRANSFER_BUFFER_SIZE;
    }

    @Override
    public int getBufferSampleCount()
    {
        return getTransferBufferSize() / 2;
    }

    public EmbeddedTuner getEmbeddedTuner()
    {
        return mEmbeddedTuner;
    }

    public boolean hasEmbeddedTuner()
    {
        return mEmbeddedTuner != null;
    }

    @Override
    public void setTunedFrequency(long frequency) throws SourceException
    {
        getEmbeddedTuner().setTunedFrequency(frequency);
        mTunedFrequency = frequency;
    }

    @Override
    public long getTunedFrequency() throws SourceException
    {
        return mTunedFrequency;
    }

    @Override
    public void apply(TunerConfiguration tunerConfig) throws SourceException
    {
        super.apply(tunerConfig);

        if(tunerConfig instanceof RTL2832TunerConfiguration rtl)
        {
            setSampleRate(rtl.getSampleRate());
            getEmbeddedTuner().apply(rtl);
        }
    }

    @Override
    protected void deviceStart() throws SourceException
    {
        boolean resetRequired = false;

        try
        {
            writeRegister(Block.USB, Address.USB_SYSCTL, 0x09, 1);
        }
        catch(LibUsbException lue)
        {
            if(lue.getErrorCode() < 0)
            {
                resetRequired = true;
            }
            else
            {
                mLog.error("Error performing test write to RTL2832 device - " + LibUsb.errorName(lue.getErrorCode()));
            }
        }

        if(resetRequired)
        {
            int status = LibUsb.resetDevice(getDeviceHandle());
            mLog.info("Resetting device - status: " + LibUsb.errorName(status));

            try
            {
                writeRegister(Block.USB, Address.USB_SYSCTL, 0x09, 1);
            }
            catch(LibUsbException lue)
            {
                mLog.error("Couldnt' reset RTL2832 device - setting error");
                throw new SourceException("unable to reset USB device - " + LibUsb.errorName(status));
            }
        }

        byte[] eeprom = null;

        try
        {
            eeprom = readEEPROM((short) 0, 256);
        }
        catch(Exception e)
        {
            mLog.error("error while reading the EEPROM device descriptor", e);
        }

        try
        {
            mDescriptor = new Descriptor(eeprom);

            if(eeprom == null)
            {
                mLog.error("eeprom byte array was null - constructed empty descriptor object");
            }
        }
        catch(Exception e)
        {
            mLog.error("error while constructing device descriptor using descriptor byte array " +
                (eeprom == null ? "[null]" : Arrays.toString(eeprom)), e);
        }

        initBaseband();

        TunerType tunerType = identifyTunerType();

        if(tunerType == TunerType.UNKNOWN)
        {
            throw new SourceException("Unrecognized RTL-2832 embedded tuner type: " + tunerType);
        }

        mEmbeddedTuner = TunerFactory.getRtlEmbeddedTuner(tunerType, new ControllerAdapter(this));

        try
        {
            enableI2CRepeater(true);
            getEmbeddedTuner().initTuner();
            enableI2CRepeater(false);
        }
        catch(UsbException ue)
        {
            throw new SourceException("Unable to initialize " + tunerType, ue);
        }

        setMinimumFrequency(mEmbeddedTuner.getMinimumFrequencySupported());
        setMaximumFrequency(mEmbeddedTuner.getMaximumFrequencySupported());
        setMiddleUnusableHalfBandwidth(mEmbeddedTuner.getDcSpikeHalfBandwidth());
        setUsableBandwidthPercentage(mEmbeddedTuner.getUsableBandwidthPercent());

        try
        {
            setSampleRate(DEFAULT_SAMPLE_RATE);
        }
        catch(Exception e)
        {
            throw new SourceException("RTL2832 Tuner Controller - couldn't set default sample rate", e);
        }
    }

    @Override
    protected void deviceStop()
    {
        try
        {
            deinitBaseband();
        }
        catch(Exception e)
        {
            //No-op
        }
    }

    public Descriptor getDescriptor()
    {
        if(mDescriptor != null && mDescriptor.isValid())
        {
            return mDescriptor;
        }

        return null;
    }

    public void setBiasT(boolean enabled)
    {
        setGPIOOutput((byte)0x01);
        setGPIOBit((byte)0x01, enabled);
        mBiasTEnabled = enabled;
    }

    public boolean isBiasT()
    {
        return mBiasTEnabled;
    }

    private void setIFFrequency(int frequency) throws LibUsbException
    {
        long ifFrequency = ((long) TWO_TO_22_POWER * (long) frequency) / (long) mOscillatorFrequency * -1;
        writeDemodRegister(Page.ONE, (short) 0x19, (short) (Long.rotateRight(ifFrequency, 16) & 0x3F), 1);
        writeDemodRegister(Page.ONE, (short) 0x1A, (short) (Long.rotateRight(ifFrequency, 8) & 0xFF), 1);
        writeDemodRegister(Page.ONE, (short) 0x1B, (short) (ifFrequency & 0xFF), 1);
    }

    public String getUniqueID()
    {
        if(mDescriptor != null)
        {
            if(mDescriptor.hasSerial())
            {
                if(hasEmbeddedTuner())
                {
                    return TunerClass.RTL2832 + "/" + getTunerType().getLabel() + " " + mDescriptor.getSerial();
                }
                else
                {
                    return TunerClass.RTL2832 + " " + mDescriptor.getSerial();
                }
            }
            else
            {
                return "RTL-2832 USB Bus:" + mBus + " Port:" + mPortAddress;
            }
        }
        else
        {
            int serial = (0xFF & getDeviceDescriptor().iSerialNumber());
            return TunerClass.RTL2832 + " " + serial;
        }
    }

    public void setSampleRateFilters(int sampleRate) throws SourceException
    {
        getEmbeddedTuner().setSampleRateFilters(sampleRate);
    }

    public TunerType getTunerType()
    {
        if(getEmbeddedTuner() != null)
        {
            return getEmbeddedTuner().getTunerType();
        }

        return TunerType.UNKNOWN;
    }

    private TunerType identifyTunerType() throws SourceException
    {
        TunerType tunerClass = TunerType.UNKNOWN;

        try
        {
            enableI2CRepeater(true);
            boolean controlI2CRepeater = false;

            if(isTuner(TunerTypeCheck.E4K, controlI2CRepeater))
            {
                tunerClass = TunerType.ELONICS_E4000;
            }
            else if(isTuner(TunerTypeCheck.R820T, controlI2CRepeater))
            {
                tunerClass = TunerType.RAFAELMICRO_R820T;
            }
            else if(isTuner(TunerTypeCheck.FC0013, controlI2CRepeater))
            {
                tunerClass = TunerType.FITIPOWER_FC0013;
            }
            else if(isTuner(TunerTypeCheck.R828D, controlI2CRepeater))
            {
                tunerClass = TunerType.RAFAELMICRO_R828D;
            }
            else if(isTuner(TunerTypeCheck.FC2580, controlI2CRepeater))
            {
                tunerClass = TunerType.FCI_FC2580;
            }
            else if(isTuner(TunerTypeCheck.FC0012, controlI2CRepeater))
            {
                tunerClass = TunerType.FITIPOWER_FC0012;
            }

            enableI2CRepeater(false);
        }
        catch(Exception e)
        {
            mLog.error("error while determining tuner type", e);
        }

        return tunerClass;
    }

    @Override
    protected void prepareStreaming()
    {
        resetUSBBuffer();
    }

    public void resetUSBBuffer() throws LibUsbException
    {
        writeRegister(Block.USB, Address.USB_EPA_CTL, 0x1002, 2);
        writeRegister(Block.USB, Address.USB_EPA_CTL, 0x0000, 2);
    }

    public void initBaseband() throws LibUsbException
    {
        writeRegister(Block.USB, Address.USB_SYSCTL, 0x09, 1);
        writeRegister(Block.USB, Address.USB_EPA_MAXPKT, 0x0002, 2);
        writeRegister(Block.USB, Address.USB_EPA_CTL, 0x1002, 2);

        writeRegister(Block.SYS, Address.DEMOD_CTL_1, 0x22, 1);
        writeRegister(Block.SYS, Address.DEMOD_CTL, 0xE8, 1);

        writeDemodRegister(Page.ONE, (short) 0x01, 0x14, 1);
        writeDemodRegister(Page.ONE, (short) 0x01, 0x10, 1);

        writeDemodRegister(Page.ONE, (short) 0x15, 0x00, 1);
        writeDemodRegister(Page.ONE, (short) 0x16, 0x0000, 2);

        writeDemodRegister(Page.ONE, (short) 0x16, 0x00, 1);
        writeDemodRegister(Page.ONE, (short) 0x17, 0x00, 1);
        writeDemodRegister(Page.ONE, (short) 0x18, 0x00, 1);
        writeDemodRegister(Page.ONE, (short) 0x19, 0x00, 1);
        writeDemodRegister(Page.ONE, (short) 0x1A, 0x00, 1);
        writeDemodRegister(Page.ONE, (short) 0x1B, 0x00, 1);

        for(int x = 0; x < FIR_FILTER_COEFFICIENTS.length; x++)
        {
            writeDemodRegister(Page.ONE, (short) (0x1C + x), FIR_FILTER_COEFFICIENTS[x], 1);
        }

        writeDemodRegister(Page.ZERO, (short) 0x19, 0x05, 1);
        writeDemodRegister(Page.ONE, (short) 0x93, 0xF0, 1);
        writeDemodRegister(Page.ONE, (short) 0x94, 0x0F, 1);
        writeDemodRegister(Page.ONE, (short) 0x11, 0x00, 1);
        writeDemodRegister(Page.ONE, (short) 0x04, 0x00, 1);
        writeDemodRegister(Page.ZERO, (short) 0x61, 0x60, 1);
        writeDemodRegister(Page.ZERO, (short) 0x06, 0x80, 1);
        writeDemodRegister(Page.ONE, (short) 0xB1, 0x1B, 1);
        writeDemodRegister(Page.ZERO, (short) 0x0D, 0x83, 1);
    }

    private void deinitBaseband() throws IllegalArgumentException, UsbDisconnectedException
    {
        writeRegister(Block.SYS, Address.DEMOD_CTL, 0x20, 1);
    }

    private void setGPIOBit(byte bitMask, boolean enabled) throws LibUsbException
    {
        int value = readRegister(Block.SYS, Address.GPO, 1);
        if(enabled)
        {
            value |= bitMask;
        }
        else
        {
            value &= ~bitMask;
        }
        writeRegister(Block.SYS, Address.GPO, value, 1);
    }

    private void setGPIOOutput(byte bitMask) throws LibUsbException
    {
        int value = readRegister(Block.SYS, Address.GPD, 1);
        writeRegister(Block.SYS, Address.GPO, value & ~bitMask, 1);
        value = readRegister(Block.SYS, Address.GPOE, 1);
        writeRegister(Block.SYS, Address.GPOE, value | bitMask, 1);
    }

    public void enableI2CRepeater(boolean enabled) throws LibUsbException
    {
        short address = 1;
        int value = (enabled ? 0x18 : 0x10);
        boolean isEnabledAlready = isI2CRepeaterEnabled();
        if (isEnabledAlready != enabled) {
            writeDemodRegister(Page.ONE, address, value, 1);
        }
    }

    public boolean isI2CRepeaterEnabled()
    {
        return readDemodRegister(Page.ONE, (short) 0x1, 1) == 0x18;
    }

    public int readI2CRegister(byte i2CAddress, byte i2CRegister, boolean controlI2CRepeater) throws LibUsbException
    {
        short address = (short) (i2CAddress & 0xFF);
        ByteBuffer buffer = ByteBuffer.allocateDirect(1);
        buffer.put(i2CRegister);
        buffer.rewind();
        ByteBuffer data = ByteBuffer.allocateDirect(1);

        if(controlI2CRepeater)
        {
            enableI2CRepeater(true);
            write(address, Block.I2C, buffer);
            read(address, Block.I2C, data);
            enableI2CRepeater(false);
        }
        else
        {
            write(address, Block.I2C, buffer);
            read(address, Block.I2C, data);
        }

        return (data.get() & 0xFF);
    }

    public void writeI2CRegister(byte i2CAddress, byte i2CRegister, byte value, boolean controlI2CRepeater)
            throws LibUsbException
    {
        short address = (short) (i2CAddress & 0xFF);
        ByteBuffer buffer = ByteBuffer.allocateDirect(2);
        buffer.put(i2CRegister);
        buffer.put(value);
        buffer.rewind();

        if(controlI2CRepeater)
        {
            enableI2CRepeater(true);
            write(address, Block.I2C, buffer);
            enableI2CRepeater(false);
        }
        else
        {
            write(address, Block.I2C, buffer);
        }
    }

    private void writeDemodRegister(Page page, short address, int value, int length) throws LibUsbException
    {
        ByteBuffer buffer = ByteBuffer.allocateDirect(length);
        buffer.order(ByteOrder.BIG_ENDIAN);

        if(length == 1)
        {
            buffer.put((byte) (value & 0xFF));
        }
        else if(length == 2)
        {
            buffer.putShort((short) (value & 0xFFFF));
        }
        else
        {
            throw new IllegalArgumentException("Cannot write value greater than 16 bits to the register");
        }

        short index = (short) (0x10 | page.getPage());
        short newAddress = (short) (address << 8 | 0x20);
        write(newAddress, index, buffer);
        readDemodRegister(Page.TEN, (short) 1, length);
    }

    private int readDemodRegister(Page page, short address, int length) throws LibUsbException
    {
        short index = page.getPage();
        short newAddress = (short) ((address << 8) | 0x20);
        ByteBuffer buffer = ByteBuffer.allocateDirect(length);
        read(newAddress, index, buffer);
        buffer.order(ByteOrder.LITTLE_ENDIAN);

        if(length == 2)
        {
            return (buffer.getShort() & 0xFFFF);
        }
        else
        {
            return (buffer.get() & 0xFF);
        }
    }

    private void writeRegister(Block block, Address address, int value, int length) throws LibUsbException
    {
        ByteBuffer buffer = ByteBuffer.allocateDirect(length);
        buffer.order(ByteOrder.BIG_ENDIAN);

        if(length == 1)
        {
            buffer.put((byte) (value & 0xFF));
        }
        else if(length == 2)
        {
            buffer.putShort((short) value);
        }
        else
        {
            throw new IllegalArgumentException("Cannot write value greater than 16 bits to the register");
        }

        buffer.rewind();
        write(address.getAddress(), block, buffer);
    }

    private int readRegister(Block block, Address address, int length) throws LibUsbException
    {
        ByteBuffer buffer = ByteBuffer.allocateDirect(2);
        read(address.getAddress(), block, buffer);
        buffer.order(ByteOrder.LITTLE_ENDIAN);

        if(length == 2)
        {
            return (buffer.getShort() & 0xFFFF);
        }
        else
        {
            return (buffer.get() & 0xFF);
        }
    }

    private void write(short address, Block block, ByteBuffer buffer) throws LibUsbException
    {
        write(address, block.getWriteIndex(), buffer);
    }

    private void write(short value, short index, ByteBuffer buffer) throws LibUsbException
    {
        if(hasDeviceHandle())
        {
            int transferred = LibUsb.controlTransfer(getDeviceHandle(), CONTROL_ENDPOINT_OUT, REQUEST_ZERO, value,
                    index, buffer, TIMEOUT_US);

            if(transferred < 0)
            {
                throw new LibUsbException("error writing byte buffer", transferred);
            }
            else if(transferred != buffer.capacity())
            {
                throw new LibUsbException("transferred bytes do not match expected capacity", transferred);
            }
        }
        else
        {
            throw new LibUsbException("device handle is null", LibUsb.ERROR_NO_DEVICE);
        }
    }

    private void read(short address, short index, ByteBuffer buffer) throws LibUsbException
    {
        if(isRunning())
        {
            int transferred = LibUsb.controlTransfer(getDeviceHandle(), CONTROL_ENDPOINT_IN, REQUEST_ZERO, address,
                    index, buffer, TIMEOUT_US);

            if(transferred < 0)
            {
                throw new LibUsbException("read error", transferred);
            }
            else if(transferred != buffer.capacity())
            {
                throw new LibUsbException("transferred bytes do not match expected capacity", transferred);
            }
        }
    }

    private void read(short address, Block block, ByteBuffer buffer) throws LibUsbException
    {
        read(address, block.getReadIndex(), buffer);
    }

    private boolean isTuner(TunerTypeCheck type, boolean controlI2CRepeater)
    {
        try
        {
            if(type == TunerTypeCheck.FC0012 || type == TunerTypeCheck.FC2580)
            {
                setGPIOOutput((byte) 0x20);
                setGPIOBit((byte) 0x20, true);
                setGPIOBit((byte) 0x20, false);
            }

            int value = readI2CRegister(type.getI2CAddress(), type.getCheckAddress(), controlI2CRepeater);

            if(type == TunerTypeCheck.FC2580)
            {
                return ((value & 0x7F) == (type.getCheckValue() & 0xFF));
            }
            else
            {
                return (value == (type.getCheckValue() & 0xFF));
            }
        }
        catch(LibUsbException e)
        {
            //Do nothing
        }

        return false;
    }

    public double getCurrentSampleRate()
    {
        return mSampleRate.getRate();
    }

    public int getSampleRateFromTuner() throws SourceException
    {
        int returnRate = DEFAULT_SAMPLE_RATE.getRate();
        getLock().lock();

        try
        {
            int high = readDemodRegister(Page.ONE, (short) 0x9F, 2);
            int low = readDemodRegister(Page.ONE, (short) 0xA1, 2);
            int ratio = Integer.rotateLeft(high, 16) | low;
            int rate = (mOscillatorFrequency * TWO_TO_22_POWER / ratio);
            SampleRate sampleRate = SampleRate.getClosest(rate);

            if(sampleRate.getRate() != rate)
            {
                setSampleRate(sampleRate);
                returnRate = sampleRate.getRate();
            }
        }
        catch(Exception e)
        {
            throw new SourceException("RTL2832 Tuner Controller - cannot get current sample rate", e);
        }
        finally
        {
            getLock().unlock();
        }

        return returnRate;
    }

    public void setSampleRate(SampleRate sampleRate) throws SourceException
    {
        getLock().lock();

        try
        {
            writeDemodRegister(Page.ONE, (short) 0x9F, sampleRate.getRatioHighBits(), 2);
            writeDemodRegister(Page.ONE, (short) 0xA1, 0, 2);
            setSampleRateFrequencyCorrection(0);

            writeDemodRegister(Page.ONE, (short) 0x01, 0x14, 1);
            writeDemodRegister(Page.ONE, (short) 0x01, 0x10, 1);

            setSampleRateFilters(sampleRate.getRate());
            mSampleRate = sampleRate;
            mFrequencyController.setSampleRate(sampleRate.getRate());
            getNativeBufferFactory().setSamplesPerMillisecond(sampleRate.getRate() / 1000.0f);
        }
        finally
        {
            getLock().unlock();
        }
    }

    public void setSampleRateFrequencyCorrection(int ppm) throws SourceException
    {
        int offset = -ppm * TWO_TO_22_POWER / 1_000_000;
        writeDemodRegister(Page.ONE, (short) 0x3F, (offset & 0xFF), 1);
        writeDemodRegister(Page.ONE, (short) 0x3E, (Integer.rotateRight(offset, 8) & 0xFF), 1);
        try
        {
            mFrequencyController.setFrequency(mFrequencyController.getFrequency());
        }
        catch(Exception e)
        {
            throw new SourceException("Couldn't set sample rate frequency correction", e);
        }
    }

    public byte[] readEEPROM(short offset, int length) throws IllegalArgumentException
    {
        if(offset + length > 256)
        {
            throw new IllegalArgumentException("cannot read more than 256 bytes from EEPROM");
        }

        byte[] data = new byte[length];
        ByteBuffer buffer = ByteBuffer.allocateDirect(1);

        try
        {
            writeRegister(Block.I2C, Address.EEPROM, (byte) offset, 1);
        }
        catch(LibUsbException e)
        {
            mLog.error("usb error while attempting to set read address to EEPROM", e);
        }

        for(int x = 0; x < length; x++)
        {
            try
            {
                read(EEPROM_ADDRESS, Block.I2C, buffer);
                data[x] = buffer.get();
                buffer.rewind();
            }
            catch(Exception e)
            {
                x = length;
            }
        }

        return data;
    }

    public enum Address
    {
        USB_SYSCTL(0x2000), USB_CTRL(0x2010), USB_STAT(0x2014),
        USB_EPA_CFG(0x2144), USB_EPA_CTL(0x2148), USB_EPA_MAXPKT(0x2158),
        USB_EPA_MAXPKT_2(0x215A), USB_EPA_FIFO_CFG(0x2160), DEMOD_CTL(0x3000),
        GPO(0x3001), GPI(0x3002), GPOE(0x3003), GPD(0x3004),
        SYSINTE(0x3005), SYSINTS(0x3006), GP_CFG0(0x3007), GP_CFG1(0x3008),
        SYSINTE_1(0x3009), SYSINTS_1(0x300A), DEMOD_CTL_1(0x300B),
        IR_SUSPEND(0x300C), EEPROM(0xA0);

        private int mAddress;
        Address(int address) { mAddress = address; }
        public short getAddress() { return (short) mAddress; }
    }

    public enum Page
    {
        ZERO(0x0), ONE(0x1), TEN(0xA);
        private int mPage;
        Page(int page) { mPage = page; }
        public byte getPage() { return (byte) (mPage & 0xFF); }
    }

    public enum SampleMode
    {
        QUADRATURE, DIRECT;
    }

    public enum Block
    {
        DEMOD(0), USB(1), SYS(2), TUN(3), ROM(4), IR(5), I2C(6);
        private int mValue;
        Block(int value) { mValue = value; }
        public short getReadIndex() { return (short) Integer.rotateLeft(mValue, 8); }
        public short getWriteIndex() { return (short) (getReadIndex() | 0x10); }
    }

    public enum SampleRate
    {
        RATE_0_230MHZ(0x1F40, 230400, "0.230 MHz"),
        RATE_0_240MHZ(0x1E00, 240000, "0.240 MHz"),
        RATE_0_256MHZ(0x1C20, 256000, "0.256 MHz"),
        RATE_0_288MHZ(0x1900, 288000, "0.288 MHz"),
        RATE_0_300MHZ(0x1800, 300000, "0.300 MHz"),
        RATE_0_960MHZ(0x0780, 960000, "0.960 MHz"),
        RATE_1_024MHZ(0x0708, 1024000, "1.024 MHz"),
        RATE_1_200MHZ(0x0600, 1200000, "1.200 MHz"),
        RATE_1_440MHZ(0x0500, 1440000, "1.440 MHz"),
        RATE_1_600MHZ(0x0480, 1600000, "1.600 MHz"),
        RATE_1_800MHZ(0x0400, 1800000, "1.800 MHz"),
        RATE_1_920MHZ(0x03C0, 1920000, "1.920 MHz"),
        RATE_2_048MHZ(0x0384, 2048000, "2.048 MHz"),
        RATE_2_304MHZ(0x0320, 2304000, "2.304 MHz"),
        RATE_2_400MHZ(0x0300, 2400000, "2.400 MHz"),
        RATE_2_560MHZ(0x02D0, 2560000, "2.560 MHz"),
        RATE_2_880MHZ(0x0280, 2880000, "2.880 MHz");

        private int mRatioHigh;
        private int mRate;
        private String mLabel;

        SampleRate(int ratioHigh, int rate, String label)
        {
            mRatioHigh = ratioHigh;
            mRate = rate;
            mLabel = label;
        }

        public int getRatioHighBits() { return mRatioHigh; }
        public int getRate() { return mRate; }
        public String getLabel() { return mLabel; }
        public String toString() { return mLabel; }

        public static SampleRate getClosest(int sampleRate)
        {
            for(SampleRate rate : values())
            {
                if(rate.getRate() >= sampleRate)
                {
                    return rate;
                }
            }
            return DEFAULT_SAMPLE_RATE;
        }
    }

    public enum TunerTypeCheck
    {
        E4K(0xC8, 0x02, 0x40),
        FC0012(0xC6, 0x00, 0xA1),
        FC0013(0xC6, 0x00, 0xA3),
        FC2580(0xAC, 0x01, 0x56),
        R820T(0x34, 0x00, 0x69),
        R828D(0x74, 0x00, 0x69);

        private int mI2CAddress, mCheckAddress, mCheckValue;
        TunerTypeCheck(int i2c, int address, int value) { mI2CAddress = i2c; mCheckAddress = address; mCheckValue = value; }
        public byte getI2CAddress() { return (byte) mI2CAddress; }
        public byte getCheckAddress() { return (byte) mCheckAddress; }
        public byte getCheckValue() { return (byte) mCheckValue; }
    }

    public class Descriptor
    {
        private byte[] mData;
        private ArrayList<String> mLabels = new ArrayList<String>();

        public Descriptor(byte[] data)
        {
            mData = (data != null) ? data : new byte[256];
            getLabels();
        }

        public boolean isRtlSdrV4()
        {
            String vendor = getVendorLabel();
            String product = getProductLabel();
            return vendor != null && vendor.contains("RTLSDRBlog") && product != null && product.contains("Blog V4") && !product.contains("V4L");
        }

        /**
         * Robust detection for V4L: checks if the product label contains "V4L" (case-insensitive)
         */
        public boolean isRtlSdrV4L()
        {
            String product = getProductLabel();
            return product != null && product.toUpperCase().contains("V4L");
        }

        public boolean isValid()
        {
            return mData[0] != (byte)0x0 && mData[1] != (byte)0x0;
        }

        public String getVendorID()
        {
            int id = Integer.rotateLeft((0xFF & mData[3]), 8) | (0xFF & mData[2]);
            return String.format("%04X", id);
        }

        public String getVendorLabel()
        {
            return mLabels.isEmpty() ? "" : mLabels.get(0);
        }

        public String getProductID()
        {
            int id = Integer.rotateLeft((0xFF & mData[5]), 8) | (0xFF & mData[4]);
            return String.format("%04X", id);
        }

        public String getProductLabel()
        {
            return mLabels.size() < 2 ? "" : mLabels.get(1);
        }

        public boolean hasSerial()
        {
            return mData[6] == (byte) 0xA5;
        }

        public String getSerial()
        {
            return mLabels.size() < 3 ? "" : mLabels.get(2);
        }

        public boolean remoteWakeupEnabled()
        {
            byte mask = (byte) 0x01;
            return (mData[7] & mask) == mask;
        }

        public boolean irEnabled()
        {
            byte mask = (byte) 0x02;
            return (mData[7] & mask) == mask;
        }

        private void getLabels()
        {
            mLabels.clear();
            int start = 0x09;
            while(start < 256)
            {
                start = getLabel(start);
            }
        }

        private int getLabel(int start)
        {
            if(start > 254 || mData[start + 1] != (byte) 0x03)
            {
                return 256;
            }
            int length = 0xFF & mData[start];
            if(start + length > 255)
            {
                return 256;
            }
            byte[] data = Arrays.copyOfRange(mData, start + 2, start + length);
            String label = new String(data, Charset.forName("UTF-16LE"));
            mLabels.add(label);
            return start + length;
        }
    }

    public class ControllerAdapter
    {
        private RTL2832TunerController mController;

        public ControllerAdapter(RTL2832TunerController controller)
        {
            mController = controller;
        }

        public void setGpioPin(int pin, boolean enabled) throws LibUsbException
        {
            byte mask = (byte) (1 << pin);
            mController.setGPIOOutput(mask);
            mController.setGPIOBit(mask, enabled);
        }

        public boolean isRunning()
        {
            return mController.isRunning();
        }

        public boolean isV4Dongle()
        {
            return mController.getDescriptor() != null && mController.getDescriptor().isRtlSdrV4();
        }

        public boolean isV4LDongle()
        {
            return mController.getDescriptor() != null && mController.getDescriptor().isRtlSdrV4L();
        }

        public DeviceHandle getDeviceHandle()
        {
            return mController.getDeviceHandle();
        }

        public ReentrantLock getLock()
        {
            return mController.getLock();
        }

        public void enableI2CRepeater()
        {
            mController.enableI2CRepeater(true);
        }

        public void disableI2CRepeater()
        {
            mController.enableI2CRepeater(false);
        }

        public boolean isI2CRepeaterEnabled()
        {
            return mController.isI2CRepeaterEnabled();
        }

        public void writeDemodRegister(Page page, short address, int value, int length) throws LibUsbException
        {
            mController.writeDemodRegister(page, address, value, length);
        }

        public void writeI2CRegister(byte i2CAddress, byte i2CRegister, byte value, boolean controlI2CRepeater)
                throws LibUsbException
        {
            mController.writeI2CRegister(i2CAddress, i2CRegister, value, controlI2CRepeater);
        }

        public int readI2CRegister(byte i2CAddress, byte i2CRegister, boolean controlI2CRepeater) throws LibUsbException
        {
            return mController.readI2CRegister(i2CAddress, i2CRegister, controlI2CRepeater);
        }

        public void read(short address, Block block, ByteBuffer buffer) throws LibUsbException
        {
            mController.read(address, block, buffer);
        }

        public void setIFFrequency(int frequency)
        {
            mController.setIFFrequency(frequency);
        }
    }
}

package frc.robot.subsystems.common;

import java.io.IOException;

import edu.wpi.first.wpilibj.I2C;

public class EndianIO {

	public enum Endianness {
		LITTLE_ENDIAN, BIG_ENDIAN
	}

	/**
	 * Read an unsigned byte from the I2C device
	 */
	public static int readReg(I2C device, int reg, boolean verbose) throws IOException {
		int result = 0;
		boolean readSuccessful = false;
		byte[] localBuffer = new byte[1];

		readSuccessful = device.read(reg, 1, localBuffer);
		if (readSuccessful) {
			result = localBuffer[0];
		}

		if (verbose) {
			System.out.println("I2C: Device " + device.toString() + " (0x" + Integer.toHexString(i2caddr)
					+ ") returned " + result + " (0x" + Integer.toHexString(result) + ") from reg " + reg + " (0x"
					+ Integer.toHexString(reg) + ")");
		}

		return result; // & 0xFF;
	}

	/**
	 * Read a signed byte from the I2C device
	 */
	public static int readS8(I2C device, int reg, boolean verbose) throws IOException {
		int result = 0;
		boolean readSuccessful = false;
		byte[] localBuffer = new byte[1];

		readSuccessful = device.read(reg, 1, localBuffer); // & 0x7F;
		if (readSuccessful) {
			result = localBuffer[0];
		}

		if (result > 127)
			result -= 256;
		if (verbose) {
			System.out.println("I2C: Device " + device.toString() + " returned " + result + " from reg " + reg);
		}
		return result; // & 0xFF;
	}

	public static int readReg16(I2C device, int register, boolean verbose) throws IOException {
		return EndianIO.readU16(device, register, Endianness.LITTLE_ENDIAN, verbose);
	}

	public static int readU16BE(I2C device, int register, boolean verbose) throws IOException {
		return EndianIO.readU16(device, register, Endianness.BIG_ENDIAN, verbose);
	}

	public static int readU16(I2C device, int register, Endianness endianness, boolean verbose) throws IOException {
		int hi = EndianIO.readReg(device, register, verbose);
		int lo = EndianIO.readReg(device, register + 1, verbose);
		return ((endianness == Endianness.BIG_ENDIAN) ? (hi << 8) + lo : (lo << 8) + hi); // & 0xFFFF;
	}

	public static int readS16(I2C device, int register, Endianness endianness, boolean verbose) throws IOException {
		int hi = 0, lo = 0;
		if (endianness == Endianness.BIG_ENDIAN) {
			hi = EndianIO.readReg(device, register, verbose);
			lo = EndianIO.readReg(device, register + 1, verbose);
		} else {
			lo = EndianIO.readReg(device, register, verbose);
			hi = EndianIO.readS8(device, register + 1, verbose);
		}
		return ((hi << 8) + lo); // & 0xFFFF;
	}

	public static int readS16LE(I2C device, int register, boolean verbose) throws IOException {
		return EndianIO.readS16(device, register, Endianness.LITTLE_ENDIAN, verbose);
	}

	public static int readS16BE(I2C device, int register, boolean verbose) throws IOException {
		return EndianIO.readS16(device, register, Endianness.BIG_ENDIAN, verbose);
	}

	public static void writeReg(I2C device, int register, byte i) {
		device.write(register, i);

	}

	public static void writeReg16Bit(I2C device, int register, int value) {
		device.write(register, (value >> 8) & 0xFF);
		device.write(register + 1, (value & 0xFF));
	}

	public static void writeReg32Bit(I2C device, int register, int value) {
		device.write(register, (value >> 24) & 0xFF);
		device.write(register + 1, (value >> 16) & 0xFF);
		device.write(register + 2, (value >> 8) & 0xFF);
		device.write(register + 3, (value & 0xFF));
	}

}
package frc.robot.subsystems;

import java.io.IOException;
import java.util.Arrays;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import frc.robot.subsystems.common.EndianIO;

/**
 * Adapted from from
 * https://github.com/adafruit/Adafruit_CircuitPython_vl53l1x/blob/master/adafruit_vl53l1x.py
 * Driver for the vl53l1x https://www.adafruit.com/product/3317
 */
public class VL53L1X {

	// The Arduino two-wire interface uses a 7-bit number for the address,
	// and sets the last bit correctly based on reads and writes
	private final static int AddressDefault = 0b0101001;

	// value used in measurement timing budget calculations
	// assumes PresetMode is LOWPOWER_AUTONOMOUS
	//
	// vhv = LOWPOWER_AUTO_VHV_LOOP_DURATION_US + LOWPOWERAUTO_VHV_LOOP_BOUND
	// (tuning parm default) * LOWPOWER_AUTO_VHV_LOOP_DURATION_US
	// = 245 + 3 * 245 = 980
	// TimingGuard = LOWPOWER_AUTO_OVERHEAD_BEFORE_A_RANGING +
	// LOWPOWER_AUTO_OVERHEAD_BETWEEN_A_B_RANGING + vhv
	// = 1448 + 2100 + 980 = 4528
	private final static int TimingGuard = 4528;

	// value in DSS_CONFIG__TARGET_TOTAL_RATE_MCPS register, used in DSS
	// calculations
	private final static int TargetRate = 0x0A00;

	// for storing values read from RESULT__RANGE_STATUS (0x0089)
	// through RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0_LOW
	// (0x0099)
	class ResultBuffer {
		byte range_status;
		// byte report_status: not used
		byte stream_count;
		int dss_actual_effective_spads_sd0;
		// uint16_t peak_signal_count_rate_mcps_sd0: not used
		int ambient_count_rate_mcps_sd0;
		// uint16_t sigma_sd0: not used
		// uint16_t phase_sd0: not used
		int final_crosstalk_corrected_range_mm_sd0;
		int peak_signal_count_rate_crosstalk_corrected_mcps_sd0;
	};

	// making this static would save RAM for multiple instances as long as there
	// aren't multiple sensors being read at the same time (e.g. on separate
	// I2C buses)
	ResultBuffer results;

	byte address;

	int io_timeout;
	boolean did_timeout;
	long timeout_start_ms;

	int fast_osc_frequency;
	int osc_calibrate_val;

	boolean calibrated;
	byte saved_vhv_init;
	byte saved_vhv_timeout;

	DistanceMode distance_mode;

	public final static int V53L1X_I2CADDR = 0x29;

	public final static int SOFT_RESET = 0x0000;
	public final static int I2C_SLAVE__DEVICE_ADDRESS = 0x0001;
	public final static int ANA_CONFIG__VHV_REF_SEL_VDDPIX = 0x0002;
	public final static int ANA_CONFIG__VHV_REF_SEL_VQUENCH = 0x0003;
	public final static int ANA_CONFIG__REG_AVDD1V2_SEL = 0x0004;
	public final static int ANA_CONFIG__FAST_OSC__TRIM = 0x0005;
	public final static int OSC_MEASURED__FAST_OSC__FREQUENCY = 0x0006;
	public final static int OSC_MEASURED__FAST_OSC__FREQUENCY_HI = 0x0006;
	public final static int OSC_MEASURED__FAST_OSC__FREQUENCY_LO = 0x0007;
	public final static int VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND = 0x0008;
	public final static int VHV_CONFIG__COUNT_THRESH = 0x0009;
	public final static int VHV_CONFIG__OFFSET = 0x000A;
	public final static int VHV_CONFIG__INIT = 0x000B;
	public final static int GLOBAL_CONFIG__SPAD_ENABLES_REF_0 = 0x000D;
	public final static int GLOBAL_CONFIG__SPAD_ENABLES_REF_1 = 0x000E;
	public final static int GLOBAL_CONFIG__SPAD_ENABLES_REF_2 = 0x000F;
	public final static int GLOBAL_CONFIG__SPAD_ENABLES_REF_3 = 0x0010;
	public final static int GLOBAL_CONFIG__SPAD_ENABLES_REF_4 = 0x0011;
	public final static int GLOBAL_CONFIG__SPAD_ENABLES_REF_5 = 0x0012;
	public final static int GLOBAL_CONFIG__REF_EN_START_SELECT = 0x0013;
	public final static int REF_SPAD_MAN__NUM_REQUESTED_REF_SPADS = 0x0014;
	public final static int REF_SPAD_MAN__REF_LOCATION = 0x0015;
	public final static int ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS = 0x0016;
	public final static int ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS_HI = 0x0016;
	public final static int ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS_LO = 0x0017;
	public final static int ALGO__CROSSTALK_COMPENSATION_X_PLANE_GRADIENT_KCPS = 0x0018;
	public final static int ALGO__CROSSTALK_COMPENSATION_X_PLANE_GRADIENT_KCPS_HI = 0x0018;
	public final static int ALGO__CROSSTALK_COMPENSATION_X_PLANE_GRADIENT_KCPS_LO = 0x0019;
	public final static int ALGO__CROSSTALK_COMPENSATION_Y_PLANE_GRADIENT_KCPS = 0x001A;
	public final static int ALGO__CROSSTALK_COMPENSATION_Y_PLANE_GRADIENT_KCPS_HI = 0x001A;
	public final static int ALGO__CROSSTALK_COMPENSATION_Y_PLANE_GRADIENT_KCPS_LO = 0x001B;
	public final static int REF_SPAD_CHAR__TOTAL_RATE_TARGET_MCPS = 0x001C;
	public final static int REF_SPAD_CHAR__TOTAL_RATE_TARGET_MCPS_HI = 0x001C;
	public final static int REF_SPAD_CHAR__TOTAL_RATE_TARGET_MCPS_LO = 0x001D;
	public final static int ALGO__PART_TO_PART_RANGE_OFFSET_MM = 0x001E;
	public final static int ALGO__PART_TO_PART_RANGE_OFFSET_MM_HI = 0x001E;
	public final static int ALGO__PART_TO_PART_RANGE_OFFSET_MM_LO = 0x001F;
	public final static int MM_CONFIG__INNER_OFFSET_MM = 0x0020;
	public final static int MM_CONFIG__INNER_OFFSET_MM_HI = 0x0020;
	public final static int MM_CONFIG__INNER_OFFSET_MM_LO = 0x0021;
	public final static int MM_CONFIG__OUTER_OFFSET_MM = 0x0022;
	public final static int MM_CONFIG__OUTER_OFFSET_MM_HI = 0x0022;
	public final static int MM_CONFIG__OUTER_OFFSET_MM_LO = 0x0023;
	public final static int DSS_CONFIG__TARGET_TOTAL_RATE_MCPS = 0x0024;
	public final static int DSS_CONFIG__TARGET_TOTAL_RATE_MCPS_HI = 0x0024;
	public final static int DSS_CONFIG__TARGET_TOTAL_RATE_MCPS_LO = 0x0025;
	public final static int DEBUG__CTRL = 0x0026;
	public final static int TEST_MODE__CTRL = 0x0027;
	public final static int CLK_GATING__CTRL = 0x0028;
	public final static int NVM_BIST__CTRL = 0x0029;
	public final static int NVM_BIST__NUM_NVM_WORDS = 0x002A;
	public final static int NVM_BIST__START_ADDRESS = 0x002B;
	public final static int HOST_IF__STATUS = 0x002C;
	public final static int PAD_I2C_HV__CONFIG = 0x002D;
	public final static int PAD_I2C_HV__EXTSUP_CONFIG = 0x002E;
	public final static int GPIO_HV_PAD__CTRL = 0x002F;
	public final static int GPIO_HV_MUX__CTRL = 0x0030;
	public final static int GPIO__TIO_HV_STATUS = 0x0031;
	public final static int GPIO__FIO_HV_STATUS = 0x0032;
	public final static int ANA_CONFIG__SPAD_SEL_PSWIDTH = 0x0033;
	public final static int ANA_CONFIG__VCSEL_PULSE_WIDTH_OFFSET = 0x0034;
	public final static int ANA_CONFIG__FAST_OSC__CONFIG_CTRL = 0x0035;
	public final static int SIGMA_ESTIMATOR__EFFECTIVE_PULSE_WIDTH_NS = 0x0036;
	public final static int SIGMA_ESTIMATOR__EFFECTIVE_AMBIENT_WIDTH_NS = 0x0037;
	public final static int SIGMA_ESTIMATOR__SIGMA_REF_MM = 0x0038;
	public final static int ALGO__CROSSTALK_COMPENSATION_VALID_HEIGHT_MM = 0x0039;
	public final static int SPARE_HOST_CONFIG__STATIC_CONFIG_SPARE_0 = 0x003A;
	public final static int SPARE_HOST_CONFIG__STATIC_CONFIG_SPARE_1 = 0x003B;
	public final static int ALGO__RANGE_IGNORE_THRESHOLD_MCPS = 0x003C;
	public final static int ALGO__RANGE_IGNORE_THRESHOLD_MCPS_HI = 0x003C;
	public final static int ALGO__RANGE_IGNORE_THRESHOLD_MCPS_LO = 0x003D;
	public final static int ALGO__RANGE_IGNORE_VALID_HEIGHT_MM = 0x003E;
	public final static int ALGO__RANGE_MIN_CLIP = 0x003F;
	public final static int ALGO__CONSISTENCY_CHECK__TOLERANCE = 0x0040;
	public final static int SPARE_HOST_CONFIG__STATIC_CONFIG_SPARE_2 = 0x0041;
	public final static int SD_CONFIG__RESET_STAGES_MSB = 0x0042;
	public final static int SD_CONFIG__RESET_STAGES_LSB = 0x0043;
	public final static int GPH_CONFIG__STREAM_COUNT_UPDATE_VALUE = 0x0044;
	public final static int GLOBAL_CONFIG__STREAM_DIVIDER = 0x0045;
	public final static int SYSTEM__INTERRUPT_CONFIG_GPIO = 0x0046;
	public final static int CAL_CONFIG__VCSEL_START = 0x0047;
	public final static int CAL_CONFIG__REPEAT_RATE = 0x0048;
	public final static int CAL_CONFIG__REPEAT_RATE_HI = 0x0048;
	public final static int CAL_CONFIG__REPEAT_RATE_LO = 0x0049;
	public final static int GLOBAL_CONFIG__VCSEL_WIDTH = 0x004A;
	public final static int PHASECAL_CONFIG__TIMEOUT_MACROP = 0x004B;
	public final static int PHASECAL_CONFIG__TARGET = 0x004C;
	public final static int PHASECAL_CONFIG__OVERRIDE = 0x004D;
	public final static int DSS_CONFIG__ROI_MODE_CONTROL = 0x004F;
	public final static int SYSTEM__THRESH_RATE_HIGH = 0x0050;
	public final static int SYSTEM__THRESH_RATE_HIGH_HI = 0x0050;
	public final static int SYSTEM__THRESH_RATE_HIGH_LO = 0x0051;
	public final static int SYSTEM__THRESH_RATE_LOW = 0x0052;
	public final static int SYSTEM__THRESH_RATE_LOW_HI = 0x0052;
	public final static int SYSTEM__THRESH_RATE_LOW_LO = 0x0053;
	public final static int DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT = 0x0054;
	public final static int DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT_HI = 0x0054;
	public final static int DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT_LO = 0x0055;
	public final static int DSS_CONFIG__MANUAL_BLOCK_SELECT = 0x0056;
	public final static int DSS_CONFIG__APERTURE_ATTENUATION = 0x0057;
	public final static int DSS_CONFIG__MAX_SPADS_LIMIT = 0x0058;
	public final static int DSS_CONFIG__MIN_SPADS_LIMIT = 0x0059;
	public final static int MM_CONFIG__TIMEOUT_MACROP_A = 0x005A; // added by Pololu for 16-bit accesses
	public final static int MM_CONFIG__TIMEOUT_MACROP_A_HI = 0x005A;
	public final static int MM_CONFIG__TIMEOUT_MACROP_A_LO = 0x005B;
	public final static int MM_CONFIG__TIMEOUT_MACROP_B = 0x005C; // added by Pololu for 16-bit accesses
	public final static int MM_CONFIG__TIMEOUT_MACROP_B_HI = 0x005C;
	public final static int MM_CONFIG__TIMEOUT_MACROP_B_LO = 0x005D;
	public final static int RANGE_CONFIG__TIMEOUT_MACROP_A = 0x005E; // added by Pololu for 16-bit accesses
	public final static int RANGE_CONFIG__TIMEOUT_MACROP_A_HI = 0x005E;
	public final static int RANGE_CONFIG__TIMEOUT_MACROP_A_LO = 0x005F;
	public final static int RANGE_CONFIG__VCSEL_PERIOD_A = 0x0060;
	public final static int RANGE_CONFIG__TIMEOUT_MACROP_B = 0x0061; // added by Pololu for 16-bit accesses
	public final static int RANGE_CONFIG__TIMEOUT_MACROP_B_HI = 0x0061;
	public final static int RANGE_CONFIG__TIMEOUT_MACROP_B_LO = 0x0062;
	public final static int RANGE_CONFIG__VCSEL_PERIOD_B = 0x0063;
	public final static int RANGE_CONFIG__SIGMA_THRESH = 0x0064;
	public final static int RANGE_CONFIG__SIGMA_THRESH_HI = 0x0064;
	public final static int RANGE_CONFIG__SIGMA_THRESH_LO = 0x0065;
	public final static int RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS = 0x0066;
	public final static int RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS_HI = 0x0066;
	public final static int RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS_LO = 0x0067;
	public final static int RANGE_CONFIG__VALID_PHASE_LOW = 0x0068;
	public final static int RANGE_CONFIG__VALID_PHASE_HIGH = 0x0069;
	public final static int SYSTEM__INTERMEASUREMENT_PERIOD = 0x006C;
	public final static int SYSTEM__INTERMEASUREMENT_PERIOD_3 = 0x006C;
	public final static int SYSTEM__INTERMEASUREMENT_PERIOD_2 = 0x006D;
	public final static int SYSTEM__INTERMEASUREMENT_PERIOD_1 = 0x006E;
	public final static int SYSTEM__INTERMEASUREMENT_PERIOD_0 = 0x006F;
	public final static int SYSTEM__FRACTIONAL_ENABLE = 0x0070;
	public final static int SYSTEM__GROUPED_PARAMETER_HOLD_0 = 0x0071;
	public final static int SYSTEM__THRESH_HIGH = 0x0072;
	public final static int SYSTEM__THRESH_HIGH_HI = 0x0072;
	public final static int SYSTEM__THRESH_HIGH_LO = 0x0073;
	public final static int SYSTEM__THRESH_LOW = 0x0074;
	public final static int SYSTEM__THRESH_LOW_HI = 0x0074;
	public final static int SYSTEM__THRESH_LOW_LO = 0x0075;
	public final static int SYSTEM__ENABLE_XTALK_PER_QUADRANT = 0x0076;
	public final static int SYSTEM__SEED_CONFIG = 0x0077;
	public final static int SD_CONFIG__WOI_SD0 = 0x0078;
	public final static int SD_CONFIG__WOI_SD1 = 0x0079;
	public final static int SD_CONFIG__INITIAL_PHASE_SD0 = 0x007A;
	public final static int SD_CONFIG__INITIAL_PHASE_SD1 = 0x007B;
	public final static int SYSTEM__GROUPED_PARAMETER_HOLD_1 = 0x007C;
	public final static int SD_CONFIG__FIRST_ORDER_SELECT = 0x007D;
	public final static int SD_CONFIG__QUANTIFIER = 0x007E;
	public final static int ROI_CONFIG__USER_ROI_CENTRE_SPAD = 0x007F;
	public final static int ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE = 0x0080;
	public final static int SYSTEM__SEQUENCE_CONFIG = 0x0081;
	public final static int SYSTEM__GROUPED_PARAMETER_HOLD = 0x0082;
	public final static int POWER_MANAGEMENT__GO1_POWER_FORCE = 0x0083;
	public final static int SYSTEM__STREAM_COUNT_CTRL = 0x0084;
	public final static int FIRMWARE__ENABLE = 0x0085;
	public final static int SYSTEM__INTERRUPT_CLEAR = 0x0086;
	public final static int SYSTEM__MODE_START = 0x0087;
	public final static int RESULT__INTERRUPT_STATUS = 0x0088;
	public final static int RESULT__RANGE_STATUS = 0x0089;
	public final static int RESULT__REPORT_STATUS = 0x008A;
	public final static int RESULT__STREAM_COUNT = 0x008B;
	public final static int RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0 = 0x008C;
	public final static int RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0_HI = 0x008C;
	public final static int RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0_LO = 0x008D;
	public final static int RESULT__PEAK_SIGNAL_COUNT_RATE_MCPS_SD0 = 0x008E;
	public final static int RESULT__PEAK_SIGNAL_COUNT_RATE_MCPS_SD0_HI = 0x008E;
	public final static int RESULT__PEAK_SIGNAL_COUNT_RATE_MCPS_SD0_LO = 0x008F;
	public final static int RESULT__AMBIENT_COUNT_RATE_MCPS_SD0 = 0x0090;
	public final static int RESULT__AMBIENT_COUNT_RATE_MCPS_SD0_HI = 0x0090;
	public final static int RESULT__AMBIENT_COUNT_RATE_MCPS_SD0_LO = 0x0091;
	public final static int RESULT__SIGMA_SD0 = 0x0092;
	public final static int RESULT__SIGMA_SD0_HI = 0x0092;
	public final static int RESULT__SIGMA_SD0_LO = 0x0093;
	public final static int RESULT__PHASE_SD0 = 0x0094;
	public final static int RESULT__PHASE_SD0_HI = 0x0094;
	public final static int RESULT__PHASE_SD0_LO = 0x0095;
	public final static int RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0 = 0x0096;
	public final static int RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0_HI = 0x0096;
	public final static int RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0_LO = 0x0097;
	public final static int RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0 = 0x0098;
	public final static int RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0_HI = 0x0098;
	public final static int RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0_LO = 0x0099;
	public final static int RESULT__MM_INNER_ACTUAL_EFFECTIVE_SPADS_SD0 = 0x009A;
	public final static int RESULT__MM_INNER_ACTUAL_EFFECTIVE_SPADS_SD0_HI = 0x009A;
	public final static int RESULT__MM_INNER_ACTUAL_EFFECTIVE_SPADS_SD0_LO = 0x009B;
	public final static int RESULT__MM_OUTER_ACTUAL_EFFECTIVE_SPADS_SD0 = 0x009C;
	public final static int RESULT__MM_OUTER_ACTUAL_EFFECTIVE_SPADS_SD0_HI = 0x009C;
	public final static int RESULT__MM_OUTER_ACTUAL_EFFECTIVE_SPADS_SD0_LO = 0x009D;
	public final static int RESULT__AVG_SIGNAL_COUNT_RATE_MCPS_SD0 = 0x009E;
	public final static int RESULT__AVG_SIGNAL_COUNT_RATE_MCPS_SD0_HI = 0x009E;
	public final static int RESULT__AVG_SIGNAL_COUNT_RATE_MCPS_SD0_LO = 0x009F;
	public final static int RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD1 = 0x00A0;
	public final static int RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD1_HI = 0x00A0;
	public final static int RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD1_LO = 0x00A1;
	public final static int RESULT__PEAK_SIGNAL_COUNT_RATE_MCPS_SD1 = 0x00A2;
	public final static int RESULT__PEAK_SIGNAL_COUNT_RATE_MCPS_SD1_HI = 0x00A2;
	public final static int RESULT__PEAK_SIGNAL_COUNT_RATE_MCPS_SD1_LO = 0x00A3;
	public final static int RESULT__AMBIENT_COUNT_RATE_MCPS_SD1 = 0x00A4;
	public final static int RESULT__AMBIENT_COUNT_RATE_MCPS_SD1_HI = 0x00A4;
	public final static int RESULT__AMBIENT_COUNT_RATE_MCPS_SD1_LO = 0x00A5;
	public final static int RESULT__SIGMA_SD1 = 0x00A6;
	public final static int RESULT__SIGMA_SD1_HI = 0x00A6;
	public final static int RESULT__SIGMA_SD1_LO = 0x00A7;
	public final static int RESULT__PHASE_SD1 = 0x00A8;
	public final static int RESULT__PHASE_SD1_HI = 0x00A8;
	public final static int RESULT__PHASE_SD1_LO = 0x00A9;
	public final static int RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD1 = 0x00AA;
	public final static int RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD1_HI = 0x00AA;
	public final static int RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD1_LO = 0x00AB;
	public final static int RESULT__SPARE_0_SD1 = 0x00AC;
	public final static int RESULT__SPARE_0_SD1_HI = 0x00AC;
	public final static int RESULT__SPARE_0_SD1_LO = 0x00AD;
	public final static int RESULT__SPARE_1_SD1 = 0x00AE;
	public final static int RESULT__SPARE_1_SD1_HI = 0x00AE;
	public final static int RESULT__SPARE_1_SD1_LO = 0x00AF;
	public final static int RESULT__SPARE_2_SD1 = 0x00B0;
	public final static int RESULT__SPARE_2_SD1_HI = 0x00B0;
	public final static int RESULT__SPARE_2_SD1_LO = 0x00B1;
	public final static int RESULT__SPARE_3_SD1 = 0x00B2;
	public final static int RESULT__THRESH_INFO = 0x00B3;
	public final static int RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD0 = 0x00B4;
	public final static int RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD0_3 = 0x00B4;
	public final static int RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD0_2 = 0x00B5;
	public final static int RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD0_1 = 0x00B6;
	public final static int RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD0_0 = 0x00B7;
	public final static int RESULT_CORE__RANGING_TOTAL_EVENTS_SD0 = 0x00B8;
	public final static int RESULT_CORE__RANGING_TOTAL_EVENTS_SD0_3 = 0x00B8;
	public final static int RESULT_CORE__RANGING_TOTAL_EVENTS_SD0_2 = 0x00B9;
	public final static int RESULT_CORE__RANGING_TOTAL_EVENTS_SD0_1 = 0x00BA;
	public final static int RESULT_CORE__RANGING_TOTAL_EVENTS_SD0_0 = 0x00BB;
	public final static int RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD0 = 0x00BC;
	public final static int RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD0_3 = 0x00BC;
	public final static int RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD0_2 = 0x00BD;
	public final static int RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD0_1 = 0x00BE;
	public final static int RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD0_0 = 0x00BF;
	public final static int RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD0 = 0x00C0;
	public final static int RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD0_3 = 0x00C0;
	public final static int RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD0_2 = 0x00C1;
	public final static int RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD0_1 = 0x00C2;
	public final static int RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD0_0 = 0x00C3;
	public final static int RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD1 = 0x00C4;
	public final static int RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD1_3 = 0x00C4;
	public final static int RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD1_2 = 0x00C5;
	public final static int RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD1_1 = 0x00C6;
	public final static int RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD1_0 = 0x00C7;
	public final static int RESULT_CORE__RANGING_TOTAL_EVENTS_SD1 = 0x00C8;
	public final static int RESULT_CORE__RANGING_TOTAL_EVENTS_SD1_3 = 0x00C8;
	public final static int RESULT_CORE__RANGING_TOTAL_EVENTS_SD1_2 = 0x00C9;
	public final static int RESULT_CORE__RANGING_TOTAL_EVENTS_SD1_1 = 0x00CA;
	public final static int RESULT_CORE__RANGING_TOTAL_EVENTS_SD1_0 = 0x00CB;
	public final static int RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD1 = 0x00CC;
	public final static int RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD1_3 = 0x00CC;
	public final static int RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD1_2 = 0x00CD;
	public final static int RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD1_1 = 0x00CE;
	public final static int RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD1_0 = 0x00CF;
	public final static int RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD1 = 0x00D0;
	public final static int RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD1_3 = 0x00D0;
	public final static int RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD1_2 = 0x00D1;
	public final static int RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD1_1 = 0x00D2;
	public final static int RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD1_0 = 0x00D3;
	public final static int RESULT_CORE__SPARE_0 = 0x00D4;
	public final static int PHASECAL_RESULT__REFERENCE_PHASE = 0x00D6;
	public final static int PHASECAL_RESULT__REFERENCE_PHASE_HI = 0x00D6;
	public final static int PHASECAL_RESULT__REFERENCE_PHASE_LO = 0x00D7;
	public final static int PHASECAL_RESULT__VCSEL_START = 0x00D8;
	public final static int REF_SPAD_CHAR_RESULT__NUM_ACTUAL_REF_SPADS = 0x00D9;
	public final static int REF_SPAD_CHAR_RESULT__REF_LOCATION = 0x00DA;
	public final static int VHV_RESULT__COLDBOOT_STATUS = 0x00DB;
	public final static int VHV_RESULT__SEARCH_RESULT = 0x00DC;
	public final static int VHV_RESULT__LATEST_SETTING = 0x00DD;
	public final static int RESULT__OSC_CALIBRATE_VAL = 0x00DE;
	public final static int RESULT__OSC_CALIBRATE_VAL_HI = 0x00DE;
	public final static int RESULT__OSC_CALIBRATE_VAL_LO = 0x00DF;
	public final static int ANA_CONFIG__POWERDOWN_GO1 = 0x00E0;
	public final static int ANA_CONFIG__REF_BG_CTRL = 0x00E1;
	public final static int ANA_CONFIG__REGDVDD1V2_CTRL = 0x00E2;
	public final static int ANA_CONFIG__OSC_SLOW_CTRL = 0x00E3;
	public final static int TEST_MODE__STATUS = 0x00E4;
	public final static int FIRMWARE__SYSTEM_STATUS = 0x00E5;
	public final static int FIRMWARE__MODE_STATUS = 0x00E6;
	public final static int FIRMWARE__SECONDARY_MODE_STATUS = 0x00E7;
	public final static int FIRMWARE__CAL_REPEAT_RATE_COUNTER = 0x00E8;
	public final static int FIRMWARE__CAL_REPEAT_RATE_COUNTER_HI = 0x00E8;
	public final static int FIRMWARE__CAL_REPEAT_RATE_COUNTER_LO = 0x00E9;
	public final static int FIRMWARE__HISTOGRAM_BIN = 0x00EA;
	public final static int GPH__SYSTEM__THRESH_HIGH = 0x00EC;
	public final static int GPH__SYSTEM__THRESH_HIGH_HI = 0x00EC;
	public final static int GPH__SYSTEM__THRESH_HIGH_LO = 0x00ED;
	public final static int GPH__SYSTEM__THRESH_LOW = 0x00EE;
	public final static int GPH__SYSTEM__THRESH_LOW_HI = 0x00EE;
	public final static int GPH__SYSTEM__THRESH_LOW_LO = 0x00EF;
	public final static int GPH__SYSTEM__ENABLE_XTALK_PER_QUADRANT = 0x00F0;
	public final static int GPH__SPARE_0 = 0x00F1;
	public final static int GPH__SD_CONFIG__WOI_SD0 = 0x00F2;
	public final static int GPH__SD_CONFIG__WOI_SD1 = 0x00F3;
	public final static int GPH__SD_CONFIG__INITIAL_PHASE_SD0 = 0x00F4;
	public final static int GPH__SD_CONFIG__INITIAL_PHASE_SD1 = 0x00F5;
	public final static int GPH__SD_CONFIG__FIRST_ORDER_SELECT = 0x00F6;
	public final static int GPH__SD_CONFIG__QUANTIFIER = 0x00F7;
	public final static int GPH__ROI_CONFIG__USER_ROI_CENTRE_SPAD = 0x00F8;
	public final static int GPH__ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE = 0x00F9;
	public final static int GPH__SYSTEM__SEQUENCE_CONFIG = 0x00FA;
	public final static int GPH__GPH_ID = 0x00FB;
	public final static int SYSTEM__INTERRUPT_SET = 0x00FC;
	public final static int INTERRUPT_MANAGER__ENABLES = 0x00FD;
	public final static int INTERRUPT_MANAGER__CLEAR = 0x00FE;
	public final static int INTERRUPT_MANAGER__STATUS = 0x00FF;
	public final static int MCU_TO_HOST_BANK__WR_ACCESS_EN = 0x0100;
	public final static int POWER_MANAGEMENT__GO1_RESET_STATUS = 0x0101;
	public final static int PAD_STARTUP_MODE__VALUE_RO = 0x0102;
	public final static int PAD_STARTUP_MODE__VALUE_CTRL = 0x0103;
	public final static int PLL_PERIOD_US = 0x0104;
	public final static int PLL_PERIOD_US_3 = 0x0104;
	public final static int PLL_PERIOD_US_2 = 0x0105;
	public final static int PLL_PERIOD_US_1 = 0x0106;
	public final static int PLL_PERIOD_US_0 = 0x0107;
	public final static int INTERRUPT_SCHEDULER__DATA_OUT = 0x0108;
	public final static int INTERRUPT_SCHEDULER__DATA_OUT_3 = 0x0108;
	public final static int INTERRUPT_SCHEDULER__DATA_OUT_2 = 0x0109;
	public final static int INTERRUPT_SCHEDULER__DATA_OUT_1 = 0x010A;
	public final static int INTERRUPT_SCHEDULER__DATA_OUT_0 = 0x010B;
	public final static int NVM_BIST__COMPLETE = 0x010C;
	public final static int NVM_BIST__STATUS = 0x010D;
	public final static int IDENTIFICATION__MODEL_ID = 0x010F;
	public final static int IDENTIFICATION__MODULE_TYPE = 0x0110;
	public final static int IDENTIFICATION__REVISION_ID = 0x0111;
	public final static int IDENTIFICATION__MODULE_ID = 0x0112;
	public final static int IDENTIFICATION__MODULE_ID_HI = 0x0112;
	public final static int IDENTIFICATION__MODULE_ID_LO = 0x0113;
	public final static int ANA_CONFIG__FAST_OSC__TRIM_MAX = 0x0114;
	public final static int ANA_CONFIG__FAST_OSC__FREQ_SET = 0x0115;
	public final static int ANA_CONFIG__VCSEL_TRIM = 0x0116;
	public final static int ANA_CONFIG__VCSEL_SELION = 0x0117;
	public final static int ANA_CONFIG__VCSEL_SELION_MAX = 0x0118;
	public final static int PROTECTED_LASER_SAFETY__LOCK_BIT = 0x0119;
	public final static int LASER_SAFETY__KEY = 0x011A;
	public final static int LASER_SAFETY__KEY_RO = 0x011B;
	public final static int LASER_SAFETY__CLIP = 0x011C;
	public final static int LASER_SAFETY__MULT = 0x011D;
	public final static int GLOBAL_CONFIG__SPAD_ENABLES_RTN_0 = 0x011E;
	public final static int GLOBAL_CONFIG__SPAD_ENABLES_RTN_1 = 0x011F;
	public final static int GLOBAL_CONFIG__SPAD_ENABLES_RTN_2 = 0x0120;
	public final static int GLOBAL_CONFIG__SPAD_ENABLES_RTN_3 = 0x0121;
	public final static int GLOBAL_CONFIG__SPAD_ENABLES_RTN_4 = 0x0122;
	public final static int GLOBAL_CONFIG__SPAD_ENABLES_RTN_5 = 0x0123;
	public final static int GLOBAL_CONFIG__SPAD_ENABLES_RTN_6 = 0x0124;
	public final static int GLOBAL_CONFIG__SPAD_ENABLES_RTN_7 = 0x0125;
	public final static int GLOBAL_CONFIG__SPAD_ENABLES_RTN_8 = 0x0126;
	public final static int GLOBAL_CONFIG__SPAD_ENABLES_RTN_9 = 0x0127;
	public final static int GLOBAL_CONFIG__SPAD_ENABLES_RTN_10 = 0x0128;
	public final static int GLOBAL_CONFIG__SPAD_ENABLES_RTN_11 = 0x0129;
	public final static int GLOBAL_CONFIG__SPAD_ENABLES_RTN_12 = 0x012A;
	public final static int GLOBAL_CONFIG__SPAD_ENABLES_RTN_13 = 0x012B;
	public final static int GLOBAL_CONFIG__SPAD_ENABLES_RTN_14 = 0x012C;
	public final static int GLOBAL_CONFIG__SPAD_ENABLES_RTN_15 = 0x012D;
	public final static int GLOBAL_CONFIG__SPAD_ENABLES_RTN_16 = 0x012E;
	public final static int GLOBAL_CONFIG__SPAD_ENABLES_RTN_17 = 0x012F;
	public final static int GLOBAL_CONFIG__SPAD_ENABLES_RTN_18 = 0x0130;
	public final static int GLOBAL_CONFIG__SPAD_ENABLES_RTN_19 = 0x0131;
	public final static int GLOBAL_CONFIG__SPAD_ENABLES_RTN_20 = 0x0132;
	public final static int GLOBAL_CONFIG__SPAD_ENABLES_RTN_21 = 0x0133;
	public final static int GLOBAL_CONFIG__SPAD_ENABLES_RTN_22 = 0x0134;
	public final static int GLOBAL_CONFIG__SPAD_ENABLES_RTN_23 = 0x0135;
	public final static int GLOBAL_CONFIG__SPAD_ENABLES_RTN_24 = 0x0136;
	public final static int GLOBAL_CONFIG__SPAD_ENABLES_RTN_25 = 0x0137;
	public final static int GLOBAL_CONFIG__SPAD_ENABLES_RTN_26 = 0x0138;
	public final static int GLOBAL_CONFIG__SPAD_ENABLES_RTN_27 = 0x0139;
	public final static int GLOBAL_CONFIG__SPAD_ENABLES_RTN_28 = 0x013A;
	public final static int GLOBAL_CONFIG__SPAD_ENABLES_RTN_29 = 0x013B;
	public final static int GLOBAL_CONFIG__SPAD_ENABLES_RTN_30 = 0x013C;
	public final static int GLOBAL_CONFIG__SPAD_ENABLES_RTN_31 = 0x013D;
	public final static int ROI_CONFIG__MODE_ROI_CENTRE_SPAD = 0x013E;
	public final static int ROI_CONFIG__MODE_ROI_XY_SIZE = 0x013F;
	public final static int GO2_HOST_BANK_ACCESS__OVERRIDE = 0x0300;
	public final static int MCU_UTIL_MULTIPLIER__MULTIPLICAND = 0x0400;
	public final static int MCU_UTIL_MULTIPLIER__MULTIPLICAND_3 = 0x0400;
	public final static int MCU_UTIL_MULTIPLIER__MULTIPLICAND_2 = 0x0401;
	public final static int MCU_UTIL_MULTIPLIER__MULTIPLICAND_1 = 0x0402;
	public final static int MCU_UTIL_MULTIPLIER__MULTIPLICAND_0 = 0x0403;
	public final static int MCU_UTIL_MULTIPLIER__MULTIPLIER = 0x0404;
	public final static int MCU_UTIL_MULTIPLIER__MULTIPLIER_3 = 0x0404;
	public final static int MCU_UTIL_MULTIPLIER__MULTIPLIER_2 = 0x0405;
	public final static int MCU_UTIL_MULTIPLIER__MULTIPLIER_1 = 0x0406;
	public final static int MCU_UTIL_MULTIPLIER__MULTIPLIER_0 = 0x0407;
	public final static int MCU_UTIL_MULTIPLIER__PRODUCT_HI = 0x0408;
	public final static int MCU_UTIL_MULTIPLIER__PRODUCT_HI_3 = 0x0408;
	public final static int MCU_UTIL_MULTIPLIER__PRODUCT_HI_2 = 0x0409;
	public final static int MCU_UTIL_MULTIPLIER__PRODUCT_HI_1 = 0x040A;
	public final static int MCU_UTIL_MULTIPLIER__PRODUCT_HI_0 = 0x040B;
	public final static int MCU_UTIL_MULTIPLIER__PRODUCT_LO = 0x040C;
	public final static int MCU_UTIL_MULTIPLIER__PRODUCT_LO_3 = 0x040C;
	public final static int MCU_UTIL_MULTIPLIER__PRODUCT_LO_2 = 0x040D;
	public final static int MCU_UTIL_MULTIPLIER__PRODUCT_LO_1 = 0x040E;
	public final static int MCU_UTIL_MULTIPLIER__PRODUCT_LO_0 = 0x040F;
	public final static int MCU_UTIL_MULTIPLIER__START = 0x0410;
	public final static int MCU_UTIL_MULTIPLIER__STATUS = 0x0411;
	public final static int MCU_UTIL_DIVIDER__START = 0x0412;
	public final static int MCU_UTIL_DIVIDER__STATUS = 0x0413;
	public final static int MCU_UTIL_DIVIDER__DIVIDEND = 0x0414;
	public final static int MCU_UTIL_DIVIDER__DIVIDEND_3 = 0x0414;
	public final static int MCU_UTIL_DIVIDER__DIVIDEND_2 = 0x0415;
	public final static int MCU_UTIL_DIVIDER__DIVIDEND_1 = 0x0416;
	public final static int MCU_UTIL_DIVIDER__DIVIDEND_0 = 0x0417;
	public final static int MCU_UTIL_DIVIDER__DIVISOR = 0x0418;
	public final static int MCU_UTIL_DIVIDER__DIVISOR_3 = 0x0418;
	public final static int MCU_UTIL_DIVIDER__DIVISOR_2 = 0x0419;
	public final static int MCU_UTIL_DIVIDER__DIVISOR_1 = 0x041A;
	public final static int MCU_UTIL_DIVIDER__DIVISOR_0 = 0x041B;
	public final static int MCU_UTIL_DIVIDER__QUOTIENT = 0x041C;
	public final static int MCU_UTIL_DIVIDER__QUOTIENT_3 = 0x041C;
	public final static int MCU_UTIL_DIVIDER__QUOTIENT_2 = 0x041D;
	public final static int MCU_UTIL_DIVIDER__QUOTIENT_1 = 0x041E;
	public final static int MCU_UTIL_DIVIDER__QUOTIENT_0 = 0x041F;
	public final static int TIMER0__VALUE_IN = 0x0420;
	public final static int TIMER0__VALUE_IN_3 = 0x0420;
	public final static int TIMER0__VALUE_IN_2 = 0x0421;
	public final static int TIMER0__VALUE_IN_1 = 0x0422;
	public final static int TIMER0__VALUE_IN_0 = 0x0423;
	public final static int TIMER1__VALUE_IN = 0x0424;
	public final static int TIMER1__VALUE_IN_3 = 0x0424;
	public final static int TIMER1__VALUE_IN_2 = 0x0425;
	public final static int TIMER1__VALUE_IN_1 = 0x0426;
	public final static int TIMER1__VALUE_IN_0 = 0x0427;
	public final static int TIMER0__CTRL = 0x0428;
	public final static int TIMER1__CTRL = 0x0429;
	public final static int MCU_GENERAL_PURPOSE__GP_0 = 0x042C;
	public final static int MCU_GENERAL_PURPOSE__GP_1 = 0x042D;
	public final static int MCU_GENERAL_PURPOSE__GP_2 = 0x042E;
	public final static int MCU_GENERAL_PURPOSE__GP_3 = 0x042F;
	public final static int MCU_RANGE_CALC__CONFIG = 0x0430;
	public final static int MCU_RANGE_CALC__OFFSET_CORRECTED_RANGE = 0x0432;
	public final static int MCU_RANGE_CALC__OFFSET_CORRECTED_RANGE_HI = 0x0432;
	public final static int MCU_RANGE_CALC__OFFSET_CORRECTED_RANGE_LO = 0x0433;
	public final static int MCU_RANGE_CALC__SPARE_4 = 0x0434;
	public final static int MCU_RANGE_CALC__SPARE_4_3 = 0x0434;
	public final static int MCU_RANGE_CALC__SPARE_4_2 = 0x0435;
	public final static int MCU_RANGE_CALC__SPARE_4_1 = 0x0436;
	public final static int MCU_RANGE_CALC__SPARE_4_0 = 0x0437;
	public final static int MCU_RANGE_CALC__AMBIENT_DURATION_PRE_CALC = 0x0438;
	public final static int MCU_RANGE_CALC__AMBIENT_DURATION_PRE_CALC_HI = 0x0438;
	public final static int MCU_RANGE_CALC__AMBIENT_DURATION_PRE_CALC_LO = 0x0439;
	public final static int MCU_RANGE_CALC__ALGO_VCSEL_PERIOD = 0x043C;
	public final static int MCU_RANGE_CALC__SPARE_5 = 0x043D;
	public final static int MCU_RANGE_CALC__ALGO_TOTAL_PERIODS = 0x043E;
	public final static int MCU_RANGE_CALC__ALGO_TOTAL_PERIODS_HI = 0x043E;
	public final static int MCU_RANGE_CALC__ALGO_TOTAL_PERIODS_LO = 0x043F;
	public final static int MCU_RANGE_CALC__ALGO_ACCUM_PHASE = 0x0440;
	public final static int MCU_RANGE_CALC__ALGO_ACCUM_PHASE_3 = 0x0440;
	public final static int MCU_RANGE_CALC__ALGO_ACCUM_PHASE_2 = 0x0441;
	public final static int MCU_RANGE_CALC__ALGO_ACCUM_PHASE_1 = 0x0442;
	public final static int MCU_RANGE_CALC__ALGO_ACCUM_PHASE_0 = 0x0443;
	public final static int MCU_RANGE_CALC__ALGO_SIGNAL_EVENTS = 0x0444;
	public final static int MCU_RANGE_CALC__ALGO_SIGNAL_EVENTS_3 = 0x0444;
	public final static int MCU_RANGE_CALC__ALGO_SIGNAL_EVENTS_2 = 0x0445;
	public final static int MCU_RANGE_CALC__ALGO_SIGNAL_EVENTS_1 = 0x0446;
	public final static int MCU_RANGE_CALC__ALGO_SIGNAL_EVENTS_0 = 0x0447;
	public final static int MCU_RANGE_CALC__ALGO_AMBIENT_EVENTS = 0x0448;
	public final static int MCU_RANGE_CALC__ALGO_AMBIENT_EVENTS_3 = 0x0448;
	public final static int MCU_RANGE_CALC__ALGO_AMBIENT_EVENTS_2 = 0x0449;
	public final static int MCU_RANGE_CALC__ALGO_AMBIENT_EVENTS_1 = 0x044A;
	public final static int MCU_RANGE_CALC__ALGO_AMBIENT_EVENTS_0 = 0x044B;
	public final static int MCU_RANGE_CALC__SPARE_6 = 0x044C;
	public final static int MCU_RANGE_CALC__SPARE_6_HI = 0x044C;
	public final static int MCU_RANGE_CALC__SPARE_6_LO = 0x044D;
	public final static int MCU_RANGE_CALC__ALGO_ADJUST_VCSEL_PERIOD = 0x044E;
	public final static int MCU_RANGE_CALC__ALGO_ADJUST_VCSEL_PERIOD_HI = 0x044E;
	public final static int MCU_RANGE_CALC__ALGO_ADJUST_VCSEL_PERIOD_LO = 0x044F;
	public final static int MCU_RANGE_CALC__NUM_SPADS = 0x0450;
	public final static int MCU_RANGE_CALC__NUM_SPADS_HI = 0x0450;
	public final static int MCU_RANGE_CALC__NUM_SPADS_LO = 0x0451;
	public final static int MCU_RANGE_CALC__PHASE_OUTPUT = 0x0452;
	public final static int MCU_RANGE_CALC__PHASE_OUTPUT_HI = 0x0452;
	public final static int MCU_RANGE_CALC__PHASE_OUTPUT_LO = 0x0453;
	public final static int MCU_RANGE_CALC__RATE_PER_SPAD_MCPS = 0x0454;
	public final static int MCU_RANGE_CALC__RATE_PER_SPAD_MCPS_3 = 0x0454;
	public final static int MCU_RANGE_CALC__RATE_PER_SPAD_MCPS_2 = 0x0455;
	public final static int MCU_RANGE_CALC__RATE_PER_SPAD_MCPS_1 = 0x0456;
	public final static int MCU_RANGE_CALC__RATE_PER_SPAD_MCPS_0 = 0x0457;
	public final static int MCU_RANGE_CALC__SPARE_7 = 0x0458;
	public final static int MCU_RANGE_CALC__SPARE_8 = 0x0459;
	public final static int MCU_RANGE_CALC__PEAK_SIGNAL_RATE_MCPS = 0x045A;
	public final static int MCU_RANGE_CALC__PEAK_SIGNAL_RATE_MCPS_HI = 0x045A;
	public final static int MCU_RANGE_CALC__PEAK_SIGNAL_RATE_MCPS_LO = 0x045B;
	public final static int MCU_RANGE_CALC__AVG_SIGNAL_RATE_MCPS = 0x045C;
	public final static int MCU_RANGE_CALC__AVG_SIGNAL_RATE_MCPS_HI = 0x045C;
	public final static int MCU_RANGE_CALC__AVG_SIGNAL_RATE_MCPS_LO = 0x045D;
	public final static int MCU_RANGE_CALC__AMBIENT_RATE_MCPS = 0x045E;
	public final static int MCU_RANGE_CALC__AMBIENT_RATE_MCPS_HI = 0x045E;
	public final static int MCU_RANGE_CALC__AMBIENT_RATE_MCPS_LO = 0x045F;
	public final static int MCU_RANGE_CALC__XTALK = 0x0460;
	public final static int MCU_RANGE_CALC__XTALK_HI = 0x0460;
	public final static int MCU_RANGE_CALC__XTALK_LO = 0x0461;
	public final static int MCU_RANGE_CALC__CALC_STATUS = 0x0462;
	public final static int MCU_RANGE_CALC__DEBUG = 0x0463;
	public final static int MCU_RANGE_CALC__PEAK_SIGNAL_RATE_XTALK_CORR_MCPS = 0x0464;
	public final static int MCU_RANGE_CALC__PEAK_SIGNAL_RATE_XTALK_CORR_MCPS_HI = 0x0464;
	public final static int MCU_RANGE_CALC__PEAK_SIGNAL_RATE_XTALK_CORR_MCPS_LO = 0x0465;
	public final static int MCU_RANGE_CALC__SPARE_0 = 0x0468;
	public final static int MCU_RANGE_CALC__SPARE_1 = 0x0469;
	public final static int MCU_RANGE_CALC__SPARE_2 = 0x046A;
	public final static int MCU_RANGE_CALC__SPARE_3 = 0x046B;
	public final static int PATCH__CTRL = 0x0470;
	public final static int PATCH__JMP_ENABLES = 0x0472;
	public final static int PATCH__JMP_ENABLES_HI = 0x0472;
	public final static int PATCH__JMP_ENABLES_LO = 0x0473;
	public final static int PATCH__DATA_ENABLES = 0x0474;
	public final static int PATCH__DATA_ENABLES_HI = 0x0474;
	public final static int PATCH__DATA_ENABLES_LO = 0x0475;
	public final static int PATCH__OFFSET_0 = 0x0476;
	public final static int PATCH__OFFSET_0_HI = 0x0476;
	public final static int PATCH__OFFSET_0_LO = 0x0477;
	public final static int PATCH__OFFSET_1 = 0x0478;
	public final static int PATCH__OFFSET_1_HI = 0x0478;
	public final static int PATCH__OFFSET_1_LO = 0x0479;
	public final static int PATCH__OFFSET_2 = 0x047A;
	public final static int PATCH__OFFSET_2_HI = 0x047A;
	public final static int PATCH__OFFSET_2_LO = 0x047B;
	public final static int PATCH__OFFSET_3 = 0x047C;
	public final static int PATCH__OFFSET_3_HI = 0x047C;
	public final static int PATCH__OFFSET_3_LO = 0x047D;
	public final static int PATCH__OFFSET_4 = 0x047E;
	public final static int PATCH__OFFSET_4_HI = 0x047E;
	public final static int PATCH__OFFSET_4_LO = 0x047F;
	public final static int PATCH__OFFSET_5 = 0x0480;
	public final static int PATCH__OFFSET_5_HI = 0x0480;
	public final static int PATCH__OFFSET_5_LO = 0x0481;
	public final static int PATCH__OFFSET_6 = 0x0482;
	public final static int PATCH__OFFSET_6_HI = 0x0482;
	public final static int PATCH__OFFSET_6_LO = 0x0483;
	public final static int PATCH__OFFSET_7 = 0x0484;
	public final static int PATCH__OFFSET_7_HI = 0x0484;
	public final static int PATCH__OFFSET_7_LO = 0x0485;
	public final static int PATCH__OFFSET_8 = 0x0486;
	public final static int PATCH__OFFSET_8_HI = 0x0486;
	public final static int PATCH__OFFSET_8_LO = 0x0487;
	public final static int PATCH__OFFSET_9 = 0x0488;
	public final static int PATCH__OFFSET_9_HI = 0x0488;
	public final static int PATCH__OFFSET_9_LO = 0x0489;
	public final static int PATCH__OFFSET_10 = 0x048A;
	public final static int PATCH__OFFSET_10_HI = 0x048A;
	public final static int PATCH__OFFSET_10_LO = 0x048B;
	public final static int PATCH__OFFSET_11 = 0x048C;
	public final static int PATCH__OFFSET_11_HI = 0x048C;
	public final static int PATCH__OFFSET_11_LO = 0x048D;
	public final static int PATCH__OFFSET_12 = 0x048E;
	public final static int PATCH__OFFSET_12_HI = 0x048E;
	public final static int PATCH__OFFSET_12_LO = 0x048F;
	public final static int PATCH__OFFSET_13 = 0x0490;
	public final static int PATCH__OFFSET_13_HI = 0x0490;
	public final static int PATCH__OFFSET_13_LO = 0x0491;
	public final static int PATCH__OFFSET_14 = 0x0492;
	public final static int PATCH__OFFSET_14_HI = 0x0492;
	public final static int PATCH__OFFSET_14_LO = 0x0493;
	public final static int PATCH__OFFSET_15 = 0x0494;
	public final static int PATCH__OFFSET_15_HI = 0x0494;
	public final static int PATCH__OFFSET_15_LO = 0x0495;
	public final static int PATCH__ADDRESS_0 = 0x0496;
	public final static int PATCH__ADDRESS_0_HI = 0x0496;
	public final static int PATCH__ADDRESS_0_LO = 0x0497;
	public final static int PATCH__ADDRESS_1 = 0x0498;
	public final static int PATCH__ADDRESS_1_HI = 0x0498;
	public final static int PATCH__ADDRESS_1_LO = 0x0499;
	public final static int PATCH__ADDRESS_2 = 0x049A;
	public final static int PATCH__ADDRESS_2_HI = 0x049A;
	public final static int PATCH__ADDRESS_2_LO = 0x049B;
	public final static int PATCH__ADDRESS_3 = 0x049C;
	public final static int PATCH__ADDRESS_3_HI = 0x049C;
	public final static int PATCH__ADDRESS_3_LO = 0x049D;
	public final static int PATCH__ADDRESS_4 = 0x049E;
	public final static int PATCH__ADDRESS_4_HI = 0x049E;
	public final static int PATCH__ADDRESS_4_LO = 0x049F;
	public final static int PATCH__ADDRESS_5 = 0x04A0;
	public final static int PATCH__ADDRESS_5_HI = 0x04A0;
	public final static int PATCH__ADDRESS_5_LO = 0x04A1;
	public final static int PATCH__ADDRESS_6 = 0x04A2;
	public final static int PATCH__ADDRESS_6_HI = 0x04A2;
	public final static int PATCH__ADDRESS_6_LO = 0x04A3;
	public final static int PATCH__ADDRESS_7 = 0x04A4;
	public final static int PATCH__ADDRESS_7_HI = 0x04A4;
	public final static int PATCH__ADDRESS_7_LO = 0x04A5;
	public final static int PATCH__ADDRESS_8 = 0x04A6;
	public final static int PATCH__ADDRESS_8_HI = 0x04A6;
	public final static int PATCH__ADDRESS_8_LO = 0x04A7;
	public final static int PATCH__ADDRESS_9 = 0x04A8;
	public final static int PATCH__ADDRESS_9_HI = 0x04A8;
	public final static int PATCH__ADDRESS_9_LO = 0x04A9;
	public final static int PATCH__ADDRESS_10 = 0x04AA;
	public final static int PATCH__ADDRESS_10_HI = 0x04AA;
	public final static int PATCH__ADDRESS_10_LO = 0x04AB;
	public final static int PATCH__ADDRESS_11 = 0x04AC;
	public final static int PATCH__ADDRESS_11_HI = 0x04AC;
	public final static int PATCH__ADDRESS_11_LO = 0x04AD;
	public final static int PATCH__ADDRESS_12 = 0x04AE;
	public final static int PATCH__ADDRESS_12_HI = 0x04AE;
	public final static int PATCH__ADDRESS_12_LO = 0x04AF;
	public final static int PATCH__ADDRESS_13 = 0x04B0;
	public final static int PATCH__ADDRESS_13_HI = 0x04B0;
	public final static int PATCH__ADDRESS_13_LO = 0x04B1;
	public final static int PATCH__ADDRESS_14 = 0x04B2;
	public final static int PATCH__ADDRESS_14_HI = 0x04B2;
	public final static int PATCH__ADDRESS_14_LO = 0x04B3;
	public final static int PATCH__ADDRESS_15 = 0x04B4;
	public final static int PATCH__ADDRESS_15_HI = 0x04B4;
	public final static int PATCH__ADDRESS_15_LO = 0x04B5;
	public final static int SPI_ASYNC_MUX__CTRL = 0x04C0;
	public final static int CLK__CONFIG = 0x04C4;
	public final static int GPIO_LV_MUX__CTRL = 0x04CC;
	public final static int GPIO_LV_PAD__CTRL = 0x04CD;
	public final static int PAD_I2C_LV__CONFIG = 0x04D0;
	public final static int PAD_STARTUP_MODE__VALUE_RO_GO1 = 0x04D4;
	public final static int HOST_IF__STATUS_GO1 = 0x04D5;
	public final static int MCU_CLK_GATING__CTRL = 0x04D8;
	public final static int TEST__BIST_ROM_CTRL = 0x04E0;
	public final static int TEST__BIST_ROM_RESULT = 0x04E1;
	public final static int TEST__BIST_ROM_MCU_SIG = 0x04E2;
	public final static int TEST__BIST_ROM_MCU_SIG_HI = 0x04E2;
	public final static int TEST__BIST_ROM_MCU_SIG_LO = 0x04E3;
	public final static int TEST__BIST_RAM_CTRL = 0x04E4;
	public final static int TEST__BIST_RAM_RESULT = 0x04E5;
	public final static int TEST__TMC = 0x04E8;
	public final static int TEST__PLL_BIST_MIN_THRESHOLD = 0x04F0;
	public final static int TEST__PLL_BIST_MIN_THRESHOLD_HI = 0x04F0;
	public final static int TEST__PLL_BIST_MIN_THRESHOLD_LO = 0x04F1;
	public final static int TEST__PLL_BIST_MAX_THRESHOLD = 0x04F2;
	public final static int TEST__PLL_BIST_MAX_THRESHOLD_HI = 0x04F2;
	public final static int TEST__PLL_BIST_MAX_THRESHOLD_LO = 0x04F3;
	public final static int TEST__PLL_BIST_COUNT_OUT = 0x04F4;
	public final static int TEST__PLL_BIST_COUNT_OUT_HI = 0x04F4;
	public final static int TEST__PLL_BIST_COUNT_OUT_LO = 0x04F5;
	public final static int TEST__PLL_BIST_GONOGO = 0x04F6;
	public final static int TEST__PLL_BIST_CTRL = 0x04F7;
	public final static int RANGING_CORE__DEVICE_ID = 0x0680;
	public final static int RANGING_CORE__REVISION_ID = 0x0681;
	public final static int RANGING_CORE__CLK_CTRL1 = 0x0683;
	public final static int RANGING_CORE__CLK_CTRL2 = 0x0684;
	public final static int RANGING_CORE__WOI_1 = 0x0685;
	public final static int RANGING_CORE__WOI_REF_1 = 0x0686;
	public final static int RANGING_CORE__START_RANGING = 0x0687;
	public final static int RANGING_CORE__LOW_LIMIT_1 = 0x0690;
	public final static int RANGING_CORE__HIGH_LIMIT_1 = 0x0691;
	public final static int RANGING_CORE__LOW_LIMIT_REF_1 = 0x0692;
	public final static int RANGING_CORE__HIGH_LIMIT_REF_1 = 0x0693;
	public final static int RANGING_CORE__QUANTIFIER_1_MSB = 0x0694;
	public final static int RANGING_CORE__QUANTIFIER_1_LSB = 0x0695;
	public final static int RANGING_CORE__QUANTIFIER_REF_1_MSB = 0x0696;
	public final static int RANGING_CORE__QUANTIFIER_REF_1_LSB = 0x0697;
	public final static int RANGING_CORE__AMBIENT_OFFSET_1_MSB = 0x0698;
	public final static int RANGING_CORE__AMBIENT_OFFSET_1_LSB = 0x0699;
	public final static int RANGING_CORE__AMBIENT_OFFSET_REF_1_MSB = 0x069A;
	public final static int RANGING_CORE__AMBIENT_OFFSET_REF_1_LSB = 0x069B;
	public final static int RANGING_CORE__FILTER_STRENGTH_1 = 0x069C;
	public final static int RANGING_CORE__FILTER_STRENGTH_REF_1 = 0x069D;
	public final static int RANGING_CORE__SIGNAL_EVENT_LIMIT_1_MSB = 0x069E;
	public final static int RANGING_CORE__SIGNAL_EVENT_LIMIT_1_LSB = 0x069F;
	public final static int RANGING_CORE__SIGNAL_EVENT_LIMIT_REF_1_MSB = 0x06A0;
	public final static int RANGING_CORE__SIGNAL_EVENT_LIMIT_REF_1_LSB = 0x06A1;
	public final static int RANGING_CORE__TIMEOUT_OVERALL_PERIODS_MSB = 0x06A4;
	public final static int RANGING_CORE__TIMEOUT_OVERALL_PERIODS_LSB = 0x06A5;
	public final static int RANGING_CORE__INVERT_HW = 0x06A6;
	public final static int RANGING_CORE__FORCE_HW = 0x06A7;
	public final static int RANGING_CORE__STATIC_HW_VALUE = 0x06A8;
	public final static int RANGING_CORE__FORCE_CONTINUOUS_AMBIENT = 0x06A9;
	public final static int RANGING_CORE__TEST_PHASE_SELECT_TO_FILTER = 0x06AA;
	public final static int RANGING_CORE__TEST_PHASE_SELECT_TO_TIMING_GEN = 0x06AB;
	public final static int RANGING_CORE__INITIAL_PHASE_VALUE_1 = 0x06AC;
	public final static int RANGING_CORE__INITIAL_PHASE_VALUE_REF_1 = 0x06AD;
	public final static int RANGING_CORE__FORCE_UP_IN = 0x06AE;
	public final static int RANGING_CORE__FORCE_DN_IN = 0x06AF;
	public final static int RANGING_CORE__STATIC_UP_VALUE_1 = 0x06B0;
	public final static int RANGING_CORE__STATIC_UP_VALUE_REF_1 = 0x06B1;
	public final static int RANGING_CORE__STATIC_DN_VALUE_1 = 0x06B2;
	public final static int RANGING_CORE__STATIC_DN_VALUE_REF_1 = 0x06B3;
	public final static int RANGING_CORE__MONITOR_UP_DN = 0x06B4;
	public final static int RANGING_CORE__INVERT_UP_DN = 0x06B5;
	public final static int RANGING_CORE__CPUMP_1 = 0x06B6;
	public final static int RANGING_CORE__CPUMP_2 = 0x06B7;
	public final static int RANGING_CORE__CPUMP_3 = 0x06B8;
	public final static int RANGING_CORE__OSC_1 = 0x06B9;
	public final static int RANGING_CORE__PLL_1 = 0x06BB;
	public final static int RANGING_CORE__PLL_2 = 0x06BC;
	public final static int RANGING_CORE__REFERENCE_1 = 0x06BD;
	public final static int RANGING_CORE__REFERENCE_3 = 0x06BF;
	public final static int RANGING_CORE__REFERENCE_4 = 0x06C0;
	public final static int RANGING_CORE__REFERENCE_5 = 0x06C1;
	public final static int RANGING_CORE__REGAVDD1V2 = 0x06C3;
	public final static int RANGING_CORE__CALIB_1 = 0x06C4;
	public final static int RANGING_CORE__CALIB_2 = 0x06C5;
	public final static int RANGING_CORE__CALIB_3 = 0x06C6;
	public final static int RANGING_CORE__TST_MUX_SEL1 = 0x06C9;
	public final static int RANGING_CORE__TST_MUX_SEL2 = 0x06CA;
	public final static int RANGING_CORE__TST_MUX = 0x06CB;
	public final static int RANGING_CORE__GPIO_OUT_TESTMUX = 0x06CC;
	public final static int RANGING_CORE__CUSTOM_FE = 0x06CD;
	public final static int RANGING_CORE__CUSTOM_FE_2 = 0x06CE;
	public final static int RANGING_CORE__SPAD_READOUT = 0x06CF;
	public final static int RANGING_CORE__SPAD_READOUT_1 = 0x06D0;
	public final static int RANGING_CORE__SPAD_READOUT_2 = 0x06D1;
	public final static int RANGING_CORE__SPAD_PS = 0x06D2;
	public final static int RANGING_CORE__LASER_SAFETY_2 = 0x06D4;
	public final static int RANGING_CORE__NVM_CTRL__MODE = 0x0780;
	public final static int RANGING_CORE__NVM_CTRL__PDN = 0x0781;
	public final static int RANGING_CORE__NVM_CTRL__PROGN = 0x0782;
	public final static int RANGING_CORE__NVM_CTRL__READN = 0x0783;
	public final static int RANGING_CORE__NVM_CTRL__PULSE_WIDTH_MSB = 0x0784;
	public final static int RANGING_CORE__NVM_CTRL__PULSE_WIDTH_LSB = 0x0785;
	public final static int RANGING_CORE__NVM_CTRL__HV_RISE_MSB = 0x0786;
	public final static int RANGING_CORE__NVM_CTRL__HV_RISE_LSB = 0x0787;
	public final static int RANGING_CORE__NVM_CTRL__HV_FALL_MSB = 0x0788;
	public final static int RANGING_CORE__NVM_CTRL__HV_FALL_LSB = 0x0789;
	public final static int RANGING_CORE__NVM_CTRL__TST = 0x078A;
	public final static int RANGING_CORE__NVM_CTRL__TESTREAD = 0x078B;
	public final static int RANGING_CORE__NVM_CTRL__DATAIN_MMM = 0x078C;
	public final static int RANGING_CORE__NVM_CTRL__DATAIN_LMM = 0x078D;
	public final static int RANGING_CORE__NVM_CTRL__DATAIN_LLM = 0x078E;
	public final static int RANGING_CORE__NVM_CTRL__DATAIN_LLL = 0x078F;
	public final static int RANGING_CORE__NVM_CTRL__DATAOUT_MMM = 0x0790;
	public final static int RANGING_CORE__NVM_CTRL__DATAOUT_LMM = 0x0791;
	public final static int RANGING_CORE__NVM_CTRL__DATAOUT_LLM = 0x0792;
	public final static int RANGING_CORE__NVM_CTRL__DATAOUT_LLL = 0x0793;
	public final static int RANGING_CORE__NVM_CTRL__ADDR = 0x0794;
	public final static int RANGING_CORE__NVM_CTRL__DATAOUT_ECC = 0x0795;
	public final static int RANGING_CORE__RET_SPAD_EN_0 = 0x0796;
	public final static int RANGING_CORE__RET_SPAD_EN_1 = 0x0797;
	public final static int RANGING_CORE__RET_SPAD_EN_2 = 0x0798;
	public final static int RANGING_CORE__RET_SPAD_EN_3 = 0x0799;
	public final static int RANGING_CORE__RET_SPAD_EN_4 = 0x079A;
	public final static int RANGING_CORE__RET_SPAD_EN_5 = 0x079B;
	public final static int RANGING_CORE__RET_SPAD_EN_6 = 0x079C;
	public final static int RANGING_CORE__RET_SPAD_EN_7 = 0x079D;
	public final static int RANGING_CORE__RET_SPAD_EN_8 = 0x079E;
	public final static int RANGING_CORE__RET_SPAD_EN_9 = 0x079F;
	public final static int RANGING_CORE__RET_SPAD_EN_10 = 0x07A0;
	public final static int RANGING_CORE__RET_SPAD_EN_11 = 0x07A1;
	public final static int RANGING_CORE__RET_SPAD_EN_12 = 0x07A2;
	public final static int RANGING_CORE__RET_SPAD_EN_13 = 0x07A3;
	public final static int RANGING_CORE__RET_SPAD_EN_14 = 0x07A4;
	public final static int RANGING_CORE__RET_SPAD_EN_15 = 0x07A5;
	public final static int RANGING_CORE__RET_SPAD_EN_16 = 0x07A6;
	public final static int RANGING_CORE__RET_SPAD_EN_17 = 0x07A7;
	public final static int RANGING_CORE__SPAD_SHIFT_EN = 0x07BA;
	public final static int RANGING_CORE__SPAD_DISABLE_CTRL = 0x07BB;
	public final static int RANGING_CORE__SPAD_EN_SHIFT_OUT_DEBUG = 0x07BC;
	public final static int RANGING_CORE__SPI_MODE = 0x07BD;
	public final static int RANGING_CORE__GPIO_DIR = 0x07BE;
	public final static int RANGING_CORE__VCSEL_PERIOD = 0x0880;
	public final static int RANGING_CORE__VCSEL_START = 0x0881;
	public final static int RANGING_CORE__VCSEL_STOP = 0x0882;
	public final static int RANGING_CORE__VCSEL_1 = 0x0885;
	public final static int RANGING_CORE__VCSEL_STATUS = 0x088D;
	public final static int RANGING_CORE__STATUS = 0x0980;
	public final static int RANGING_CORE__LASER_CONTINUITY_STATE = 0x0981;
	public final static int RANGING_CORE__RANGE_1_MMM = 0x0982;
	public final static int RANGING_CORE__RANGE_1_LMM = 0x0983;
	public final static int RANGING_CORE__RANGE_1_LLM = 0x0984;
	public final static int RANGING_CORE__RANGE_1_LLL = 0x0985;
	public final static int RANGING_CORE__RANGE_REF_1_MMM = 0x0986;
	public final static int RANGING_CORE__RANGE_REF_1_LMM = 0x0987;
	public final static int RANGING_CORE__RANGE_REF_1_LLM = 0x0988;
	public final static int RANGING_CORE__RANGE_REF_1_LLL = 0x0989;
	public final static int RANGING_CORE__AMBIENT_WINDOW_EVENTS_1_MMM = 0x098A;
	public final static int RANGING_CORE__AMBIENT_WINDOW_EVENTS_1_LMM = 0x098B;
	public final static int RANGING_CORE__AMBIENT_WINDOW_EVENTS_1_LLM = 0x098C;
	public final static int RANGING_CORE__AMBIENT_WINDOW_EVENTS_1_LLL = 0x098D;
	public final static int RANGING_CORE__RANGING_TOTAL_EVENTS_1_MMM = 0x098E;
	public final static int RANGING_CORE__RANGING_TOTAL_EVENTS_1_LMM = 0x098F;
	public final static int RANGING_CORE__RANGING_TOTAL_EVENTS_1_LLM = 0x0990;
	public final static int RANGING_CORE__RANGING_TOTAL_EVENTS_1_LLL = 0x0991;
	public final static int RANGING_CORE__SIGNAL_TOTAL_EVENTS_1_MMM = 0x0992;
	public final static int RANGING_CORE__SIGNAL_TOTAL_EVENTS_1_LMM = 0x0993;
	public final static int RANGING_CORE__SIGNAL_TOTAL_EVENTS_1_LLM = 0x0994;
	public final static int RANGING_CORE__SIGNAL_TOTAL_EVENTS_1_LLL = 0x0995;
	public final static int RANGING_CORE__TOTAL_PERIODS_ELAPSED_1_MM = 0x0996;
	public final static int RANGING_CORE__TOTAL_PERIODS_ELAPSED_1_LM = 0x0997;
	public final static int RANGING_CORE__TOTAL_PERIODS_ELAPSED_1_LL = 0x0998;
	public final static int RANGING_CORE__AMBIENT_MISMATCH_MM = 0x0999;
	public final static int RANGING_CORE__AMBIENT_MISMATCH_LM = 0x099A;
	public final static int RANGING_CORE__AMBIENT_MISMATCH_LL = 0x099B;
	public final static int RANGING_CORE__AMBIENT_WINDOW_EVENTS_REF_1_MMM = 0x099C;
	public final static int RANGING_CORE__AMBIENT_WINDOW_EVENTS_REF_1_LMM = 0x099D;
	public final static int RANGING_CORE__AMBIENT_WINDOW_EVENTS_REF_1_LLM = 0x099E;
	public final static int RANGING_CORE__AMBIENT_WINDOW_EVENTS_REF_1_LLL = 0x099F;
	public final static int RANGING_CORE__RANGING_TOTAL_EVENTS_REF_1_MMM = 0x09A0;
	public final static int RANGING_CORE__RANGING_TOTAL_EVENTS_REF_1_LMM = 0x09A1;
	public final static int RANGING_CORE__RANGING_TOTAL_EVENTS_REF_1_LLM = 0x09A2;
	public final static int RANGING_CORE__RANGING_TOTAL_EVENTS_REF_1_LLL = 0x09A3;
	public final static int RANGING_CORE__SIGNAL_TOTAL_EVENTS_REF_1_MMM = 0x09A4;
	public final static int RANGING_CORE__SIGNAL_TOTAL_EVENTS_REF_1_LMM = 0x09A5;
	public final static int RANGING_CORE__SIGNAL_TOTAL_EVENTS_REF_1_LLM = 0x09A6;
	public final static int RANGING_CORE__SIGNAL_TOTAL_EVENTS_REF_1_LLL = 0x09A7;
	public final static int RANGING_CORE__TOTAL_PERIODS_ELAPSED_REF_1_MM = 0x09A8;
	public final static int RANGING_CORE__TOTAL_PERIODS_ELAPSED_REF_1_LM = 0x09A9;
	public final static int RANGING_CORE__TOTAL_PERIODS_ELAPSED_REF_1_LL = 0x09AA;
	public final static int RANGING_CORE__AMBIENT_MISMATCH_REF_MM = 0x09AB;
	public final static int RANGING_CORE__AMBIENT_MISMATCH_REF_LM = 0x09AC;
	public final static int RANGING_CORE__AMBIENT_MISMATCH_REF_LL = 0x09AD;
	public final static int RANGING_CORE__GPIO_CONFIG__A0 = 0x0A00;
	public final static int RANGING_CORE__RESET_CONTROL__A0 = 0x0A01;
	public final static int RANGING_CORE__INTR_MANAGER__A0 = 0x0A02;
	public final static int RANGING_CORE__POWER_FSM_TIME_OSC__A0 = 0x0A06;
	public final static int RANGING_CORE__VCSEL_ATEST__A0 = 0x0A07;
	public final static int RANGING_CORE__VCSEL_PERIOD_CLIPPED__A0 = 0x0A08;
	public final static int RANGING_CORE__VCSEL_STOP_CLIPPED__A0 = 0x0A09;
	public final static int RANGING_CORE__CALIB_2__A0 = 0x0A0A;
	public final static int RANGING_CORE__STOP_CONDITION__A0 = 0x0A0B;
	public final static int RANGING_CORE__STATUS_RESET__A0 = 0x0A0C;
	public final static int RANGING_CORE__READOUT_CFG__A0 = 0x0A0D;
	public final static int RANGING_CORE__WINDOW_SETTING__A0 = 0x0A0E;
	public final static int RANGING_CORE__VCSEL_DELAY__A0 = 0x0A1A;
	public final static int RANGING_CORE__REFERENCE_2__A0 = 0x0A1B;
	public final static int RANGING_CORE__REGAVDD1V2__A0 = 0x0A1D;
	public final static int RANGING_CORE__TST_MUX__A0 = 0x0A1F;
	public final static int RANGING_CORE__CUSTOM_FE_2__A0 = 0x0A20;
	public final static int RANGING_CORE__SPAD_READOUT__A0 = 0x0A21;
	public final static int RANGING_CORE__CPUMP_1__A0 = 0x0A22;
	public final static int RANGING_CORE__SPARE_REGISTER__A0 = 0x0A23;
	public final static int RANGING_CORE__VCSEL_CONT_STAGE5_BYPASS__A0 = 0x0A24;
	public final static int RANGING_CORE__RET_SPAD_EN_18 = 0x0A25;
	public final static int RANGING_CORE__RET_SPAD_EN_19 = 0x0A26;
	public final static int RANGING_CORE__RET_SPAD_EN_20 = 0x0A27;
	public final static int RANGING_CORE__RET_SPAD_EN_21 = 0x0A28;
	public final static int RANGING_CORE__RET_SPAD_EN_22 = 0x0A29;
	public final static int RANGING_CORE__RET_SPAD_EN_23 = 0x0A2A;
	public final static int RANGING_CORE__RET_SPAD_EN_24 = 0x0A2B;
	public final static int RANGING_CORE__RET_SPAD_EN_25 = 0x0A2C;
	public final static int RANGING_CORE__RET_SPAD_EN_26 = 0x0A2D;
	public final static int RANGING_CORE__RET_SPAD_EN_27 = 0x0A2E;
	public final static int RANGING_CORE__RET_SPAD_EN_28 = 0x0A2F;
	public final static int RANGING_CORE__RET_SPAD_EN_29 = 0x0A30;
	public final static int RANGING_CORE__RET_SPAD_EN_30 = 0x0A31;
	public final static int RANGING_CORE__RET_SPAD_EN_31 = 0x0A32;
	public final static int RANGING_CORE__REF_SPAD_EN_0__EWOK = 0x0A33;
	public final static int RANGING_CORE__REF_SPAD_EN_1__EWOK = 0x0A34;
	public final static int RANGING_CORE__REF_SPAD_EN_2__EWOK = 0x0A35;
	public final static int RANGING_CORE__REF_SPAD_EN_3__EWOK = 0x0A36;
	public final static int RANGING_CORE__REF_SPAD_EN_4__EWOK = 0x0A37;
	public final static int RANGING_CORE__REF_SPAD_EN_5__EWOK = 0x0A38;
	public final static int RANGING_CORE__REF_EN_START_SELECT = 0x0A39;
	public final static int RANGING_CORE__REGDVDD1V2_ATEST__EWOK = 0x0A41;
	public final static int SOFT_RESET_GO1 = 0x0B00;
	public final static int PRIVATE__PATCH_BASE_ADDR_RSLV = 0x0E00;
	public final static int PREV_SHADOW_RESULT__INTERRUPT_STATUS = 0x0ED0;
	public final static int PREV_SHADOW_RESULT__RANGE_STATUS = 0x0ED1;
	public final static int PREV_SHADOW_RESULT__REPORT_STATUS = 0x0ED2;
	public final static int PREV_SHADOW_RESULT__STREAM_COUNT = 0x0ED3;
	public final static int PREV_SHADOW_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0 = 0x0ED4;
	public final static int PREV_SHADOW_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0_HI = 0x0ED4;
	public final static int PREV_SHADOW_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0_LO = 0x0ED5;
	public final static int PREV_SHADOW_RESULT__PEAK_SIGNAL_COUNT_RATE_MCPS_SD0 = 0x0ED6;
	public final static int PREV_SHADOW_RESULT__PEAK_SIGNAL_COUNT_RATE_MCPS_SD0_HI = 0x0ED6;
	public final static int PREV_SHADOW_RESULT__PEAK_SIGNAL_COUNT_RATE_MCPS_SD0_LO = 0x0ED7;
	public final static int PREV_SHADOW_RESULT__AMBIENT_COUNT_RATE_MCPS_SD0 = 0x0ED8;
	public final static int PREV_SHADOW_RESULT__AMBIENT_COUNT_RATE_MCPS_SD0_HI = 0x0ED8;
	public final static int PREV_SHADOW_RESULT__AMBIENT_COUNT_RATE_MCPS_SD0_LO = 0x0ED9;
	public final static int PREV_SHADOW_RESULT__SIGMA_SD0 = 0x0EDA;
	public final static int PREV_SHADOW_RESULT__SIGMA_SD0_HI = 0x0EDA;
	public final static int PREV_SHADOW_RESULT__SIGMA_SD0_LO = 0x0EDB;
	public final static int PREV_SHADOW_RESULT__PHASE_SD0 = 0x0EDC;
	public final static int PREV_SHADOW_RESULT__PHASE_SD0_HI = 0x0EDC;
	public final static int PREV_SHADOW_RESULT__PHASE_SD0_LO = 0x0EDD;
	public final static int PREV_SHADOW_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0 = 0x0EDE;
	public final static int PREV_SHADOW_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0_HI = 0x0EDE;
	public final static int PREV_SHADOW_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0_LO = 0x0EDF;
	public final static int PREV_SHADOW_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0 = 0x0EE0;
	public final static int PREV_SHADOW_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0_HI = 0x0EE0;
	public final static int PREV_SHADOW_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0_LO = 0x0EE1;
	public final static int PREV_SHADOW_RESULT__MM_INNER_ACTUAL_EFFECTIVE_SPADS_SD0 = 0x0EE2;
	public final static int PREV_SHADOW_RESULT__MM_INNER_ACTUAL_EFFECTIVE_SPADS_SD0_HI = 0x0EE2;
	public final static int PREV_SHADOW_RESULT__MM_INNER_ACTUAL_EFFECTIVE_SPADS_SD0_LO = 0x0EE3;
	public final static int PREV_SHADOW_RESULT__MM_OUTER_ACTUAL_EFFECTIVE_SPADS_SD0 = 0x0EE4;
	public final static int PREV_SHADOW_RESULT__MM_OUTER_ACTUAL_EFFECTIVE_SPADS_SD0_HI = 0x0EE4;
	public final static int PREV_SHADOW_RESULT__MM_OUTER_ACTUAL_EFFECTIVE_SPADS_SD0_LO = 0x0EE5;
	public final static int PREV_SHADOW_RESULT__AVG_SIGNAL_COUNT_RATE_MCPS_SD0 = 0x0EE6;
	public final static int PREV_SHADOW_RESULT__AVG_SIGNAL_COUNT_RATE_MCPS_SD0_HI = 0x0EE6;
	public final static int PREV_SHADOW_RESULT__AVG_SIGNAL_COUNT_RATE_MCPS_SD0_LO = 0x0EE7;
	public final static int PREV_SHADOW_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD1 = 0x0EE8;
	public final static int PREV_SHADOW_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD1_HI = 0x0EE8;
	public final static int PREV_SHADOW_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD1_LO = 0x0EE9;
	public final static int PREV_SHADOW_RESULT__PEAK_SIGNAL_COUNT_RATE_MCPS_SD1 = 0x0EEA;
	public final static int PREV_SHADOW_RESULT__PEAK_SIGNAL_COUNT_RATE_MCPS_SD1_HI = 0x0EEA;
	public final static int PREV_SHADOW_RESULT__PEAK_SIGNAL_COUNT_RATE_MCPS_SD1_LO = 0x0EEB;
	public final static int PREV_SHADOW_RESULT__AMBIENT_COUNT_RATE_MCPS_SD1 = 0x0EEC;
	public final static int PREV_SHADOW_RESULT__AMBIENT_COUNT_RATE_MCPS_SD1_HI = 0x0EEC;
	public final static int PREV_SHADOW_RESULT__AMBIENT_COUNT_RATE_MCPS_SD1_LO = 0x0EED;
	public final static int PREV_SHADOW_RESULT__SIGMA_SD1 = 0x0EEE;
	public final static int PREV_SHADOW_RESULT__SIGMA_SD1_HI = 0x0EEE;
	public final static int PREV_SHADOW_RESULT__SIGMA_SD1_LO = 0x0EEF;
	public final static int PREV_SHADOW_RESULT__PHASE_SD1 = 0x0EF0;
	public final static int PREV_SHADOW_RESULT__PHASE_SD1_HI = 0x0EF0;
	public final static int PREV_SHADOW_RESULT__PHASE_SD1_LO = 0x0EF1;
	public final static int PREV_SHADOW_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD1 = 0x0EF2;
	public final static int PREV_SHADOW_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD1_HI = 0x0EF2;
	public final static int PREV_SHADOW_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD1_LO = 0x0EF3;
	public final static int PREV_SHADOW_RESULT__SPARE_0_SD1 = 0x0EF4;
	public final static int PREV_SHADOW_RESULT__SPARE_0_SD1_HI = 0x0EF4;
	public final static int PREV_SHADOW_RESULT__SPARE_0_SD1_LO = 0x0EF5;
	public final static int PREV_SHADOW_RESULT__SPARE_1_SD1 = 0x0EF6;
	public final static int PREV_SHADOW_RESULT__SPARE_1_SD1_HI = 0x0EF6;
	public final static int PREV_SHADOW_RESULT__SPARE_1_SD1_LO = 0x0EF7;
	public final static int PREV_SHADOW_RESULT__SPARE_2_SD1 = 0x0EF8;
	public final static int PREV_SHADOW_RESULT__SPARE_2_SD1_HI = 0x0EF8;
	public final static int PREV_SHADOW_RESULT__SPARE_2_SD1_LO = 0x0EF9;
	public final static int PREV_SHADOW_RESULT__SPARE_3_SD1 = 0x0EFA;
	public final static int PREV_SHADOW_RESULT__SPARE_3_SD1_HI = 0x0EFA;
	public final static int PREV_SHADOW_RESULT__SPARE_3_SD1_LO = 0x0EFB;
	public final static int PREV_SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD0 = 0x0EFC;
	public final static int PREV_SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD0_3 = 0x0EFC;
	public final static int PREV_SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD0_2 = 0x0EFD;
	public final static int PREV_SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD0_1 = 0x0EFE;
	public final static int PREV_SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD0_0 = 0x0EFF;
	public final static int PREV_SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD0 = 0x0F00;
	public final static int PREV_SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD0_3 = 0x0F00;
	public final static int PREV_SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD0_2 = 0x0F01;
	public final static int PREV_SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD0_1 = 0x0F02;
	public final static int PREV_SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD0_0 = 0x0F03;
	public final static int PREV_SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD0 = 0x0F04;
	public final static int PREV_SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD0_3 = 0x0F04;
	public final static int PREV_SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD0_2 = 0x0F05;
	public final static int PREV_SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD0_1 = 0x0F06;
	public final static int PREV_SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD0_0 = 0x0F07;
	public final static int PREV_SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD0 = 0x0F08;
	public final static int PREV_SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD0_3 = 0x0F08;
	public final static int PREV_SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD0_2 = 0x0F09;
	public final static int PREV_SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD0_1 = 0x0F0A;
	public final static int PREV_SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD0_0 = 0x0F0B;
	public final static int PREV_SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD1 = 0x0F0C;
	public final static int PREV_SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD1_3 = 0x0F0C;
	public final static int PREV_SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD1_2 = 0x0F0D;
	public final static int PREV_SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD1_1 = 0x0F0E;
	public final static int PREV_SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD1_0 = 0x0F0F;
	public final static int PREV_SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD1 = 0x0F10;
	public final static int PREV_SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD1_3 = 0x0F10;
	public final static int PREV_SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD1_2 = 0x0F11;
	public final static int PREV_SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD1_1 = 0x0F12;
	public final static int PREV_SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD1_0 = 0x0F13;
	public final static int PREV_SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD1 = 0x0F14;
	public final static int PREV_SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD1_3 = 0x0F14;
	public final static int PREV_SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD1_2 = 0x0F15;
	public final static int PREV_SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD1_1 = 0x0F16;
	public final static int PREV_SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD1_0 = 0x0F17;
	public final static int PREV_SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD1 = 0x0F18;
	public final static int PREV_SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD1_3 = 0x0F18;
	public final static int PREV_SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD1_2 = 0x0F19;
	public final static int PREV_SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD1_1 = 0x0F1A;
	public final static int PREV_SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD1_0 = 0x0F1B;
	public final static int PREV_SHADOW_RESULT_CORE__SPARE_0 = 0x0F1C;
	public final static int RESULT__DEBUG_STATUS = 0x0F20;
	public final static int RESULT__DEBUG_STAGE = 0x0F21;
	public final static int GPH__SYSTEM__THRESH_RATE_HIGH = 0x0F24;
	public final static int GPH__SYSTEM__THRESH_RATE_HIGH_HI = 0x0F24;
	public final static int GPH__SYSTEM__THRESH_RATE_HIGH_LO = 0x0F25;
	public final static int GPH__SYSTEM__THRESH_RATE_LOW = 0x0F26;
	public final static int GPH__SYSTEM__THRESH_RATE_LOW_HI = 0x0F26;
	public final static int GPH__SYSTEM__THRESH_RATE_LOW_LO = 0x0F27;
	public final static int GPH__SYSTEM__INTERRUPT_CONFIG_GPIO = 0x0F28;
	public final static int GPH__DSS_CONFIG__ROI_MODE_CONTROL = 0x0F2F;
	public final static int GPH__DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT = 0x0F30;
	public final static int GPH__DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT_HI = 0x0F30;
	public final static int GPH__DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT_LO = 0x0F31;
	public final static int GPH__DSS_CONFIG__MANUAL_BLOCK_SELECT = 0x0F32;
	public final static int GPH__DSS_CONFIG__MAX_SPADS_LIMIT = 0x0F33;
	public final static int GPH__DSS_CONFIG__MIN_SPADS_LIMIT = 0x0F34;
	public final static int GPH__MM_CONFIG__TIMEOUT_MACROP_A_HI = 0x0F36;
	public final static int GPH__MM_CONFIG__TIMEOUT_MACROP_A_LO = 0x0F37;
	public final static int GPH__MM_CONFIG__TIMEOUT_MACROP_B_HI = 0x0F38;
	public final static int GPH__MM_CONFIG__TIMEOUT_MACROP_B_LO = 0x0F39;
	public final static int GPH__RANGE_CONFIG__TIMEOUT_MACROP_A_HI = 0x0F3A;
	public final static int GPH__RANGE_CONFIG__TIMEOUT_MACROP_A_LO = 0x0F3B;
	public final static int GPH__RANGE_CONFIG__VCSEL_PERIOD_A = 0x0F3C;
	public final static int GPH__RANGE_CONFIG__VCSEL_PERIOD_B = 0x0F3D;
	public final static int GPH__RANGE_CONFIG__TIMEOUT_MACROP_B_HI = 0x0F3E;
	public final static int GPH__RANGE_CONFIG__TIMEOUT_MACROP_B_LO = 0x0F3F;
	public final static int GPH__RANGE_CONFIG__SIGMA_THRESH = 0x0F40;
	public final static int GPH__RANGE_CONFIG__SIGMA_THRESH_HI = 0x0F40;
	public final static int GPH__RANGE_CONFIG__SIGMA_THRESH_LO = 0x0F41;
	public final static int GPH__RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS = 0x0F42;
	public final static int GPH__RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS_HI = 0x0F42;
	public final static int GPH__RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS_LO = 0x0F43;
	public final static int GPH__RANGE_CONFIG__VALID_PHASE_LOW = 0x0F44;
	public final static int GPH__RANGE_CONFIG__VALID_PHASE_HIGH = 0x0F45;
	public final static int FIRMWARE__INTERNAL_STREAM_COUNT_DIV = 0x0F46;
	public final static int FIRMWARE__INTERNAL_STREAM_COUNTER_VAL = 0x0F47;
	public final static int DSS_CALC__ROI_CTRL = 0x0F54;
	public final static int DSS_CALC__SPARE_1 = 0x0F55;
	public final static int DSS_CALC__SPARE_2 = 0x0F56;
	public final static int DSS_CALC__SPARE_3 = 0x0F57;
	public final static int DSS_CALC__SPARE_4 = 0x0F58;
	public final static int DSS_CALC__SPARE_5 = 0x0F59;
	public final static int DSS_CALC__SPARE_6 = 0x0F5A;
	public final static int DSS_CALC__SPARE_7 = 0x0F5B;
	public final static int DSS_CALC__USER_ROI_SPAD_EN_0 = 0x0F5C;
	public final static int DSS_CALC__USER_ROI_SPAD_EN_1 = 0x0F5D;
	public final static int DSS_CALC__USER_ROI_SPAD_EN_2 = 0x0F5E;
	public final static int DSS_CALC__USER_ROI_SPAD_EN_3 = 0x0F5F;
	public final static int DSS_CALC__USER_ROI_SPAD_EN_4 = 0x0F60;
	public final static int DSS_CALC__USER_ROI_SPAD_EN_5 = 0x0F61;
	public final static int DSS_CALC__USER_ROI_SPAD_EN_6 = 0x0F62;
	public final static int DSS_CALC__USER_ROI_SPAD_EN_7 = 0x0F63;
	public final static int DSS_CALC__USER_ROI_SPAD_EN_8 = 0x0F64;
	public final static int DSS_CALC__USER_ROI_SPAD_EN_9 = 0x0F65;
	public final static int DSS_CALC__USER_ROI_SPAD_EN_10 = 0x0F66;
	public final static int DSS_CALC__USER_ROI_SPAD_EN_11 = 0x0F67;
	public final static int DSS_CALC__USER_ROI_SPAD_EN_12 = 0x0F68;
	public final static int DSS_CALC__USER_ROI_SPAD_EN_13 = 0x0F69;
	public final static int DSS_CALC__USER_ROI_SPAD_EN_14 = 0x0F6A;
	public final static int DSS_CALC__USER_ROI_SPAD_EN_15 = 0x0F6B;
	public final static int DSS_CALC__USER_ROI_SPAD_EN_16 = 0x0F6C;
	public final static int DSS_CALC__USER_ROI_SPAD_EN_17 = 0x0F6D;
	public final static int DSS_CALC__USER_ROI_SPAD_EN_18 = 0x0F6E;
	public final static int DSS_CALC__USER_ROI_SPAD_EN_19 = 0x0F6F;
	public final static int DSS_CALC__USER_ROI_SPAD_EN_20 = 0x0F70;
	public final static int DSS_CALC__USER_ROI_SPAD_EN_21 = 0x0F71;
	public final static int DSS_CALC__USER_ROI_SPAD_EN_22 = 0x0F72;
	public final static int DSS_CALC__USER_ROI_SPAD_EN_23 = 0x0F73;
	public final static int DSS_CALC__USER_ROI_SPAD_EN_24 = 0x0F74;
	public final static int DSS_CALC__USER_ROI_SPAD_EN_25 = 0x0F75;
	public final static int DSS_CALC__USER_ROI_SPAD_EN_26 = 0x0F76;
	public final static int DSS_CALC__USER_ROI_SPAD_EN_27 = 0x0F77;
	public final static int DSS_CALC__USER_ROI_SPAD_EN_28 = 0x0F78;
	public final static int DSS_CALC__USER_ROI_SPAD_EN_29 = 0x0F79;
	public final static int DSS_CALC__USER_ROI_SPAD_EN_30 = 0x0F7A;
	public final static int DSS_CALC__USER_ROI_SPAD_EN_31 = 0x0F7B;
	public final static int DSS_CALC__USER_ROI_0 = 0x0F7C;
	public final static int DSS_CALC__USER_ROI_1 = 0x0F7D;
	public final static int DSS_CALC__MODE_ROI_0 = 0x0F7E;
	public final static int DSS_CALC__MODE_ROI_1 = 0x0F7F;
	public final static int SIGMA_ESTIMATOR_CALC__SPARE_0 = 0x0F80;
	public final static int VHV_RESULT__PEAK_SIGNAL_RATE_MCPS = 0x0F82;
	public final static int VHV_RESULT__PEAK_SIGNAL_RATE_MCPS_HI = 0x0F82;
	public final static int VHV_RESULT__PEAK_SIGNAL_RATE_MCPS_LO = 0x0F83;
	public final static int VHV_RESULT__SIGNAL_TOTAL_EVENTS_REF = 0x0F84;
	public final static int VHV_RESULT__SIGNAL_TOTAL_EVENTS_REF_3 = 0x0F84;
	public final static int VHV_RESULT__SIGNAL_TOTAL_EVENTS_REF_2 = 0x0F85;
	public final static int VHV_RESULT__SIGNAL_TOTAL_EVENTS_REF_1 = 0x0F86;
	public final static int VHV_RESULT__SIGNAL_TOTAL_EVENTS_REF_0 = 0x0F87;
	public final static int PHASECAL_RESULT__PHASE_OUTPUT_REF = 0x0F88;
	public final static int PHASECAL_RESULT__PHASE_OUTPUT_REF_HI = 0x0F88;
	public final static int PHASECAL_RESULT__PHASE_OUTPUT_REF_LO = 0x0F89;
	public final static int DSS_RESULT__TOTAL_RATE_PER_SPAD = 0x0F8A;
	public final static int DSS_RESULT__TOTAL_RATE_PER_SPAD_HI = 0x0F8A;
	public final static int DSS_RESULT__TOTAL_RATE_PER_SPAD_LO = 0x0F8B;
	public final static int DSS_RESULT__ENABLED_BLOCKS = 0x0F8C;
	public final static int DSS_RESULT__NUM_REQUESTED_SPADS = 0x0F8E;
	public final static int DSS_RESULT__NUM_REQUESTED_SPADS_HI = 0x0F8E;
	public final static int DSS_RESULT__NUM_REQUESTED_SPADS_LO = 0x0F8F;
	public final static int MM_RESULT__INNER_INTERSECTION_RATE = 0x0F92;
	public final static int MM_RESULT__INNER_INTERSECTION_RATE_HI = 0x0F92;
	public final static int MM_RESULT__INNER_INTERSECTION_RATE_LO = 0x0F93;
	public final static int MM_RESULT__OUTER_COMPLEMENT_RATE = 0x0F94;
	public final static int MM_RESULT__OUTER_COMPLEMENT_RATE_HI = 0x0F94;
	public final static int MM_RESULT__OUTER_COMPLEMENT_RATE_LO = 0x0F95;
	public final static int MM_RESULT__TOTAL_OFFSET = 0x0F96;
	public final static int MM_RESULT__TOTAL_OFFSET_HI = 0x0F96;
	public final static int MM_RESULT__TOTAL_OFFSET_LO = 0x0F97;
	public final static int XTALK_CALC__XTALK_FOR_ENABLED_SPADS = 0x0F98;
	public final static int XTALK_CALC__XTALK_FOR_ENABLED_SPADS_3 = 0x0F98;
	public final static int XTALK_CALC__XTALK_FOR_ENABLED_SPADS_2 = 0x0F99;
	public final static int XTALK_CALC__XTALK_FOR_ENABLED_SPADS_1 = 0x0F9A;
	public final static int XTALK_CALC__XTALK_FOR_ENABLED_SPADS_0 = 0x0F9B;
	public final static int XTALK_RESULT__AVG_XTALK_USER_ROI_KCPS = 0x0F9C;
	public final static int XTALK_RESULT__AVG_XTALK_USER_ROI_KCPS_3 = 0x0F9C;
	public final static int XTALK_RESULT__AVG_XTALK_USER_ROI_KCPS_2 = 0x0F9D;
	public final static int XTALK_RESULT__AVG_XTALK_USER_ROI_KCPS_1 = 0x0F9E;
	public final static int XTALK_RESULT__AVG_XTALK_USER_ROI_KCPS_0 = 0x0F9F;
	public final static int XTALK_RESULT__AVG_XTALK_MM_INNER_ROI_KCPS = 0x0FA0;
	public final static int XTALK_RESULT__AVG_XTALK_MM_INNER_ROI_KCPS_3 = 0x0FA0;
	public final static int XTALK_RESULT__AVG_XTALK_MM_INNER_ROI_KCPS_2 = 0x0FA1;
	public final static int XTALK_RESULT__AVG_XTALK_MM_INNER_ROI_KCPS_1 = 0x0FA2;
	public final static int XTALK_RESULT__AVG_XTALK_MM_INNER_ROI_KCPS_0 = 0x0FA3;
	public final static int XTALK_RESULT__AVG_XTALK_MM_OUTER_ROI_KCPS = 0x0FA4;
	public final static int XTALK_RESULT__AVG_XTALK_MM_OUTER_ROI_KCPS_3 = 0x0FA4;
	public final static int XTALK_RESULT__AVG_XTALK_MM_OUTER_ROI_KCPS_2 = 0x0FA5;
	public final static int XTALK_RESULT__AVG_XTALK_MM_OUTER_ROI_KCPS_1 = 0x0FA6;
	public final static int XTALK_RESULT__AVG_XTALK_MM_OUTER_ROI_KCPS_0 = 0x0FA7;
	public final static int RANGE_RESULT__ACCUM_PHASE = 0x0FA8;
	public final static int RANGE_RESULT__ACCUM_PHASE_3 = 0x0FA8;
	public final static int RANGE_RESULT__ACCUM_PHASE_2 = 0x0FA9;
	public final static int RANGE_RESULT__ACCUM_PHASE_1 = 0x0FAA;
	public final static int RANGE_RESULT__ACCUM_PHASE_0 = 0x0FAB;
	public final static int RANGE_RESULT__OFFSET_CORRECTED_RANGE = 0x0FAC;
	public final static int RANGE_RESULT__OFFSET_CORRECTED_RANGE_HI = 0x0FAC;
	public final static int RANGE_RESULT__OFFSET_CORRECTED_RANGE_LO = 0x0FAD;
	public final static int SHADOW_PHASECAL_RESULT__VCSEL_START = 0x0FAE;
	public final static int SHADOW_RESULT__INTERRUPT_STATUS = 0x0FB0;
	public final static int SHADOW_RESULT__RANGE_STATUS = 0x0FB1;
	public final static int SHADOW_RESULT__REPORT_STATUS = 0x0FB2;
	public final static int SHADOW_RESULT__STREAM_COUNT = 0x0FB3;
	public final static int SHADOW_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0 = 0x0FB4;
	public final static int SHADOW_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0_HI = 0x0FB4;
	public final static int SHADOW_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0_LO = 0x0FB5;
	public final static int SHADOW_RESULT__PEAK_SIGNAL_COUNT_RATE_MCPS_SD0 = 0x0FB6;
	public final static int SHADOW_RESULT__PEAK_SIGNAL_COUNT_RATE_MCPS_SD0_HI = 0x0FB6;
	public final static int SHADOW_RESULT__PEAK_SIGNAL_COUNT_RATE_MCPS_SD0_LO = 0x0FB7;
	public final static int SHADOW_RESULT__AMBIENT_COUNT_RATE_MCPS_SD0 = 0x0FB8;
	public final static int SHADOW_RESULT__AMBIENT_COUNT_RATE_MCPS_SD0_HI = 0x0FB8;
	public final static int SHADOW_RESULT__AMBIENT_COUNT_RATE_MCPS_SD0_LO = 0x0FB9;
	public final static int SHADOW_RESULT__SIGMA_SD0 = 0x0FBA;
	public final static int SHADOW_RESULT__SIGMA_SD0_HI = 0x0FBA;
	public final static int SHADOW_RESULT__SIGMA_SD0_LO = 0x0FBB;
	public final static int SHADOW_RESULT__PHASE_SD0 = 0x0FBC;
	public final static int SHADOW_RESULT__PHASE_SD0_HI = 0x0FBC;
	public final static int SHADOW_RESULT__PHASE_SD0_LO = 0x0FBD;
	public final static int SHADOW_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0 = 0x0FBE;
	public final static int SHADOW_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0_HI = 0x0FBE;
	public final static int SHADOW_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0_LO = 0x0FBF;
	public final static int SHADOW_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0 = 0x0FC0;
	public final static int SHADOW_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0_HI = 0x0FC0;
	public final static int SHADOW_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0_LO = 0x0FC1;
	public final static int SHADOW_RESULT__MM_INNER_ACTUAL_EFFECTIVE_SPADS_SD0 = 0x0FC2;
	public final static int SHADOW_RESULT__MM_INNER_ACTUAL_EFFECTIVE_SPADS_SD0_HI = 0x0FC2;
	public final static int SHADOW_RESULT__MM_INNER_ACTUAL_EFFECTIVE_SPADS_SD0_LO = 0x0FC3;
	public final static int SHADOW_RESULT__MM_OUTER_ACTUAL_EFFECTIVE_SPADS_SD0 = 0x0FC4;
	public final static int SHADOW_RESULT__MM_OUTER_ACTUAL_EFFECTIVE_SPADS_SD0_HI = 0x0FC4;
	public final static int SHADOW_RESULT__MM_OUTER_ACTUAL_EFFECTIVE_SPADS_SD0_LO = 0x0FC5;
	public final static int SHADOW_RESULT__AVG_SIGNAL_COUNT_RATE_MCPS_SD0 = 0x0FC6;
	public final static int SHADOW_RESULT__AVG_SIGNAL_COUNT_RATE_MCPS_SD0_HI = 0x0FC6;
	public final static int SHADOW_RESULT__AVG_SIGNAL_COUNT_RATE_MCPS_SD0_LO = 0x0FC7;
	public final static int SHADOW_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD1 = 0x0FC8;
	public final static int SHADOW_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD1_HI = 0x0FC8;
	public final static int SHADOW_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD1_LO = 0x0FC9;
	public final static int SHADOW_RESULT__PEAK_SIGNAL_COUNT_RATE_MCPS_SD1 = 0x0FCA;
	public final static int SHADOW_RESULT__PEAK_SIGNAL_COUNT_RATE_MCPS_SD1_HI = 0x0FCA;
	public final static int SHADOW_RESULT__PEAK_SIGNAL_COUNT_RATE_MCPS_SD1_LO = 0x0FCB;
	public final static int SHADOW_RESULT__AMBIENT_COUNT_RATE_MCPS_SD1 = 0x0FCC;
	public final static int SHADOW_RESULT__AMBIENT_COUNT_RATE_MCPS_SD1_HI = 0x0FCC;
	public final static int SHADOW_RESULT__AMBIENT_COUNT_RATE_MCPS_SD1_LO = 0x0FCD;
	public final static int SHADOW_RESULT__SIGMA_SD1 = 0x0FCE;
	public final static int SHADOW_RESULT__SIGMA_SD1_HI = 0x0FCE;
	public final static int SHADOW_RESULT__SIGMA_SD1_LO = 0x0FCF;
	public final static int SHADOW_RESULT__PHASE_SD1 = 0x0FD0;
	public final static int SHADOW_RESULT__PHASE_SD1_HI = 0x0FD0;
	public final static int SHADOW_RESULT__PHASE_SD1_LO = 0x0FD1;
	public final static int SHADOW_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD1 = 0x0FD2;
	public final static int SHADOW_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD1_HI = 0x0FD2;
	public final static int SHADOW_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD1_LO = 0x0FD3;
	public final static int SHADOW_RESULT__SPARE_0_SD1 = 0x0FD4;
	public final static int SHADOW_RESULT__SPARE_0_SD1_HI = 0x0FD4;
	public final static int SHADOW_RESULT__SPARE_0_SD1_LO = 0x0FD5;
	public final static int SHADOW_RESULT__SPARE_1_SD1 = 0x0FD6;
	public final static int SHADOW_RESULT__SPARE_1_SD1_HI = 0x0FD6;
	public final static int SHADOW_RESULT__SPARE_1_SD1_LO = 0x0FD7;
	public final static int SHADOW_RESULT__SPARE_2_SD1 = 0x0FD8;
	public final static int SHADOW_RESULT__SPARE_2_SD1_HI = 0x0FD8;
	public final static int SHADOW_RESULT__SPARE_2_SD1_LO = 0x0FD9;
	public final static int SHADOW_RESULT__SPARE_3_SD1 = 0x0FDA;
	public final static int SHADOW_RESULT__THRESH_INFO = 0x0FDB;
	public final static int SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD0 = 0x0FDC;
	public final static int SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD0_3 = 0x0FDC;
	public final static int SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD0_2 = 0x0FDD;
	public final static int SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD0_1 = 0x0FDE;
	public final static int SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD0_0 = 0x0FDF;
	public final static int SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD0 = 0x0FE0;
	public final static int SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD0_3 = 0x0FE0;
	public final static int SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD0_2 = 0x0FE1;
	public final static int SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD0_1 = 0x0FE2;
	public final static int SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD0_0 = 0x0FE3;
	public final static int SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD0 = 0x0FE4;
	public final static int SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD0_3 = 0x0FE4;
	public final static int SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD0_2 = 0x0FE5;
	public final static int SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD0_1 = 0x0FE6;
	public final static int SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD0_0 = 0x0FE7;
	public final static int SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD0 = 0x0FE8;
	public final static int SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD0_3 = 0x0FE8;
	public final static int SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD0_2 = 0x0FE9;
	public final static int SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD0_1 = 0x0FEA;
	public final static int SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD0_0 = 0x0FEB;
	public final static int SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD1 = 0x0FEC;
	public final static int SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD1_3 = 0x0FEC;
	public final static int SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD1_2 = 0x0FED;
	public final static int SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD1_1 = 0x0FEE;
	public final static int SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD1_0 = 0x0FEF;
	public final static int SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD1 = 0x0FF0;
	public final static int SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD1_3 = 0x0FF0;
	public final static int SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD1_2 = 0x0FF1;
	public final static int SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD1_1 = 0x0FF2;
	public final static int SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD1_0 = 0x0FF3;
	public final static int SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD1 = 0x0FF4;
	public final static int SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD1_3 = 0x0FF4;
	public final static int SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD1_2 = 0x0FF5;
	public final static int SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD1_1 = 0x0FF6;
	public final static int SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD1_0 = 0x0FF7;
	public final static int SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD1 = 0x0FF8;
	public final static int SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD1_3 = 0x0FF8;
	public final static int SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD1_2 = 0x0FF9;
	public final static int SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD1_1 = 0x0FFA;
	public final static int SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD1_0 = 0x0FFB;
	public final static int SHADOW_RESULT_CORE__SPARE_0 = 0x0FFC;
	public final static int SHADOW_PHASECAL_RESULT__REFERENCE_PHASE_HI = 0x0FFE;
	public final static int SHADOW_PHASECAL_RESULT__REFERENCE_PHASE_LO = 0x0FFF;

	enum DistanceMode {
		Short, Medium, Long, Unknown
	};

	enum RangeStatus {
		RangeValid(0),

		// "sigma estimator check is above the internal defined threshold"
		// (sigma = standard deviation of measurement)
		SigmaFail(1),

		// "signal value is below the internal defined threshold"
		SignalFail(2),

		// "Target is below minimum detection threshold."
		RangeValidMinRangeClipped(3),

		// "phase is out of bounds"
		// (nothing detected in range; try a longer distance mode if applicable)
		OutOfBoundsFail(4),

		// "HW or VCSEL failure"
		HardwareFail(5),

		// "The Range is valid but the wraparound check has not been done."
		RangeValidNoWrapCheckFail(6),

		// "Wrapped target, not matching phases"
		// "no matching phase in other VCSEL period timing."
		WrapTargetFail(7),

		// "Internal algo underflow or overflow in lite ranging."
		// ProcessingFail (8),: not used in API

		// "Specific to lite ranging."
		// should never occur with this lib (which uses low power auto ranging,
		// as the API does)
		XtalkSignalFail(9),

		// "1st interrupt when starting ranging in back to back mode. Ignore
		// data."
		// should never occur with this lib
		SynchronizationInt(10), // (the API spells this "syncronisation")

		// "All Range ok but object is result of multiple pulses merging together.
		// Used by RQL for merged pulse detection"
		// RangeValid MergedPulse (11), not used in API

		// "Used by RQL as different to phase fail."
		// TargetPresentLackOfSignal (12),:

		// "Target is below minimum detection threshold."
		MinRangeFail(13),

		// "The reported range is invalid"
		// RangeInvalid (14),: can't actually be returned by API (range can never become
		// negative, even after correction)

		// "No Update."
		None(255);

		private int action;

		public int getAction() {
			return this.action;
		}

		private RangeStatus(int action) {
			this.action = action;
		}
	};

	class RangingData {
		int range_mm;
		RangeStatus range_status;
		float peak_signal_count_rate_MCPS;
		float ambient_count_rate_MCPS;
	};

	RangingData ranging_data;

	int last_status; // status of last I2C transmission

	I2C i2cDevice;

	public VL53L1X(Port port) {
		address = AddressDefault;
		io_timeout = 0;
		did_timeout = false;
		calibrated = false;
		saved_vhv_init = 0;
		saved_vhv_timeout = 0;
		distance_mode = DistanceMode.Unknown;

		i2cDevice = new I2C(port, address);
	}

	// Record the current time to check an upcoming timeout against
	private void startTimeout() {
		timeout_start_ms = System.currentTimeMillis();
	}

	// Check if timeout is enabled (set to nonzero value) and has expired
	private boolean checkTimeoutExpired() {
		return (io_timeout > 0) && ((int) (System.currentTimeMillis() - timeout_start_ms) > io_timeout);
	}

	// Convert count rate from fixed point 9.7 format to float
	private float countRateFixedToFloat(int count_rate_fixed) {
		return (float) count_rate_fixed / (1 << 7);
	}

	public int readRangeContinuousMillimeters(boolean blocking) {
		return read(blocking);
	} // alias of read()

	public boolean dataReady() throws IOException {
		return (EndianIO.readReg(i2cDevice, GPIO__TIO_HV_STATUS, false) & 0x01) == 0;
	}

	/**
	 * @return the results
	 */
	public ResultBuffer getResults() {
		return results;
	}

	/**
	 * @param results the results to set
	 */
	public void setResults(ResultBuffer results) {
		this.results = results;
	}

	public boolean init(boolean io_2v8) throws IOException, InterruptedException {
		// check model ID and module type registers (values specified in datasheet)
		if (EndianIO.readReg16(i2cDevice, IDENTIFICATION__MODEL_ID, false) != 0xEACC) {
			return false;
		}

		// VL53L1_software_reset() begin

		EndianIO.writeReg(i2cDevice, SOFT_RESET, (byte) 0x00);
		Thread.sleep(100);
		EndianIO.writeReg(i2cDevice, SOFT_RESET, (byte) 0x01);

		// give it some time to boot; otherwise the sensor NACKs during the readReg()
		// call below and the Arduino 101 doesn't seem to handle that well
		Thread.sleep(1);

		// VL53L1_poll_for_boot_completion() begin

		startTimeout();

		// check last_status in case we still get a NACK to try to deal with it
		// correctly
		while ((EndianIO.readReg(i2cDevice, FIRMWARE__SYSTEM_STATUS, false) & 0x01) == 0 || last_status != 0) {
			if (checkTimeoutExpired()) {
				did_timeout = true;
				return false;
			}
		}
		// VL53L1_poll_for_boot_completion() end

		// VL53L1_software_reset() end

		// VL53L1_DataInit() begin

		// sensor uses 1V8 mode for I/O by default; switch to 2V8 mode if necessary
		if (io_2v8) {
			EndianIO.writeReg(i2cDevice, PAD_I2C_HV__EXTSUP_CONFIG,
					(byte) (EndianIO.readReg(i2cDevice, PAD_I2C_HV__EXTSUP_CONFIG, false) | 0x01));
		}

		// store oscillator info for later use
		fast_osc_frequency = EndianIO.readReg16(i2cDevice, OSC_MEASURED__FAST_OSC__FREQUENCY, false);
		osc_calibrate_val = EndianIO.readReg16(i2cDevice, RESULT__OSC_CALIBRATE_VAL, false);

		// VL53L1_DataInit() end

		// VL53L1_StaticInit() begin

		// Note that the API does not actually apply the configuration settings below
		// when VL53L1_StaticInit() is called: it keeps a copy of the sensor's
		// register contents in memory and doesn't actually write them until a
		// measurement is started. Writing the configuration here means we don't have
		// to keep it all in memory and avoids a lot of redundant writes later.

		// the API sets the preset mode to LOWPOWER_AUTONOMOUS here:
		// VL53L1_set_preset_mode() begin

		// VL53L1_preset_mode_standard_ranging() begin

		// values labeled "tuning parm default" are from vl53l1_tuning_parm_defaults.h
		// (API uses these in VL53L1_init_tuning_parm_storage_struct())

		// static config
		// API resets PAD_I2C_HV__EXTSUP_CONFIG here, but maybe we don't want to do
		// that? (seems like it would disable 2V8 mode)
		EndianIO.writeReg16Bit(i2cDevice, DSS_CONFIG__TARGET_TOTAL_RATE_MCPS, TargetRate); // should already be this
																							// value after reset
		EndianIO.writeReg(i2cDevice, GPIO__TIO_HV_STATUS, (byte) 0x02);
		EndianIO.writeReg(i2cDevice, SIGMA_ESTIMATOR__EFFECTIVE_PULSE_WIDTH_NS, (byte) 8); // tuning parm default
		EndianIO.writeReg(i2cDevice, SIGMA_ESTIMATOR__EFFECTIVE_AMBIENT_WIDTH_NS, (byte) 16); // tuning parm default
		EndianIO.writeReg(i2cDevice, ALGO__CROSSTALK_COMPENSATION_VALID_HEIGHT_MM, (byte) 0x01);
		EndianIO.writeReg(i2cDevice, ALGO__RANGE_IGNORE_VALID_HEIGHT_MM, (byte) 0xFF);
		EndianIO.writeReg(i2cDevice, ALGO__RANGE_MIN_CLIP, (byte) 0); // tuning parm default
		EndianIO.writeReg(i2cDevice, ALGO__CONSISTENCY_CHECK__TOLERANCE, (byte) 2); // tuning parm default

		// general config
		EndianIO.writeReg16Bit(i2cDevice, SYSTEM__THRESH_RATE_HIGH, 0x0000);
		EndianIO.writeReg16Bit(i2cDevice, SYSTEM__THRESH_RATE_LOW, 0x0000);
		EndianIO.writeReg(i2cDevice, DSS_CONFIG__APERTURE_ATTENUATION, (byte) 0x38);

		// timing config
		// most of these settings will be determined later by distance and timing
		// budget configuration
		EndianIO.writeReg16Bit(i2cDevice, RANGE_CONFIG__SIGMA_THRESH, (byte) 360); // tuning parm default
		EndianIO.writeReg16Bit(i2cDevice, RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS, (byte) 192); // tuning parm
																									// default

		// dynamic config

		EndianIO.writeReg(i2cDevice, SYSTEM__GROUPED_PARAMETER_HOLD_0, (byte) 0x01);
		EndianIO.writeReg(i2cDevice, SYSTEM__GROUPED_PARAMETER_HOLD_1, (byte) 0x01);
		EndianIO.writeReg(i2cDevice, SD_CONFIG__QUANTIFIER, (byte) 2); // tuning parm default

		// VL53L1_preset_mode_standard_ranging() end

		// from VL53L1_preset_mode_timed_ranging_*
		// GPH is 0 after reset, but writing GPH0 and GPH1 above seem to set GPH to 1,
		// and things don't seem to work if we don't set GPH back to 0 (which the API
		// does here).
		EndianIO.writeReg(i2cDevice, SYSTEM__GROUPED_PARAMETER_HOLD, (byte) 0x00);
		EndianIO.writeReg(i2cDevice, SYSTEM__SEED_CONFIG, (byte) 1); // tuning parm default

		// from VL53L1_config_low_power_auto_mode
		EndianIO.writeReg(i2cDevice, SYSTEM__SEQUENCE_CONFIG, (byte) 0x8B); // VHV, PHASECAL, DSS1, RANGE
		EndianIO.writeReg16Bit(i2cDevice, DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, 200 << 8);
		EndianIO.writeReg(i2cDevice, DSS_CONFIG__ROI_MODE_CONTROL, (byte) 2); // REQUESTED_EFFFECTIVE_SPADS

		// VL53L1_set_preset_mode() end

		// default to long range, 50 ms timing budget
		// note that this is different than what the API defaults to
		setDistance_mode(DistanceMode.Long);
		setMeasurementTimingBudget(50000);

		// VL53L1_StaticInit() end

		// the API triggers this change in VL53L1_init_and_start_range() once a
		// measurement is started; assumes MM1 and MM2 are disabled
		EndianIO.writeReg16Bit(i2cDevice, ALGO__PART_TO_PART_RANGE_OFFSET_MM,
				EndianIO.readReg16(i2cDevice, MM_CONFIG__OUTER_OFFSET_MM, false) * 4);

		return true;
	}

	/**
	 * @return the address
	 */
	public byte getAddress() {
		return address;
	}

	/**
	 * @param address the address to set
	 */
	public void setAddress(byte address) {
		EndianIO.writeReg(i2cDevice, I2C_SLAVE__DEVICE_ADDRESS, (byte) (address & 0x7F));
		this.address = address;
	}

	/**
	 * @return the io_timeout
	 */
	public int getIo_timeout() {
		return io_timeout;
	}

	/**
	 * @param io_timeout the io_timeout to set
	 */
	public void setIo_timeout(int io_timeout) {
		this.io_timeout = io_timeout;
	}

	/**
	 * @return the did_timeout
	 */
	public boolean isDid_timeout() {
		return did_timeout;
	}

	/**
	 * @param did_timeout the did_timeout to set
	 */
	public void setDid_timeout(boolean did_timeout) {
		this.did_timeout = did_timeout;
	}

	/**
	 * @return the timeout_start_ms
	 */
	public long getTimeout_start_ms() {
		return timeout_start_ms;
	}

	/**
	 * @param timeout_start_ms the timeout_start_ms to set
	 */
	public void setTimeout_start_ms(long timeout_start_ms) {
		this.timeout_start_ms = timeout_start_ms;
	}

	/**
	 * @return the fast_osc_frequency
	 */
	public int getFast_osc_frequency() {
		return fast_osc_frequency;
	}

	/**
	 * @param fast_osc_frequency the fast_osc_frequency to set
	 */
	public void setFast_osc_frequency(int fast_osc_frequency) {
		this.fast_osc_frequency = fast_osc_frequency;
	}

	/**
	 * @return the osc_calibrate_val
	 */
	public int getOsc_calibrate_val() {
		return osc_calibrate_val;
	}

	/**
	 * @param osc_calibrate_val the osc_calibrate_val to set
	 */
	public void setOsc_calibrate_val(int osc_calibrate_val) {
		this.osc_calibrate_val = osc_calibrate_val;
	}

	/**
	 * @return the calibrated
	 */
	public boolean isCalibrated() {
		return calibrated;
	}

	/**
	 * @param calibrated the calibrated to set
	 */
	public void setCalibrated(boolean calibrated) {
		this.calibrated = calibrated;
	}

	/**
	 * @return the saved_vhv_init
	 */
	public byte getSaved_vhv_init() {
		return saved_vhv_init;
	}

	/**
	 * @param saved_vhv_init the saved_vhv_init to set
	 */
	public void setSaved_vhv_init(byte saved_vhv_init) {
		this.saved_vhv_init = saved_vhv_init;
	}

	/**
	 * @return the saved_vhv_timeout
	 */
	public byte getSaved_vhv_timeout() {
		return saved_vhv_timeout;
	}

	/**
	 * @param saved_vhv_timeout the saved_vhv_timeout to set
	 */
	public void setSaved_vhv_timeout(byte saved_vhv_timeout) {
		this.saved_vhv_timeout = saved_vhv_timeout;
	}

	/**
	 * @return the distance_mode
	 */
	public DistanceMode getDistance_mode() {
		return distance_mode;
	}

	/**
	 * @param distance_mode the distance_mode to set
	 * @throws IOException
	 */
	public void setDistance_mode(DistanceMode distanceMode) throws IOException {
		// save existing timing budget
		int budget_us = getMeasurementTimingBudget();

		switch (distanceMode) {
		case Short:
			// from VL53L1_preset_mode_standard_ranging_short_range()

			// timing config
			EndianIO.writeReg(i2cDevice, RANGE_CONFIG__VCSEL_PERIOD_A, (byte) 0x07);
			EndianIO.writeReg(i2cDevice, RANGE_CONFIG__VCSEL_PERIOD_B, (byte) 0x05);
			EndianIO.writeReg(i2cDevice, RANGE_CONFIG__VALID_PHASE_HIGH, (byte) 0x38);

			// dynamic config
			EndianIO.writeReg(i2cDevice, SD_CONFIG__WOI_SD0, (byte) 0x07);
			EndianIO.writeReg(i2cDevice, SD_CONFIG__WOI_SD1, (byte) 0x05);
			EndianIO.writeReg(i2cDevice, SD_CONFIG__INITIAL_PHASE_SD0, (byte) 6); // tuning parm default
			EndianIO.writeReg(i2cDevice, SD_CONFIG__INITIAL_PHASE_SD1, (byte) 6); // tuning parm default

			break;

		case Medium:
			// from VL53L1_preset_mode_standard_ranging()

			// timing config
			EndianIO.writeReg(i2cDevice, RANGE_CONFIG__VCSEL_PERIOD_A, (byte) 0x0B);
			EndianIO.writeReg(i2cDevice, RANGE_CONFIG__VCSEL_PERIOD_B, (byte) 0x09);
			EndianIO.writeReg(i2cDevice, RANGE_CONFIG__VALID_PHASE_HIGH, (byte) 0x78);

			// dynamic config
			EndianIO.writeReg(i2cDevice, SD_CONFIG__WOI_SD0, (byte) 0x0B);
			EndianIO.writeReg(i2cDevice, SD_CONFIG__WOI_SD1, (byte) 0x09);
			EndianIO.writeReg(i2cDevice, SD_CONFIG__INITIAL_PHASE_SD0, (byte) 10); // tuning parm default
			EndianIO.writeReg(i2cDevice, SD_CONFIG__INITIAL_PHASE_SD1, (byte) 10); // tuning parm default

			break;

		case Long: // long
			// from VL53L1_preset_mode_standard_ranging_long_range()

			// timing config
			EndianIO.writeReg(i2cDevice, RANGE_CONFIG__VCSEL_PERIOD_A, (byte) 0x0F);
			EndianIO.writeReg(i2cDevice, RANGE_CONFIG__VCSEL_PERIOD_B, (byte) 0x0D);
			EndianIO.writeReg(i2cDevice, RANGE_CONFIG__VALID_PHASE_HIGH, (byte) 0xB8);

			// dynamic config
			EndianIO.writeReg(i2cDevice, SD_CONFIG__WOI_SD0, (byte) 0x0F);
			EndianIO.writeReg(i2cDevice, SD_CONFIG__WOI_SD1, (byte) 0x0D);
			EndianIO.writeReg(i2cDevice, SD_CONFIG__INITIAL_PHASE_SD0, (byte) 14); // tuning parm default
			EndianIO.writeReg(i2cDevice, SD_CONFIG__INITIAL_PHASE_SD1, (byte) 14); // tuning parm default

			break;

		default:
			// unrecognized mode - do nothing
		}

		// reapply timing budget
		setMeasurementTimingBudget(budget_us);

		// save mode so it can be returned by getDistanceMode()
		this.distance_mode = distanceMode;
	}

	// Set the measurement timing budget in microseconds, which is the time allowed
	// for one measurement. A longer timing budget allows for more accurate
	// measurements.
	// based on VL53L1_SetMeasurementTimingBudgetMicroSeconds()
	boolean setMeasurementTimingBudget(int budget_us) throws IOException {
		// assumes PresetMode is LOWPOWER_AUTONOMOUS

		if (budget_us <= TimingGuard) {
			return false;
		}

		int range_config_timeout_us = budget_us -= TimingGuard;
		if (range_config_timeout_us > 1100000) {
			return false;
		} // FDA_MAX_TIMING_BUDGET_US * 2

		range_config_timeout_us /= 2;

		// VL53L1_calc_timeout_register_values() begin

		int macro_period_us;

		// "Update Macro Period for Range A VCSEL Period"
		macro_period_us = calcMacroPeriod((byte) EndianIO.readReg(i2cDevice, RANGE_CONFIG__VCSEL_PERIOD_A, false));

		// "Update Phase timeout - uses Timing A"
		// Timeout of 1000 is tuning parm default
		// (TIMED_PHASECAL_CONFIG_TIMEOUT_US_DEFAULT)
		// via VL53L1_get_preset_mode_timing_cfg().
		int phasecal_timeout_mclks = timeoutMicrosecondsToMclks(1000, macro_period_us);
		if (phasecal_timeout_mclks > 0xFF) {
			phasecal_timeout_mclks = 0xFF;
		}
		EndianIO.writeReg(i2cDevice, PHASECAL_CONFIG__TIMEOUT_MACROP, (byte) phasecal_timeout_mclks);

		// "Update MM Timing A timeout"
		// Timeout of 1 is tuning parm default
		// (LOWPOWERAUTO_MM_CONFIG_TIMEOUT_US_DEFAULT)
		// via VL53L1_get_preset_mode_timing_cfg(). With the API, the register
		// actually ends up with a slightly different value because it gets assigned,
		// retrieved, recalculated with a different macro period, and reassigned,
		// but it probably doesn't matter because it seems like the MM ("mode
		// mitigation"?) sequence steps are disabled in low power auto mode anyway.
		EndianIO.writeReg16Bit(i2cDevice, MM_CONFIG__TIMEOUT_MACROP_A,
				encodeTimeout(timeoutMicrosecondsToMclks(1, macro_period_us)));

		// "Update Range Timing A timeout"
		EndianIO.writeReg16Bit(i2cDevice, RANGE_CONFIG__TIMEOUT_MACROP_A,
				encodeTimeout(timeoutMicrosecondsToMclks(range_config_timeout_us, macro_period_us)));

		// "Update Macro Period for Range B VCSEL Period"
		macro_period_us = calcMacroPeriod((byte) EndianIO.readReg(i2cDevice, RANGE_CONFIG__VCSEL_PERIOD_B, false));

		// "Update MM Timing B timeout"
		// (See earlier comment about MM Timing A timeout.)
		EndianIO.writeReg16Bit(i2cDevice, MM_CONFIG__TIMEOUT_MACROP_B,
				encodeTimeout(timeoutMicrosecondsToMclks(1, macro_period_us)));

		// "Update Range Timing B timeout"
		EndianIO.writeReg16Bit(i2cDevice, RANGE_CONFIG__TIMEOUT_MACROP_B,
				encodeTimeout(timeoutMicrosecondsToMclks(range_config_timeout_us, macro_period_us)));

		// VL53L1_calc_timeout_register_values() end

		return true;
	}

	// Get the measurement timing budget in microseconds
	// based on VL53L1_SetMeasurementTimingBudgetMicroSeconds()
	int getMeasurementTimingBudget() throws IOException {
		// assumes PresetMode is LOWPOWER_AUTONOMOUS and these sequence steps are
		// enabled: VHV, PHASECAL, DSS1, RANGE

		// VL53L1_get_timeouts_us() begin

		// "Update Macro Period for Range A VCSEL Period"
		int macro_period_us = calcMacroPeriod((byte) EndianIO.readReg(i2cDevice, RANGE_CONFIG__VCSEL_PERIOD_A, false));

		// "Get Range Timing A timeout"

		int range_config_timeout_us = timeoutMclksToMicroseconds(
				decodeTimeout(EndianIO.readReg16(i2cDevice, RANGE_CONFIG__TIMEOUT_MACROP_A, false)), macro_period_us);

		// VL53L1_get_timeouts_us() end

		return 2 * range_config_timeout_us + TimingGuard;
	}

	// Start continuous ranging measurements, with the given inter-measurement
	// period in milliseconds determining how often the sensor takes a measurement.
	void startContinuous(int period_ms) {
		// from VL53L1_set_inter_measurement_period_ms()
		EndianIO.writeReg32Bit(i2cDevice, SYSTEM__INTERMEASUREMENT_PERIOD, period_ms * osc_calibrate_val);

		EndianIO.writeReg(i2cDevice, SYSTEM__INTERRUPT_CLEAR, (byte) 0x01); // sys_interrupt_clear_range
		EndianIO.writeReg(i2cDevice, SYSTEM__MODE_START, (byte) 0x40); // mode_range__timed
	}

	// Stop continuous measurements
	// based on VL53L1_stop_range()
	void stopContinuous() {
		EndianIO.writeReg(i2cDevice, SYSTEM__MODE_START, (byte) 0x80); // mode_range__abort

		// VL53L1_low_power_auto_data_stop_range() begin

		calibrated = false;

		// "restore vhv configs"
		if (saved_vhv_init != 0) {
			EndianIO.writeReg(i2cDevice, VHV_CONFIG__INIT, saved_vhv_init);
		}
		if (saved_vhv_timeout != 0) {
			EndianIO.writeReg(i2cDevice, VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, saved_vhv_timeout);
		}

		// "remove phasecal override"
		EndianIO.writeReg(i2cDevice, PHASECAL_CONFIG__OVERRIDE, (byte) 0x00);

		// VL53L1_low_power_auto_data_stop_range() end
	}

	// Returns a range reading in millimeters when continuous mode is active
	// (readRangeSingleMillimeters() also calls this function after starting a
	// single-shot range measurement)
	int read(boolean blocking) throws IOException {
		if (blocking) {
			startTimeout();
			while (!dataReady()) {
				if (checkTimeoutExpired()) {
					did_timeout = true;
					ranging_data.range_status = RangeStatus.None;
					ranging_data.range_mm = 0;
					ranging_data.peak_signal_count_rate_MCPS = 0;
					ranging_data.ambient_count_rate_MCPS = 0;
					return ranging_data.range_mm;
				}
			}
		}

		readResults();

		if (!calibrated) {
			setupManualCalibration();
			calibrated = true;
		}

		updateDSS();

		getRangingData();

		EndianIO.writeReg(i2cDevice, SYSTEM__INTERRUPT_CLEAR, (byte) 0x01); // sys_interrupt_clear_range

		return ranging_data.range_mm;
	}

	// convert a RangeStatus to a readable string
	// Note that on an AVR, these strings are stored in RAM (dynamic memory), which
	// makes working with them easier but uses up 200+ bytes of RAM (many AVR-based
	// Arduinos only have about 2000 bytes of RAM). You can avoid this memory usage
	// if you do not call this function in your sketch.
	String rangeStatusToString(RangeStatus status) {
		switch (status) {
		case RangeValid:
			return "range valid";

		case SigmaFail:
			return "sigma fail";

		case SignalFail:
			return "signal fail";

		case RangeValidMinRangeClipped:
			return "range valid, min range clipped";

		case OutOfBoundsFail:
			return "out of bounds fail";

		case HardwareFail:
			return "hardware fail";

		case RangeValidNoWrapCheckFail:
			return "range valid, no wrap check fail";

		case WrapTargetFail:
			return "wrap target fail";

		case XtalkSignalFail:
			return "xtalk signal fail";

		case SynchronizationInt:
			return "synchronization int";

		case MinRangeFail:
			return "min range fail";

		case None:
			return "no update";

		default:
			return "unknown status";
		}
	}

	// Did a timeout occur in one of the read functions since the last call to
	// timeoutOccurred()?
	boolean timeoutOccurred() {
		boolean tmp = did_timeout;
		did_timeout = false;
		return tmp;
	}

	// "Setup ranges after the first one in low power auto mode by turning off
	// FW calibration steps and programming static values"
	// based on VL53L1_low_power_auto_setup_manual_calibration()
	private void setupManualCalibration() throws IOException {
		// "save original vhv configs"
		saved_vhv_init = (byte) EndianIO.readReg(i2cDevice, VHV_CONFIG__INIT, false);
		saved_vhv_timeout = (byte) EndianIO.readReg(i2cDevice, VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, false);

		// "disable VHV init"
		EndianIO.writeReg(i2cDevice, VHV_CONFIG__INIT, (byte) (saved_vhv_init & 0x7F));

		// "set loop bound to tuning param"
		EndianIO.writeReg(i2cDevice, VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND,
				(byte) ((saved_vhv_timeout & 0x03) + (3 << 2))); // tuning parm default
																	// (LOWPOWERAUTO_VHV_LOOP_BOUND_DEFAULT)

		// "override phasecal"
		EndianIO.writeReg(i2cDevice, PHASECAL_CONFIG__OVERRIDE, (byte) 0x01);
		EndianIO.writeReg(i2cDevice, CAL_CONFIG__VCSEL_START,
				(byte) EndianIO.readReg(i2cDevice, PHASECAL_RESULT__VCSEL_START, false));
	}

	// read measurement results into buffer
	void readResults() throws IOException {
		int position = 0;

		// range status
		results.range_status = (byte) EndianIO.readReg(i2cDevice, RESULT__RANGE_STATUS, false);

		// skip report status
		position += 2;

		// stream count
		results.stream_count = (byte) EndianIO.readReg(i2cDevice, RESULT__RANGE_STATUS + position, false);
		position++;

		// high and low byte
		results.dss_actual_effective_spads_sd0 = EndianIO.readReg(i2cDevice, RESULT__RANGE_STATUS + position,
				false) << 8;
		position++;
		results.dss_actual_effective_spads_sd0 |= EndianIO.readReg(i2cDevice, RESULT__RANGE_STATUS + position, false);

		// skip peak signam count rate
		position += 2;

		// high and low byte
		results.ambient_count_rate_mcps_sd0 = EndianIO.readReg(i2cDevice, RESULT__RANGE_STATUS + position, false) << 8;
		position++;
		results.ambient_count_rate_mcps_sd0 |= EndianIO.readReg(i2cDevice, RESULT__RANGE_STATUS + position, false);

		// skip siggma sd0
		position += 2;

		// skipi phase sd0
		position += 2;

		// high and low byte
		results.final_crosstalk_corrected_range_mm_sd0 = EndianIO.readReg(i2cDevice, RESULT__RANGE_STATUS + position,
				false) << 8;
		position++;
		results.final_crosstalk_corrected_range_mm_sd0 |= EndianIO.readReg(i2cDevice, RESULT__RANGE_STATUS + position,
				false);
		position++;

		// high and low byte
		results.peak_signal_count_rate_crosstalk_corrected_mcps_sd0 = EndianIO.readReg(i2cDevice,
				RESULT__RANGE_STATUS + position, false) << 8;
		position++;
		results.peak_signal_count_rate_crosstalk_corrected_mcps_sd0 |= EndianIO.readReg(i2cDevice,
				RESULT__RANGE_STATUS + position, false);
		position++;
	}

	// perform Dynamic SPAD Selection calculation/update
	// based on VL53L1_low_power_auto_update_DSS()
	void updateDSS() {
		int spadCount = results.dss_actual_effective_spads_sd0;

		if (spadCount != 0) {
			// "Calc total rate per spad"

			int totalRatePerSpad = results.peak_signal_count_rate_crosstalk_corrected_mcps_sd0
					+ results.ambient_count_rate_mcps_sd0;

			// "clip to 16 bits"
			if (totalRatePerSpad > 0xFFFF) {
				totalRatePerSpad = 0xFFFF;
			}

			// "shift up to take advantage of 32 bits"
			totalRatePerSpad <<= 16;

			totalRatePerSpad /= spadCount;

			if (totalRatePerSpad != 0) {
				// "get the target rate and shift up by 16"
				int requiredSpads = ((int) TargetRate << 16) / totalRatePerSpad;

				// "clip to 16 bit"
				if (requiredSpads > 0xFFFF) {
					requiredSpads = 0xFFFF;
				}

				// "override DSS config"
				EndianIO.writeReg16Bit(i2cDevice, DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, requiredSpads);
				// DSS_CONFIG__ROI_MODE_CONTROL should already be set to
				// REQUESTED_EFFFECTIVE_SPADS

				return;
			}
		}

		// If we reached this point, it means something above would have resulted in a
		// divide by zero.
		// "We want to gracefully set a spad target, not just exit with an error"

		// "set target to mid point"
		EndianIO.writeReg16Bit(i2cDevice, DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, 0x8000);
	}

	// get range, status, rates from results buffer
	// based on VL53L1_GetRangingMeasurementData()
	void getRangingData() {
		// VL53L1_copy_sys_and_core_results_to_range_results() begin

		int range = results.final_crosstalk_corrected_range_mm_sd0;

		// "apply correction gain"
		// gain factor of 2011 is tuning parm default
		// (VL53L1_TUNINGPARM_LITE_RANGING_GAIN_FACTOR_DEFAULT)
		// Basically, this appears to scale the result by 2011/2048, or about 98%
		// (with the 1024 added for proper rounding).
		ranging_data.range_mm = ((int) range * 2011 + 0x0400) / 0x0800;

		// VL53L1_copy_sys_and_core_results_to_range_results() end

		// set range_status in ranging_data based on value of RESULT__RANGE_STATUS
		// register
		// mostly based on ConvertStatusLite()
		switch (results.range_status) {
		case 17: // MULTCLIPFAIL
		case 2: // VCSELWATCHDOGTESTFAILURE
		case 1: // VCSELCONTINUITYTESTFAILURE
		case 3: // NOVHVVALUEFOUND
			// from SetSimpleData()
			ranging_data.range_status = RangeStatus.HardwareFail;
			break;

		case 13: // USERROICLIP
			// from SetSimpleData()
			ranging_data.range_status = RangeStatus.MinRangeFail;
			break;

		case 18: // GPHSTREAMCOUNT0READY
			ranging_data.range_status = RangeStatus.SynchronizationInt;
			break;

		case 5: // RANGEPHASECHECK
			ranging_data.range_status = RangeStatus.OutOfBoundsFail;
			break;

		case 4: // MSRCNOTARGET
			ranging_data.range_status = RangeStatus.SignalFail;
			break;

		case 6: // SIGMATHRESHOLDCHECK
			ranging_data.range_status = RangeStatus.SignalFail;
			break;

		case 7: // PHASECONSISTENCY
			ranging_data.range_status = RangeStatus.WrapTargetFail;
			break;

		case 12: // RANGEIGNORETHRESHOLD
			ranging_data.range_status = RangeStatus.XtalkSignalFail;
			break;

		case 8: // MINCLIP
			ranging_data.range_status = RangeStatus.RangeValidMinRangeClipped;
			break;

		case 9: // RANGECOMPLETE
			// from VL53L1_copy_sys_and_core_results_to_range_results()
			if (results.stream_count == 0) {
				ranging_data.range_status = RangeStatus.RangeValidNoWrapCheckFail;
			} else {
				ranging_data.range_status = RangeStatus.RangeValid;
			}
			break;

		default:
			ranging_data.range_status = RangeStatus.None;
		}

		// from SetSimpleData()
		ranging_data.peak_signal_count_rate_MCPS = countRateFixedToFloat(
				results.peak_signal_count_rate_crosstalk_corrected_mcps_sd0);
		ranging_data.ambient_count_rate_MCPS = countRateFixedToFloat(results.ambient_count_rate_mcps_sd0);
	}

	// Decode sequence step timeout in MCLKs from register value
	// based on VL53L1_decode_timeout()
	int decodeTimeout(int reg_val) {
		return ((int) (reg_val & 0xFF) << (reg_val >> 8)) + 1;
	}

	// Encode sequence step timeout register value from timeout in MCLKs
	// based on VL53L1_encode_timeout()
	int encodeTimeout(int timeout_mclks) {
		// encoded format: "(LSByte * 2^MSByte) + 1"

		int ls_byte = 0;
		int ms_byte = 0;

		if (timeout_mclks > 0) {
			ls_byte = timeout_mclks - 1;

			while ((ls_byte & 0xFFFFFF00) > 0) {
				ls_byte >>= 1;
				ms_byte++;
			}

			return (ms_byte << 8) | (ls_byte & 0xFF);
		} else {
			return 0;
		}
	}

	// Convert sequence step timeout from macro periods to microseconds with given
	// macro period in microseconds (12.12 format)
	// based on VL53L1_calc_timeout_us()
	int timeoutMclksToMicroseconds(int timeout_mclks, int macro_period_us) {
		return (int) ((long) timeout_mclks * macro_period_us + 0x800) >> 12;
	}

	// Convert sequence step timeout from microseconds to macro periods with given
	// macro period in microseconds (12.12 format)
	// based on VL53L1_calc_timeout_mclks()
	int timeoutMicrosecondsToMclks(int timeout_us, int macro_period_us) {
		return (((int) timeout_us << 12) + (macro_period_us >> 1)) / macro_period_us;
	}

	// Calculate macro period in microseconds (12.12 format) with given VCSEL period
	// assumes fast_osc_frequency has been read and stored
	// based on VL53L1_calc_macro_period_us()
	int calcMacroPeriod(byte vcsel_period) {
		// from VL53L1_calc_pll_period_us()
		// fast osc frequency in 4.12 format; PLL period in 0.24 format
		int pll_period_us = ((int) 0x01 << 30) / fast_osc_frequency;

		// from VL53L1_decode_vcsel_period()
		byte vcsel_period_pclks = (byte) ((vcsel_period + 1) << 1);

		// VL53L1_MACRO_PERIOD_VCSEL_PERIODS = 2304
		int macro_period_us = (int) 2304 * pll_period_us;
		macro_period_us >>= 6;
		macro_period_us *= vcsel_period_pclks;
		macro_period_us >>= 6;

		return macro_period_us;
	}
}
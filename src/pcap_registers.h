#ifndef registers_h
#define registers_h

#include "pcap_types.h"

// Bit             Bit Name                Settings                    Bit Description
// ----------------------------------------------------------------------------------------------------------
// 7:6             I2C_A                   0 to 3                      Complement to the I²C-address
// 5:2             OLF_FTUNE               0 : Minimum                 Fine-tune the low-frequency clock
//                                         7 : Typ.,recommended
//                                         15 : Maximum
// 1:0             OLF_CTUNE               0 : 10kHz                   Coarse-tune the low-frequency clock
//                                         1 : 50kHz
//                                         2 : 100kHz
//                                         3 : 200kHz
union PCAP_REG_CFG0_T
{
    unsigned char REGVAL;
    struct
    {

        unsigned char OLF_CTUNE : 2;
        unsigned char OLF_FTUNE : 4;
        unsigned char I2C_A : 2;

    } CFG0; /* data */
};

// Bit             Bit Name                Settings                    Bit Description
// ----------------------------------------------------------------------------------------------------------
// 7               OX_DIS                  Default : 0                 Disable the OX clock
// 5               OX_DIV4                 0 : No division;            OX clock frequency : Raw freq./ 4
//                                         f_ox = 2MHz
//                                         1 : Division by 4;
//                                         f_ox = 0.5MHz
// 4               OX_AUTOSTOP_DIS         Default : 0                 ams internal bits
// 3               OX_STOP                 Default : 0                 ams internal bits
// 2:0             OX_RUN                  0 : Generator off           Control the permanency or the latency of
//                                         6 : OX latency = 1 / fOLF   the OX generator. Latency means an
//                                         3 : OX latency = 2 / fOLF   oscillator settling time before a
//                                         2 : OX latency = 31 / fOLF  measurement starts
//                                         1 : OX runs permanently
union PCAP_REG_CFG1_T
{
    unsigned char REGVAL;
    struct
    {
        unsigned char OX_RUN : 3;
        const unsigned char OX_STOP : 1;
        const unsigned char OX_AUTOSTOP_DIS : 1;
        unsigned char OX_DIV4 : 1;
        unsigned char RSRVD : 1;
        unsigned char OX_DIS : 1;

    } CFG1;
};

// Bit             Bit Name                Settings                    Bit Description
// ----------------------------------------------------------------------------------------------------------
// 7:4             RDCHG_INT_SEL           0 : 180kΩ                   Same, but for ports PC4 – PC5
//                                         1 : 90kΩ
//                                         2 : 30kΩ (default)          Choice of one out of 4 on-chip discharge
//                                         3 : 10kΩ                    resistors for the CDC ports PC0 – PC3 plus
//                                                                     internal port PC6
// 3               RDCHG_INT_EN            0 : Off
//                                         1 : Internal on (default)   Enable internal discharge resistors
// 1               RDCHG_EXT_EN            0 : Off (default)           Enable external discharge resistor
//                                         1 : External on             switching on PCAUX during discharge
//                                                                     phase (High-Z during pre- and
//                                                                     full-charge phase)
//                                                                     Note: AUX_PD_DIS (PCAUX pull down
//                                                                     disable) : 1
union PCAP_REG_CFG2_T
{
    unsigned char REGVAL;
    struct
    {

        unsigned char RSRVD0 : 1;
        unsigned char RDCHG_EXT_EN : 1;
        unsigned char RSRVD1 : 1;
        unsigned char RDCHG_INT_EN : 1;
        unsigned char RDCHG_INT_SEL0 : 2;
        unsigned char RDCHG_INT_SEL1 : 2;

    } CFG2;
};

// Bit             Bit Name                Settings                    Bit Description
// ----------------------------------------------------------------------------------------------------------
// 6               AUX_PD_DIS              0 : Pull-down active        Disable pull-down at PCAUX
//                                         1 : Pull-down disabled
// 5               AUX_CINT                0 : Normal (default)        Activates auxiliary Port PCAUX during
//                                         1 : Aux during c-internal   internal c-reference conversion only
//                                         active
// 4:3             RDCHG_OPEN              2 : Recommended             ams internal bits
// 2               RDCHG_PERM_EN           0 : Off (default)           Keep the chip-internal discharge resistor
//                                         1 : On                      permanently connected.
// 1               RDCHG_EXT_PERM          0 : Off (default)           Activates auxiliary Port PCAUX
//                                         1 : On                      permanently for a) permanently
//                                                                     discharge or b) to add an offset
//                                                                     capacitance to every charge/discharge
//                                                                     cycle
//                                                                     Note: AUX_PD_DIS (PCAUX pull down
//                                                                     disable) : 1
// 0               RCHG_SEL                0 : 180kΩ                   Choice of one out of 2 on-chip charging
//                                         1 : 10kΩ (default)          resistors for the CDC, permitting to limit
//                                                                     the charging current, avoiding transients
union PCAP_REG_CFG3_T
{
    unsigned char REGVAL;
    struct
    {

        unsigned char RCHG_SEL : 1;
        unsigned char RDCHG_EXT_PERM : 1;
        unsigned char RDCHG_PERM_EN : 1;
        unsigned char RDCHG_OPEN : 2;
        unsigned char AUX_CINT : 1;
        unsigned char AUX_PD_DIS : 1;
        unsigned char RSRVD : 1;

    } CFG3;
};

// Bit             Bit Name                Settings                    Bit Description
// ----------------------------------------------------------------------------------------------------------
// 7               C_REF_INT               0: External reference at    Use on-chip reference capacitor at CDC
//                                         PC0/GND or PC0/PC1)         special ports PC6
//                                         1: Internal reference
// 5               C_COMP_EXT              0 : Idle                    Activate the compensation mechanism
//                                         1 : Active; must be avoided for off-chip parasitic capacitances
//                                         when C_FLOATING==0
// 4               C_COMP_INT              0 : Idle                    Activate the compensation mechanism
//                                         1 : Active                  for on-chip parasitic capacitances and
//                                                                     gain compensation
// 1               C_DIFFERENTIAL          0 : Ordinary                Select between single or differential
//                                         1 : Differential            sensors
// 0               C_FLOATING              0 : Grounded                Select between grounded or floating
//                                         1 : Floating                sensors
union PCAP_REG_CFG4_T
{
    unsigned char REGVAL;
    struct
    {

        unsigned char C_FLOATING : 1;
        unsigned char C_DIFFERENTIAL : 1;
        unsigned char RSRVD0 : 2;
        unsigned char C_COMP_INT : 1;
        unsigned char C_COMP_EXT : 1;
        unsigned char RSRVD1 : 1;
        unsigned char C_REF_INT : 1;

    } CFG4;
};

// Bit             Bit Name                Settings                    Bit Description
// ----------------------------------------------------------------------------------------------------------
// 7               CY_PRE_MR1_SHORT        0 : Normal (recommended)    Reduce delays between internal clock
//                                         1 : Reduced                 paths
// 5               C_PORT_PAT              0 : Normal                  The order of the measured ports will be
//                                         1 : Alternating order of    reversed after each sequence. If C_
//                                         ports                       PORT_PAT is activated then C_AVRG + C_
//                                                                     FAKE should be an even number
// 3               CY_HFCLK_SEL            0 : OLF                     Clock source for the CDC
//                                         1 : OHF
// 2               CY_DIV4_DIS             0 : Off                     Quadruple the clock period
//                                         1 : On                      (only in combination with CY_HFCLK_
//                                                                     SEL == 1)
// 1               CY_PRE_LONG             0 : Off, recommended        Adds safety delay between internal clock
//                                         1 : On                      paths
// 0               C_DC_BALANCE            0 : Off ("single HiZ")      Only for differential floating mode (other
//                                         1 : DC free ("both HiZ")    modi are DC free),
//                                                                     changes port control to eliminate DC at
//                                                                     Capacity sense
union PCAP_REG_CFG5_T
{
    unsigned char REGVAL;
    struct
    {

        unsigned char C_DC_BALANCE : 1;
        unsigned char CY_PRE_LONG : 1;
        unsigned char CY_DIV4_DIS : 1;
        unsigned char CY_HFCLK_SEL : 1;
        unsigned char RSRVD0 : 1;
        unsigned char C_PORT_PAT : 1;
        unsigned char RSRVD1 : 1;
        unsigned char CY_PRE_MR1_SHORT : 1;

    } CFG5;
};

// Bit             Bit Name                Settings                    Bit Description
// ----------------------------------------------------------------------------------------------------------
// 5:0             C_PORT_EN               0x00 : All off, the CDC     Enables bitwise the CDC ports from PC0
//                                         will not work               to PC5, bit #0 for port PC0, #1 for PC1 etc.
//                                         0x01 : Only port PC0 is
//                                         activated etc.
//                                         0x3F : All ports activated
union PCAP_REG_CFG6_T
{
    unsigned char REGVAL;
    struct
    {

        unsigned char C_PORT_EN : 6;
        unsigned char RSRVD : 2;

    } CFG6;
};

// Bit             Bit Name                Settings                    Bit Description
// ----------------------------------------------------------------------------------------------------------
// 7:0            C_AVRG[7..0]              0, 1 : Sample size = 1      Sample size for averaging (calculating
//                                         2 : Sample size = 2         the mean value) over CDC
//                                         3 : Sample size = 3         measurements
//                                         ...
//                                         8191 : Maximum sample
//                                         size
union PCAP_REG_CFG7_T
{
    unsigned char REGVAL;
    struct
    {

        unsigned char C_AVRG_LOW : 8;

    } CFG7;
};

// Bit             Bit Name                Settings                    Bit Description
// ----------------------------------------------------------------------------------------------------------
// 3:0            C_AVG[12..8]             0, 1 : Sample size = 1      Sample size for averaging (calculating
//                                         2 : Sample size = 2         the mean value) over CDC
//                                         3 : Sample size = 3         measurements
//                                         ...
//                                         8191 : Maximum sample
//                                         size
union PCAP_REG_CFG8_T
{
    unsigned char REGVAL;
    struct
    {

        unsigned char C_AVRG_HIGH : 5;

    } CFG8;
};

// Bit             Bit Name                Settings                    Bit Description
// ----------------------------------------------------------------------------------------------------------
// 7:0             CONV_TIME[7:0]          Concerning CDC, a           Conversion trigger period or:
//                                         particular period for       sequence period (in stretched mode)
//                                         triggering the
//                                         measurements
//                                         Tconv./seq =
//                                         2 * CONV_TIME[..] / fOLF
union PCAP_REG_CFG9_T
{
    unsigned char REGVAL;
    struct
    {

        unsigned char CONV_TIME_LOW : 8;

    } CFG9;
};

// Bit             Bit Name                Settings                    Bit Description
// ----------------------------------------------------------------------------------------------------------
// 7:0             CONV_TIME[15:8]         Concerning CDC, a           Conversion trigger period or:
//                                         particular period for       sequence period (in stretched mode)
//                                         triggering the
//                                         measurements
//                                         Tconv./seq =
//                                         2 * CONV_TIME[..] / fOLF
union PCAP_REG_CFG10_T
{
    unsigned char REGVAL;
    struct
    {

        unsigned char CONV_TIME_MID : 8;

    } CFG10;
};

// Bit             Bit Name                Settings                    Bit Description
// ----------------------------------------------------------------------------------------------------------
// 5:0             CONV_TIME[22:16]        Concerning CDC, a           Conversion trigger period or:
//                                         particular period for       sequence period (in stretched mode)
//                                         triggering the
//                                         measurements
//                                         Tconv./seq =
//                                         2 * CONV_TIME[..] / fOLF
union PCAP_REG_CFG11_T
{
    unsigned char REGVAL;

    struct
    {

        unsigned char CONV_TIME_HIGH : 7;
        unsigned char RSRVD : 1;

    } CFG11;
};

// Bit             Bit Name                Settings                    Bit Description
// ----------------------------------------------------------------------------------------------------------
// 7:0 &           DISCHARGE_TIME[7:0]     OLF:                        Sets CDC discharge time
// Reg.13: 1:0                             Tdischarge =                Tdischarge. Time interval
//                                         (DISCHARGE_TIME + 1) *      reserved for discharge time
//                                         Tcycleclock                 measurement.
//                                         OHF:
//                                         Tdischarge =
//                                         (DISCHARGE_TIME + 0) *
//                                         Tcycleclock
//                                         1023 : Off
union PCAP_REG_CFG12_T
{
    unsigned char REGVAL;
    struct
    {

        unsigned char DISCHARGE_TIME_LOW : 8;

    } CFG12;
};

// Bit             Bit Name                Settings                    Bit Description
// ----------------------------------------------------------------------------------------------------------
// 7:6             C_STARTONPIN            0 : PG0, 1 : PG1,           Selection of the GPIO port that permits
//                                         2 : PG2, 3 : PG3            triggering a CDC start
// 4:2             C_TRIG_SEL              0 : Continuous              CDC Trigger Mode
//                                         1 : Read triggered
//                                         2 : Timer triggered
//                                         3 : Timer triggered
//                                         (stretched)
//                                         4 : n.d.
//                                         5 : Pin triggered
//                                         6 : Opcode triggered
//                                         (7 : continuous_exp, not
//                                         recommended)
// 1:0 &
// Reg12: 7:0      DISCHARGE_TIME[10:8]    See Register 12             See Register 12
union PCAP_REG_CFG13_T
{
    unsigned char REGVAL;
    struct
    {

        unsigned char DISCHARGE_TIME_HIGH : 2;
        unsigned char C_TRIG_SEL : 3;
        unsigned char RSRVD : 1;
        unsigned char C_STARTONPIN : 2;

    } CFG13;
};

// Bit             Bit Name                Settings                    Bit Description
// ----------------------------------------------------------------------------------------------------------
// 7:0 &           PRECHARGE_TIME[7:0]     OLF:                        Sets CDC discharge time
// Reg.15: 1:0                             Tprecharge =                Tprecharge. Time interval
//                                         (PRECHARGE_TIME + 1) *      reserved for discharge time
//                                         Tcycleclock                 measurement.
//                                         OHF:
//                                         Tprecharge =
//                                         (PRECHARGE_TIME + 0) *
//                                         Tcycleclock
//                                         1023 : Off
union PCAP_REG_CFG14_T
{
    unsigned char REGVAL;
    struct
    {

        unsigned char PRECHARGE_TIME_LOW : 8;

    } CFG14;
};

// Bit             Bit Name                Settings                    Bit Description
// ----------------------------------------------------------------------------------------------------------
// 5:2             C_FAKE                  0 : None                    Number of ”fake“ or ”warm-up”
//                                         1 : 1 fake                  measurements for the CDC, performed
//                                         …                           just before the “real” ones; the “fake”
//                                         15: 15 fakes                values do not count
// 1:0 &
// Reg14: 7:0      PRECHARGE_TIME[10:8]    See Register 14             See Register 14
union PCAP_REG_CFG15_T
{
    unsigned char REGVAL;
    struct
    {

        unsigned char PRECHARGE_TIME_HIGH : 2;
        unsigned char C_FAKE : 4;
        unsigned char RSRVD : 2;

    } CFG15;
};

// Bit             Bit Name                Settings                    Bit Description
// ----------------------------------------------------------------------------------------------------------
// 7:0 &           FULLCHARGE_TIME[7:0]    OLF:                        Sets CDC discharge time
// Reg.15: 1:0                             Tprecharge =                Tprecharge. Time interval
//                                         (FULLCHARGE_TIME + 1) *     reserved for discharge time
//                                         Tcycleclock                 measurement.
//                                         OHF:
//                                         Tprecharge =
//                                         (FULLCHARGE_TIME + 0) *
//                                         Tcycleclock
//                                         1023 : Off
union PCAP_REG_CFG16_T
{
    unsigned char REGVAL;
    struct
    {

        unsigned char FULLCHARGE_TIME_LOW : 8;

    } CFG16;
};

// Bit             Bit Name                Settings                    Bit Description
// ----------------------------------------------------------------------------------------------------------
// 1:0 &           FULLCHARGE_TIME[10:8]   See Register 16             See Register 16
// Reg.16: 7:0
// 6:2             C_REF_SEL               0 : Minimum                 Setting the on-chip
//                                         1 : Approx.1pF              reference capacitor for the CDC
//                                         ...                         Note: Step width varies from 0.3pF to
//                                         31 : Maximum (approx.31pF)  1.5pF
union PCAP_REG_CFG17_T
{
    unsigned char REGVAL;
    struct
    {

        unsigned char FULLCHARGE_TIME_HIGH : 2;
        unsigned char C_REF_SEL : 5;
        unsigned char RSRVD : 1;

    } CFG17;
};

// Bit             Bit Name                Settings                    Bit Description
// ----------------------------------------------------------------------------------------------------------
// 7               C_G_OP_RUN              Guard: OP Mode              0 : Permanent
//                                                                     1 : Pulsed (set OP to sleep mode between
//                                                                     conversions)
// 6               C_G_OP_EXT              Guard: Activate external OP 0 : Internal OP
//                                                                     1 : External OP, PG3 as C_G_MUX_SEL
// 5:0             C_G_EN                  Guard Enable, for each port b’xxxxx1 : Activates port PC0
//                                                                     b’xxxx1x : Activates port PC1
//                                                                     b’xxx1xx : Activates port PC2
//                                                                     b’xx1xxx : Activates port PC3
//                                                                     b’x1xxxx : Activates port PC4
//                                                                     b’1xxxxx : Activates port PC5
union PCAP_REG_CFG18_T
{
    unsigned char REGVAL;
    struct
    {

        unsigned char C_G_EN : 6;
        unsigned char C_G_OP_EXT : 1;
        unsigned char C_G_OP_RUN : 1;

    } CFG18;
};

// Bit             Bit Name                Settings                    Bit Description
// ----------------------------------------------------------------------------------------------------------
// 7:6             C_G_OP_VU               Guard: OP gain (from Sense  0 : x 1.00
//                                         Port to Guard)              1 : x 1.01
//                                                                     2 : x 1.02
//                                                                     3 : x 1.03
// 5:4             C_G_OP_ATTN             Guard: OP attenuation       0 : 0.5aF
//                                                                     1 : 1.0aF
//                                                                     2 : 1.5aF
//                                                                     3 : 2.0aF
// 3:0             C_G_TIME                Guard: Time during          t : C_G_TIME * 500ns
//                                         Precharge to switch Guard
//                                         Port from "direct
//                                         connected" to OP
union PCAP_REG_CFG19_T
{
    unsigned char REGVAL;
    struct
    {

        unsigned char C_G_TIME : 4;
        unsigned char C_G_OP_ATTN : 2;
        unsigned char C_G_OP_VU : 2;

    } CFG19;
};

// Bit             Bit Name                Settings                    Bit Description
// ----------------------------------------------------------------------------------------------------------
// 7               R_CY                    Cycle-time for the RDC      OLF f       R_CY=0      R_CY=1
//                                         Precharge/Charge/           10kHz       100μs       200μs
//                                         Discharge, depending on     50kHz       20μs        40μs
//                                         OLF frequency               100kHz      10μs        20μs
//                                                                     200kHz      20μs        40μs
// 2:0             C_G_OP_TR               Guard OP current trim       0 :
//                                                                     …
//                                                                     7 : Recommended
union PCAP_REG_CFG20_T
{
    unsigned char REGVAL;
    struct
    {

        unsigned char C_G_OP_TR : 3;
        unsigned char RSRVD : 4;
        unsigned char R_CY : 1;

    } CFG20;
};

// Bit             Bit Name                Settings                    Bit Description
// ----------------------------------------------------------------------------------------------------------
// 7:0 &           R_TRIG_PREDIV[7:0]      0, 1 : Every signal         Pre-divider, permits to make less
// Reg22: 1:0                              triggers                    temperature measurements than
//                                         2 : Every 2nd signal        capacitance measurements. This is a
//                                         triggers                    factor between measurement rates of
//                                         3 : Every 3rd signal        CDC over RDC. It is used also as OLF clock
//                                         triggers                    divider if OLF is used as trigger source.
//                                         ...
//                                         1023 : Maximum factor
union PCAP_REG_CFG21_T
{
    unsigned char REGVAL;
    struct
    {

        unsigned char R_TRIG_PREDIV_LOW : 8;

    } CFG21;
};

// Bit             Bit Name                Settings                    Bit Description
// ----------------------------------------------------------------------------------------------------------
// 6:4             R_TRIG_SEL              0 : Off                     Trigger source selection for the RDC
//                                         1 : Timer triggered         5 & 6: triggered by the end of CDC
//                                         3 : Pin triggered           conversion
//                                         5 : CDC asynchronous
//                                         (recommended)
//                                         6 : CDC synchronous
// 3:2             R_AVRG                  0 : Not averaged            Sample size for the mean value
//                                         1 : 4-fold averaged         calculation (averaging) in the RDC part
//                                         2 : 8-fold averaged
//                                         3 : 16-fold averaged
// 1:0 &           R_TRIG_PREDIV[10:8]     0, 1 : Every signal trigger Pre-divider, permits to make less
// Reg21: 7:0                              2 : Every 2nd signal        temperature measurements than
//                                         3 : Every 3rd signal        capacitance measurements. This is a
//                                         ...                         factor between measurement rates of
//                                         1023 : Maximum factor       CDC over RDC. It is used also as OLF clock
//                                                                     divider if OLF is used as trigger source.
union PCAP_REG_CFG22_T
{
    unsigned char REGVAL;
    struct
    {

        unsigned char R_TRIG_PREDIV_HIGH : 2;
        unsigned char R_AVRG : 2;
        unsigned char R_TRIG_SEL : 3;
        unsigned char RSRVD : 1;

    } CFG22;
};

// Bit             Bit Name                Settings                    Bit Description
// ----------------------------------------------------------------------------------------------------------
// 7:6             R_PORT_EN               ‘bx0 : Disabled             Port activation for the RDC part
//                                         ‘bx1 : Activates port PT0REF
//                                         ‘b0x : Disabled
//                                         ‘b1x : Activates port PT1
// 5               R_PORT_EN_IMES          0 : Disabled                Port activation for internal aluminum
//                                         1 : Enabled                 temperature sensor
// 4               R_PORT_EN_IREF          0 : Disabled                Port activation for internal reference
//                                         1 : Enabled                 resistor
// 2               R_FAKE                  0: 2 fake cycles per        Number of ”fake“ or ”warm-up”
//                                         average value               measurements for the RDC, performed
//                                         1: 8 fake cycle per         just before the “real” ones; the “fake”
//                                         average value               values do not count
// 1:0             R_STARTONPIN            0 : PG0                     Selection of the GPIO port that permits
//                                         1 : PG1,                    triggering a RDC star
//                                         2 : PG2
//                                         3 : PG3
union PCAP_REG_CFG23_T
{
    unsigned char REGVAL;
    struct
    {

        unsigned char R_STARTONPIN : 2;
        unsigned char R_FAKE : 1;
        unsigned char RSRVD : 1;
        unsigned char R_PORT_EN_IREF : 1;
        unsigned char R_PORT_EN_IMES : 1;
        unsigned char R_PORT_EN : 2;

    } CFG23;
};

// Bit             Bit Name                Settings                    Bit Description
// ----------------------------------------------------------------------------------------------------------
// 5:4             TDC_CHAN_EN             Mandatory : 3               ams internal bits
// 3               TDC_ALUPERMOPEN         Mandatory : 0               ams internal bits
// 2               TDC_NOISE_DIS           Mandatory : 0               ams internal bits
// 1:0             TDC_MUPU_SPEED          Mandatory : 3               ams internal bits
union PCAP_REG_CFG24_T
{
    unsigned char REGVAL;
    struct
    {

        unsigned char TDC_MUPU_SPEED : 2;
        unsigned char TDC_NOISE_DIS : 1;
        unsigned char TDC_ALUPERMOPEN : 1;
        unsigned char TDC_CHAN_EN : 2;
        unsigned char RSRVD : 2;

    } CFG24;
};

// Bit             Bit Name                Settings                    Bit Description
// ----------------------------------------------------------------------------------------------------------
// 7:2             TDC_MUPU_NO             Mandatory : 1               ams internal bits
union PCAP_REG_CFG25_T
{
    unsigned char REGVAL;
    struct
    {

        unsigned char RSRVD : 2;
        unsigned char TDC_MUPU_NO : 6;

    } CFG25;
};

// Bit             Bit Name                Settings                    Bit Description
// ----------------------------------------------------------------------------------------------------------
// 7:2             TDC_QHA_SEL             Mandatory : 20              ams internal bits
// 1               TDC_NOISE_CY_DIS        Mandatory : 1               ams internal bits
union PCAP_REG_CFG26_T
{
    unsigned char REGVAL;
    struct
    {

        unsigned char RSRVD : 1;
        unsigned char TDC_NOISE_CY_DIS : 1;
        unsigned char TDC_QHA_SEL : 6;

    } CFG26;
};

// Bit             Bit Name                Settings                    Bit Description
// ----------------------------------------------------------------------------------------------------------
// 7:6             DSP_MOFLO_EN            0 : Off                     Enable the mono-flop (anti-bouncing
//                                         3 : On                      filter)” in the GPIO pulse line.
// 3:2             DSP_SPEED               0 : Fastest
//                                         1 : Fast                    DSP speed
//                                         2 : Slow, recommended
//                                         3 : Slowest
// 1               PG1xPG3                 0 : Pulse output at PG3     Switch PG1/PG3 wiring to/from DSP
//                                         1 : Pulse output at PG1
// 0               PG0xPG2                 0 : Pulse output at PG2     Switch PG0/PG2 wiring to/from DSP
//                                         1 : Pulse output at PG0
union PCAP_REG_CFG27_T
{
    unsigned char REGVAL;
    struct
    {

        unsigned char PG0xPG2 : 1;
        unsigned char PG1xPG3 : 1;
        unsigned char DSP_SPEED : 2;
        unsigned char RSRVD : 2;
        unsigned char DSP_MOFLO_EN : 2;

    } CFG27;
};

// Bit             Bit Name                Settings                    Bit Description
// ----------------------------------------------------------------------------------------------------------
// 7:0             WD_DIS                  0x5A : Watchdog disabled    Watchdog Disable,
//                                         (off)                       to disable Watchdog 0x5A has to be
//                                         0x00 (recommended) /        written to this register. The watchdog
//                                         else : Watchdog enabled     period is between 9s and 15s
union PCAP_REG_CFG28_T
{
    unsigned char REGVAL;
    struct
    {

        unsigned char WD_DIS : 8;

    } CFG28;
};

// Bit             Bit Name                Settings                    Bit Description
// ----------------------------------------------------------------------------------------------------------
// 7:4             DSP_STARTONPIN          Bitwise PG0 to PG3          Pin mask for starting the DSP - This mask
//                                                                     permits assigning one or more GPIO pins
//                                                                     to start the DSP
// 3:0             DSP_FF_IN               Bitwise DSP_IN_0 to         Pin mask for flip-flop activation
//                                         DSP_IN_3
union PCAP_REG_CFG29_T
{
    unsigned char REGVAL;
    struct
    {

        unsigned char DSP_FF_IN : 4;
        unsigned char DSP_STARTONPIN : 4;

    } CFG29;
};

// Bit             Bit Name                Settings                    Bit Description
// ----------------------------------------------------------------------------------------------------------
// 7               PG5_INTN_EN             0 : PG5 normal operation    Route INTN Signal to PG5
//                                         1 : PG5 <== INTN
// 6               PG4_INTN_EN             0 : PG4 normal operation    Route INTN Signal to PG4
//                                         1 : PG4 <== INTN
// 2:0             DSP_START_EN            'bxxx1 : Trigger by         DSP Trigger Enable
//                                         end of CDC
//                                         'bxx1x : Trigger by
//                                         end of RDC (recommended)
//                                         'bx1xx : Trigger by timer
union PCAP_REG_CFG30_T
{
    unsigned char REGVAL;
    struct
    {

        unsigned char DSP_START_EN : 3;
        unsigned char RSRVD : 3;
        unsigned char PG4_INTN_EN : 1;
        unsigned char PG5_INTN_EN : 1;

    } CFG30;
};

// Bit             Bit Name                Settings                    Bit Description
// ----------------------------------------------------------------------------------------------------------
// 7               PI1_TOGGLE_EN           0 : Normal operation        Activates toggle flip flop at Pulse
//                                         1 : Toggle flip flop active Interface 1 Output
//                                                                     especially for PDM to create 1:1 duty factor
// 6               PI0_TOGGLE_EN           0 : Normal operation        Activates toggle flip flop at Pulse
//                                         1 : Toggle flip flop active Interface 0 Output
//                                                                     especially for PDM to create 1:1 duty factor
// 5:4             PI0_RES                 0 : 10 bit                  Resolution of the pulse-code interfaces
//                                         1 : 12 bit
//                                         2 : 14 bit
//                                         3 : 16 bit
// 3               PI0_PDM_SEL             0 : PWM                     Pulse Interface 0
//                                         1 : PDM                     PWM / PDM switch
// 2:0             PI0_CLK_SEL             0 : Off                     Pulse Interface 0
//                                         1 : OLF / 1                 Clock Select
//                                         2 : OLF / 2
//                                         3 : OLF / 4
//                                         4 : OX / 1
//                                         5 : OX / 2
//                                         6 : OX / 4
//                                         7 : n.d.
union PCAP_REG_CFG31_T
{
    unsigned char REGVAL;
    struct
    {

        unsigned char PI0_CLK_SEL : 3;
        unsigned char PI0_PDM_SEL : 1;
        unsigned char PI0_RES : 2;
        unsigned char PI0_TOGGLE_EN : 1;
        unsigned char PI1_TOGGLE_EN : 1;

    } CFG31;
};

// Bit             Bit Name                Settings                    Bit Description
// ----------------------------------------------------------------------------------------------------------
// 5:4             PI1_RES                 0 : 10 bit                  Resolution of the pulse-code interfaces
//                                         1 : 12 bit
//                                         2 : 14 bit
//                                         3 : 16 bit
// 3               PI1_PDM_SEL             0 : PWM                     Pulse Interface 1
//                                         1 : PDM                     PWM / PDM switch
// 2:0             PI1_CLK_SEL             0 : Off                     Pulse Interface 1
//                                         1 : OLF / 1                 Clock Select
//                                         2 : OLF / 2
//                                         3 : OLF / 4
//                                         4 : OX / 1
//                                         5 : OX / 2
//                                         6 : OX / 4
//                                         7 : n.d.
union PCAP_REG_CFG32_T
{
    unsigned char REGVAL;
    struct
    {

        unsigned char PI1_CLK_SEL : 3;
        unsigned char PI1_PDM_SEL : 1;
        unsigned char PI1_RES : 2;
        unsigned char RSRVD : 2;

    } CFG32;
};

// Bit             Bit Name                Settings                    Bit Description
// ----------------------------------------------------------------------------------------------------------
// 7:4             PG_DIR_IN               0 : Output                  Toggles general-purpose port direction
//                                         1 : Input                   between input and output
//                                                                     #4: PG0
//                                                                     #5: PG1
//                                                                     #6: PG2
//                                                                     #7: PG3
// 3:0             PG_PU                   0 : Pull-up disabled        Activates protective pull-up resistors at
//                                         1 : Pull-up active          general-purpose ports
//                                                                     #0: PG0
//                                                                     #1: PG1
//                                                                     #2: PG2
//                                                                     #3: PG3
union PCAP_REG_CFG33_T
{
    unsigned char REGVAL;
    struct
    {

        unsigned char PG_PU : 4;
        unsigned char PG_DIR_IN : 4;

    } CFG33;
};

// Bit             Bit Name                Settings                    Bit Description
// ----------------------------------------------------------------------------------------------------------
// 7               INT_TRIG_BG             0 : Disabled                End of Reading triggers Bandgap
//                                         1 : Enabled
// 6               DSP_TRIG_BG             0 : Disabled                Bandgap refresh is triggered by the DSP
//                                         1 : Enabled                 bit setting
// 5               BG_PERM                 0 : Bandgap pulsed
//                                         1 : Bandgap permanent       Activate Bandgap permanently. With
//                                         enabled                     BG_PERM = 1 the current consumption
//                                                                     rises by approx. 20μA
// 4               AUTOSTART               0 : Disabled                For standalone operation, triggers CDC
//                                         1 : CDC trigger after       after Power On
//                                         Power On
// 3:0             AMS_INT_REG             Mandatory : 7               ams internal bit
union PCAP_REG_CFG34_T
{
    unsigned char REGVAL;
    struct
    {

        unsigned char AMS_INT_REG : 4;
        unsigned char AUTOSTART : 1;
        unsigned char BG_PERM : 1;
        unsigned char DSP_TRIG_BG : 1;
        unsigned char INT_TRIG_BG : 1;

    } CFG34;
};

// Bit             Bit Name                Settings                    Bit Description
// ----------------------------------------------------------------------------------------------------------
// 7:0             CDC_GAIN_CORR           Recommended                 Firmware defined configuration of the
//                                         1.25 ==> 0x40               gain correction factor.
//                                                                     Bits 0 to 7 of 8fpp
//                                                                     0 : 0
//                                                                     n : 1 + n/256
union PCAP_REG_CFG35_T
{
    unsigned char REGVAL;
    struct
    {

        unsigned char CDC_GAIN_CORR : 8;

    } CFG35;
};

// Bit             Bit Name                Settings                    Bit Description
// ----------------------------------------------------------------------------------------------------------
// 7:0             -                                                   Not used
union PCAP_REG_CFG36_T
{
    unsigned char REGVAL;
    struct
    {

        unsigned char RSRVD : 8;

    } CFG36;
};

// Bit             Bit Name                Settings                    Bit Description
// ----------------------------------------------------------------------------------------------------------
// 7:0             -                                                   Not used
union PCAP_REG_CFG37_T
{
    unsigned char REGVAL;
    struct
    {

        unsigned char RSRVD : 8;

    } CFG37;
};

// Bit             Bit Name                Settings                    Bit Description
// ----------------------------------------------------------------------------------------------------------
// 7:0             BG_TIME                 0: Recommended              Firmware defined
union PCAP_REG_CFG38_T
{
    unsigned char REGVAL;
    struct
    {

        unsigned char BG_TIME : 8;

    } CFG38;
};

// Bit             Bit Name                Settings                    Bit Description
// ----------------------------------------------------------------------------------------------------------
// 7:4             PULSE_SEL1              0 to 5 : Res0 to Res5       Firmware defined, select source for Pulse IF 1
//                                         (C0..5/Cref @PCap04_standard)
//                                         6 : Res6
//                                         (PT1/Ref @PCap04_standard)
//                                         7 : Res7
//                                         (Alu/Ref @PCap04_standard)
// 3:0             PULSE_SEL0              0 to 5 : Res0 to Res5       Firmware defined, select source for Pulse IF 0
//                                         (C0..5/Cref @PCap04_standard)
//                                         6 : Res6
//                                         (PT1/Ref @PCap04_standard)
//                                         7 : Res7
//                                         (Alu/Ref @PCap04_standard)
union PCAP_REG_CFG39_T
{
    unsigned char REGVAL;
    struct
    {

        unsigned char PULSE_SEL0 : 4;
        unsigned char PULSE_SEL1 : 4;

    } CFG39;
};

// Bit             Bit Name                Settings                    Bit Description
// ----------------------------------------------------------------------------------------------------------
// 7:0             C_SENSE_SEL             PCap04_linearize firmware   Firmware defined
//                                         only, select C ratio
//                                         for linearization
//                                         0..5 : C0 to 5 / Cref
union PCAP_REG_CFG40_T
{
    unsigned char REGVAL;
    struct
    {

        unsigned char C_SENSE_SEL : 8;

    } CFG40;
};

// Bit             Bit Name                Settings                    Bit Description
// ----------------------------------------------------------------------------------------------------------
// 7:0             R_SENSE_SEL             PCap04_linearize firmware   Firmware defined
//                                         only, select R ratio
//                                         for temperature
//                                         determination
union PCAP_REG_CFG41_T
{
    unsigned char REGVAL;
    struct
    {

        unsigned char R_SENSE_SEL : 8;

    } CFG41;
};

// Bit             Bit Name                Settings                    Bit Description
// ----------------------------------------------------------------------------------------------------------
// 6               ALARM1_SELECT           PCap04_linearize            Firmware defined. Select source of alarm
// 4               ALARM0_SELECT           firmware only               signals at PG0 and PG1
//                                         Polarity                    Active High
//                                         Select 0 : Z;
//                                         1 : Theta
// 3               EN_ASYNC_READ           1 : Active                  Values in result registers Res0 to Res7
//                                                                     are only updated if the previous value
//                                                                     has been read
// 2               HS_MODE_SEL             0 : Mandatory               ams internal bit
// 1               R_MEDIAN_EN             PCap04_linearize            Enable median filters for ci/ri in linearize
// 0               C_MEDIAN_EN             firmware only               firmware
union PCAP_REG_CFG42_T
{
    unsigned char REGVAL;
    struct
    {

        unsigned char C_MEDIAN_EN : 1;
        unsigned char R_MEDIAN_EN : 1;
        unsigned char HS_MODE_SEL : 1;
        unsigned char EN_ASYNC_READ : 1;
        unsigned char ALARM0_SELECT : 1;
        unsigned char RSRVD0 : 1;
        unsigned char ALARM1_SELECT : 1;
        unsigned char RSRVD1 : 1;
    } CFG42;
};

// Bit             Bit Name                Settings                    Bit Description
// ----------------------------------------------------------------------------------------------------------
// 7:0             -                                                   Not used
union PCAP_REG_CFG43_T
{
    unsigned char REGVAL;
    struct
    {

        unsigned char RSRVD : 8;

    } CFG43;
};

// Bit             Bit Name                Settings                    Bit Description
// ----------------------------------------------------------------------------------------------------------
// 7:0             -                                                   Not used
union PCAP_REG_CFG44_T
{
    unsigned char REGVAL;
    struct
    {

        unsigned char RSRVD : 8;

    } CFG44;
};

// Bit             Bit Name                Settings                    Bit Description
// ----------------------------------------------------------------------------------------------------------
// 7:0             -                                                   Not used
union PCAP_REG_CFG45_T
{
    unsigned char REGVAL;
    struct
    {

        unsigned char RSRVD : 8;

    } CFG45;
};

// Bit             Bit Name                Settings                    Bit Description
// ----------------------------------------------------------------------------------------------------------
// 7:0             -                                                   Not used
union PCAP_REG_CFG46_T
{
    unsigned char REGVAL;
    struct
    {

        unsigned char RSRVD : 8;

    } CFG46;
};

// Bit             Bit Name                Settings                    Bit Description
// ----------------------------------------------------------------------------------------------------------
// 0               RUNBIT                  0 : Off = the chip system   On/off switch for front-end and DSP:
//                                         is idle and protected       It should be ”off“ during
//                                         1 : On = the protection is  programming and any registry
//                                         removed,                    modification, thus protecting the
//                                         and the system may run      chip from any
//                                                                     undesirable/unspecified states
union PCAP_REG_CFG47_T
{
    unsigned char REGVAL;
    struct
    {

        unsigned char RUNBIT : 1;
        unsigned char RSRVD : 7;

    } CFG47;
};

// Bit             Bit Name                Settings                    Bit Description
// ----------------------------------------------------------------------------------------------------------
// 3:0             MEM_LOCK                NVRAM range                 Data secure function to safe parts of
//                                         'bxxx1 :  `d0 to 703        NVRAM from reading and writing
//                                         'bxx1x :  `d704 to 831      via SIF
//                                         'bx1xx :  `d832 to 959
//                                         'b1xxx :  `d960 to 1007 and
//                                         `d1022 to 1023
union PCAP_REG_CFG48_T
{
    unsigned char REGVAL;
    struct
    {

        unsigned char MEM_LOCK : 4;
        unsigned char RSRVD : 4;

    } CFG48;
};

// Bit             Bit Name                Settings                    Bit Description
// ----------------------------------------------------------------------------------------------------------
// 7:0             SERIAL_NUMBER[7:0]      Free disposal for customer. Lower byte reserved for serial number
//                                         Could only be written as
//                                         far as the byte is zero.
//                                         Afterwards it could just be
//                                         cleared by an complete Erase
union PCAP_REG_CFG49_T
{
    unsigned char REGVAL;
    struct
    {

        unsigned char SERIAL_NUMBER_LOW : 8;

    } CFG49;
};

// Bit             Bit Name                Settings                    Bit Description
// ----------------------------------------------------------------------------------------------------------
// 7:0            SERIAL_NUMBER[15:8]      Free disposal for customer. Higher byte reserved for serial number
//                                         Could only be written as
//                                         far as the byte is zero.
//                                         Afterwards it could just be
//                                         cleared by an complete Erase
union PCAP_REG_CFG50_T
{
    unsigned char REGVAL;
    struct
    {

        unsigned char SERIAL_NUMBER_HIGH : 8;

    } CFG50;
};

// Bit             Bit Name                Settings                    Bit Description
// ----------------------------------------------------------------------------------------------------------
// 7:0             -                                                   mandatory 0x00
union PCAP_REG_CFG51_T
{
    unsigned char REGVAL;
    struct
    {

        unsigned char RSRVD : 8;

    } CFG51;
};

// Bit             Bit Name                Settings                    Bit Description
// ----------------------------------------------------------------------------------------------------------
// 7:0             -                                                   mandatory 0x00
union PCAP_REG_CFG52_T
{
    unsigned char REGVAL;
    struct
    {

        unsigned char RSRVD : 8;

    } CFG52;
};

// Bit             Bit Name                Settings                    Bit Description
// ----------------------------------------------------------------------------------------------------------
// 7:0             -                                                   mandatory 0x00
union PCAP_REG_CFG53_T
{
    unsigned char REGVAL;
    struct
    {

        unsigned char RSRVD : 8;

    } CFG53;
};

// Bit             Bit Name                Settings                    Bit Description
// ----------------------------------------------------------------------------------------------------------
// 7:0             MEM_CTRL                0x2d : NVRAM store enable   Register is reset automatically
//                                         0x59 : NVRAM recall enable  after following SIF activity
//                                         0xb8 : NVRAM erase          Memory control
union PCAP_REG_CFG54_T
{
    unsigned char REGVAL;
    struct
    {

        unsigned char MEM_CTRL : 8;

    } CFG54;
};

// Bit             Bit Name                Settings                    Bit Description
// ----------------------------------------------------------------------------------------------------------
// 7:0             -                                                   mandatory 0x00
union PCAP_REG_CFG55_T
{
    unsigned char REGVAL;
    struct
    {

        unsigned char RSRVD : 8;

    } CFG55;
};

// Bit             Bit Name                Settings                    Bit Description
// ----------------------------------------------------------------------------------------------------------
// 7:0             -                                                   mandatory 0x00
union PCAP_REG_CFG56_T
{
    unsigned char REGVAL;
    struct
    {

        unsigned char RSRVD : 8;

    } CFG56;
};

// Bit             Bit Name                Settings                    Bit Description
// ----------------------------------------------------------------------------------------------------------
// 7:0             -                                                   mandatory 0x00
union PCAP_REG_CFG57_T
{
    unsigned char REGVAL;
    struct
    {

        unsigned char RSRVD : 8;

    } CFG57;
};

// Bit             Bit Name                Settings                    Bit Description
// ----------------------------------------------------------------------------------------------------------
// 7:0             -                                                   mandatory 0x00
union PCAP_REG_CFG58_T
{
    unsigned char REGVAL;
    struct
    {

        unsigned char RSRVD : 8;

    } CFG58;
};

// Bit             Bit Name                Settings                    Bit Description
// ----------------------------------------------------------------------------------------------------------
// 7:0             -                                                   mandatory 0x00
union PCAP_REG_CFG59_T
{
    unsigned char REGVAL;
    struct
    {

        unsigned char RSRVD : 8;

    } CFG59;
};

// Bit             Bit Name                Settings                    Bit Description
// ----------------------------------------------------------------------------------------------------------
// 7:0             -                                                   mandatory 0x00
union PCAP_REG_CFG60_T
{
    unsigned char REGVAL;
    struct
    {

        unsigned char RSRVD : 8;

    } CFG60;
};

// Bit             Bit Name                Settings                    Bit Description
// ----------------------------------------------------------------------------------------------------------
// 7:0             -                                                   mandatory 0x00
union PCAP_REG_CFG61_T
{
    unsigned char REGVAL;
    struct
    {

        unsigned char RSRVD : 8;

    } CFG61;
};

// Bit             Bit Name                Settings                    Bit Description
// ----------------------------------------------------------------------------------------------------------
// 7:0             CHARGE_PUMP[7:0]        Individual, device-specific Lower byte of NVRAM charge pump trim
//                                         setting. Not allowed to be
//                                         changed
// Important Note: We guarantee the data for data retention and
// endurance only under the assumption, that the customer does
// not change the registers 62 and 63. In addition, it is mandatory
// to follow the given procedure for ERASE NVRAM as described
// in section NVRAM and ROM precisely. Otherwise, we do no
// longer guarantee the data retention time and endurance
// cycles.
union PCAP_REG_CFG62_T
{
    unsigned char REGVAL;
    struct
    {

        unsigned char CHARGE_PUMP_LOW : 8;

    } CFG62;
};

// Bit             Bit Name                Settings                    Bit Description
// ----------------------------------------------------------------------------------------------------------
// 7:0             CHARGE_PUMP[15:8]       Individual, device-specific Higher byte of NVRAM charge pump trim
//                                         setting. Not allowed to be
//                                         changed
union PCAP_REG_CFG63_T
{
    unsigned char REGVAL;
    struct
    {

        unsigned char CHARGE_PUMP_HIGH : 8;

    } CFG63;
};

// Addr        Name        <D7>    <D6>    <D5>    <D4>    <D3>    <D2>    <D1>    <D0>
// 0           RES0        7                                                       0
// 1                       15                                                      8
// 2                       23                                                      16
// 3                       31                                                      24
union PCAP_REG_RES0_T
{
    unsigned int REGVAL;
    struct
    {

        unsigned int RES01 : 8;
        unsigned int RES02 : 8;
        unsigned int RES03 : 8;
        unsigned int RES04 : 8;

    } RES0;
};

// Addr        Name        <D7>    <D6>    <D5>    <D4>    <D3>    <D2>    <D1>    <D0>
// 0           RES1        7                                                       0
// 1                       15                                                      8
// 2                       23                                                      16
// 3                       31                                                      24
union PCAP_REG_RES1_T
{
    unsigned int REGVAL;
    struct
    {

        unsigned int RES11 : 8;
        unsigned int RES12 : 8;
        unsigned int RES13 : 8;
        unsigned int RES14 : 8;

    } RES1;
};

// Addr        Name        <D7>    <D6>    <D5>    <D4>    <D3>    <D2>    <D1>    <D0>
// 0           RES2        7                                                       0
// 1                       15                                                      8
// 2                       23                                                      16
// 3                       31                                                      24
union PCAP_REG_RES2_T
{
    unsigned int REGVAL;
    struct
    {

        unsigned int RES11 : 8;
        unsigned int RES12 : 8;
        unsigned int RES13 : 8;
        unsigned int RES14 : 8;

    } RES2;
};

// Addr        Name        <D7>    <D6>    <D5>    <D4>    <D3>    <D2>    <D1>    <D0>
// 0           RES3        7                                                       0
// 1                       15                                                      8
// 2                       23                                                      16
// 3                       31                                                      24
union PCAP_REG_RES3_T
{
    unsigned int REGVAL;
    struct
    {

        unsigned int RES31 : 8;
        unsigned int RES32 : 8;
        unsigned int RES33 : 8;
        unsigned int RES34 : 8;

    } RES3;
};

// Addr        Name        <D7>    <D6>    <D5>    <D4>    <D3>    <D2>    <D1>    <D0>
// 0           RES4        7                                                       0
// 1                       15                                                      8
// 2                       23                                                      16
// 3                       31                                                      24
union PCAP_REG_RES4_T
{
    unsigned int REGVAL;
    struct
    {

        unsigned int RES41 : 8;
        unsigned int RES42 : 8;
        unsigned int RES43 : 8;
        unsigned int RES44 : 8;

    } RES4;
};

// Addr        Name        <D7>    <D6>    <D5>    <D4>    <D3>    <D2>    <D1>    <D0>
// 0           RES5        7                                                       0
// 1                       15                                                      8
// 2                       23                                                      16
// 3                       31                                                      24
union PCAP_REG_RES5_T
{
    unsigned int REGVAL;
    struct
    {

        unsigned int RES51 : 8;
        unsigned int RES52 : 8;
        unsigned int RES53 : 8;
        unsigned int RES54 : 8;

    } RES5;
};

// Addr        Name        <D7>    <D6>    <D5>    <D4>    <D3>    <D2>    <D1>    <D0>
// 0           RES6        7                                                       0
// 1                       15                                                      8
// 2                       23                                                      16
// 3                       31                                                      24
union PCAP_REG_RES6_T
{
    unsigned int REGVAL;
    struct
    {

        unsigned int RES61 : 8;
        unsigned int RES62 : 8;
        unsigned int RES63 : 8;
        unsigned int RES64 : 8;

    } RES6;
};

// Addr        Name        <D7>    <D6>    <D5>    <D4>    <D3>    <D2>    <D1>    <D0>
// 0           RES7        7                                                       0
// 1                       15                                                      8
// 2                       23                                                      16
// 3                       31                                                      24
union PCAP_REG_RES7_T
{
    unsigned int REGVAL;
    struct
    {

        unsigned int RES71 : 8;
        unsigned int RES72 : 8;
        unsigned int RES73 : 8;
        unsigned int RES74 : 8;

    } RES7;
};

// Addr        Name        <D7>         <D6>            <D5>            <D4>        <D3>        <D2>        <D1>        <D0>
// 32          STATUS0     POR_FLAG_WDG POR_FLAG_CONFIG IR_FLAG_COLL    AUTOBOOT                RDC_READY   CDC_ACTIVE  RUNBIT
union PCAP_REG_STATUS0_T
{
    unsigned char REGVAL;
    struct
    {

        unsigned char RUNBIT : 1;
        unsigned char CDC_ACTIVE : 1;
        unsigned char RDC_READY : 1;
        unsigned char RSRVD : 1;
        unsigned char AUTOBOOT_BUSY : 1;
        unsigned char POR_CDC_DSP_COLL : 1;
        unsigned char POR_FLAG_CONFIG : 1;
        unsigned char POR_FLAG_WDOG : 1;

    } STATUS0;
};

// Addr        Name        <D7>         <D6>            <D5>            <D4>        <D3>        <D2>        <D1>        <D0>
// 33          STATUS1                                                              RDC_ERR     MUP_ERR     ERR_OVFL    COMB_ERR
union PCAP_REG_STATUS1_T
{
    unsigned char REGVAL;
    struct
    {

        unsigned char COMB_ERR : 1;
        unsigned char ERR_OVFL : 1;
        unsigned char MUP_ERR : 1;
        unsigned char RDC_ERR : 1;
        unsigned char RSRVD : 4;

    } STATUS1;
};

// Addr        Name        <D7>         <D6>            <D5>            <D4>        <D3>        <D2>        <D1>        <D0>
// 33          STATUS2                  C_PORTERR_INT   C_PORTERR5     C_PORTERR4   C_PORTERR3  C_PORTERR2  C_PORTERR1  C_PORTERR0
union PCAP_REG_STATUS2_T
{
    unsigned char REGVAL;
    struct
    {

        unsigned char C_PORTERR0 : 1;
        unsigned char C_PORTERR1 : 1;
        unsigned char C_PORTERR2 : 1;
        unsigned char C_PORTERR3 : 1;
        unsigned char C_PORTERR4 : 1;
        unsigned char C_PORTERR5 : 1;
        unsigned char C_PORTERR_INT : 1;
        unsigned char RSRVD : 1;

    } STATUS2;
};

struct __PCAP_RESULTS_REGS_T 
{
    PCAP_REG_RES0_T RES0 = {.REGVAL = 0};
    PCAP_REG_RES1_T RES1 = {.REGVAL = 0};
    PCAP_REG_RES2_T RES2 = {.REGVAL = 0};
    PCAP_REG_RES3_T RES3 = {.REGVAL = 0};
    PCAP_REG_RES4_T RES4 = {.REGVAL = 0};
    PCAP_REG_RES5_T RES5 = {.REGVAL = 0};
    PCAP_REG_RES6_T RES6 = {.REGVAL = 0};
    PCAP_REG_RES7_T RES7 = {.REGVAL = 0};
    PCAP_REG_STATUS0_T STATUS0 = {.REGVAL = 0};
    PCAP_REG_STATUS1_T STATUS1 = {.REGVAL = 0};
    PCAP_REG_STATUS2_T STATUS2 = {.REGVAL = 0};
}RESULT_REGS_packed;

struct __PCAP_CONFIG_REGS_T 
{
    PCAP_REG_CFG0_T CFG0 = {.REGVAL = 0};
    PCAP_REG_CFG1_T CFG1 = {.REGVAL = 0};
    PCAP_REG_CFG2_T CFG2 = {.REGVAL = 0};
    PCAP_REG_CFG3_T CFG3 = {.REGVAL = 0};
    PCAP_REG_CFG4_T CFG4 = {.REGVAL = 0};
    PCAP_REG_CFG5_T CFG5 = {.REGVAL = 0};
    PCAP_REG_CFG6_T CFG6 = {.REGVAL = 0};
    PCAP_REG_CFG7_T CFG7 = {.REGVAL = 0};
    PCAP_REG_CFG8_T CFG8 = {.REGVAL = 0};
    PCAP_REG_CFG9_T CFG9 = {.REGVAL = 0};
    PCAP_REG_CFG10_T CFG10 = {.REGVAL = 0};
    PCAP_REG_CFG11_T CFG11 = {.REGVAL = 0};
    PCAP_REG_CFG12_T CFG12 = {.REGVAL = 0};
    PCAP_REG_CFG13_T CFG13 = {.REGVAL = 0};
    PCAP_REG_CFG14_T CFG14 = {.REGVAL = 0};
    PCAP_REG_CFG15_T CFG15 = {.REGVAL = 0};
    PCAP_REG_CFG16_T CFG16 = {.REGVAL = 0};
    PCAP_REG_CFG17_T CFG17 = {.REGVAL = 0};
    PCAP_REG_CFG18_T CFG18 = {.REGVAL = 0};
    PCAP_REG_CFG19_T CFG19 = {.REGVAL = 0};
    PCAP_REG_CFG20_T CFG20 = {.REGVAL = 0};
    PCAP_REG_CFG21_T CFG21 = {.REGVAL = 0};
    PCAP_REG_CFG22_T CFG22 = {.REGVAL = 0};
    PCAP_REG_CFG23_T CFG23 = {.REGVAL = 0};
    PCAP_REG_CFG24_T CFG24 = {.REGVAL = 0};
    PCAP_REG_CFG25_T CFG25 = {.REGVAL = 0};
    PCAP_REG_CFG26_T CFG26 = {.REGVAL = 0};
    PCAP_REG_CFG27_T CFG27 = {.REGVAL = 0};
    PCAP_REG_CFG28_T CFG28 = {.REGVAL = 0};
    PCAP_REG_CFG29_T CFG29 = {.REGVAL = 0};
    PCAP_REG_CFG30_T CFG30 = {.REGVAL = 0};
    PCAP_REG_CFG31_T CFG31 = {.REGVAL = 0};
    PCAP_REG_CFG32_T CFG32 = {.REGVAL = 0};
    PCAP_REG_CFG33_T CFG33 = {.REGVAL = 0};
    PCAP_REG_CFG34_T CFG34 = {.REGVAL = 0};
    PCAP_REG_CFG35_T CFG35 = {.REGVAL = 0};
    PCAP_REG_CFG36_T CFG36 = {.REGVAL = 0};
    PCAP_REG_CFG37_T CFG37 = {.REGVAL = 0};
    PCAP_REG_CFG38_T CFG38 = {.REGVAL = 0};
    PCAP_REG_CFG39_T CFG39 = {.REGVAL = 0};
    PCAP_REG_CFG40_T CFG40 = {.REGVAL = 0};
    PCAP_REG_CFG41_T CFG41 = {.REGVAL = 0};
    PCAP_REG_CFG42_T CFG42 = {.REGVAL = 0};
    PCAP_REG_CFG43_T CFG43 = {.REGVAL = 0};
    PCAP_REG_CFG44_T CFG44 = {.REGVAL = 0};
    PCAP_REG_CFG45_T CFG45 = {.REGVAL = 0};
    PCAP_REG_CFG46_T CFG46 = {.REGVAL = 0};
    PCAP_REG_CFG47_T CFG47 = {.REGVAL = 0};
    PCAP_REG_CFG48_T CFG48 = {.REGVAL = 0};
    PCAP_REG_CFG49_T CFG49 = {.REGVAL = 0};
    PCAP_REG_CFG50_T CFG50 = {.REGVAL = 0};
    PCAP_REG_CFG51_T CFG51 = {.REGVAL = 0};
    PCAP_REG_CFG52_T CFG52 = {.REGVAL = 0};
    PCAP_REG_CFG53_T CFG53 = {.REGVAL = 0};
    PCAP_REG_CFG54_T CFG54 = {.REGVAL = 0};
    PCAP_REG_CFG55_T CFG55 = {.REGVAL = 0};
    PCAP_REG_CFG56_T CFG56 = {.REGVAL = 0};
    PCAP_REG_CFG57_T CFG57 = {.REGVAL = 0};
    PCAP_REG_CFG58_T CFG58 = {.REGVAL = 0};
    PCAP_REG_CFG59_T CFG59 = {.REGVAL = 0};
    PCAP_REG_CFG60_T CFG60 = {.REGVAL = 0};
    PCAP_REG_CFG61_T CFG61 = {.REGVAL = 0};
    PCAP_REG_CFG62_T CFG62 = {.REGVAL = 0};
    PCAP_REG_CFG63_T CFG63 = {.REGVAL = 0};
}CONFIG_REGS_packed;

typedef __PCAP_CONFIG_REGS_T* pcap_config_handler_t;

struct __PCAP_FW_T
{
    unsigned char data[PCAP_NVRAM_FW_SIZE];
}_FW_packed;

struct __PCAP_FW_CAL0_T
{
    unsigned char data[PCAP_NVRAM_FW_CAL0_SIZE];
}_FW_CAL0_packed;

struct __PCAP_FW_CAL1_T
{
    unsigned char data[PCAP_NVRAM_FW_CAL1_SIZE];
}_FW_CAL1_packed;

struct __PCAP_NVRAM_T 
{
    __PCAP_FW_T FW;
    __PCAP_FW_CAL0_T FW_CAL0;
    __PCAP_FW_CAL1_T FW_CAL1;
    __PCAP_CONFIG_REGS_T CFG;
}PCAP_NVRAM_packed;

#endif
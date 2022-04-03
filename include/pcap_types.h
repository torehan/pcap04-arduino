#ifndef pcap_types_h
#define pcap_types_h

#include <pcap_defines.h>

enum pcap04_version_t
{
    PCAP04_V0 = 0,
    PCAP04_V1,
};


enum regtype_t
{
    CFG_REG = 0,
    READ_REG,
};

enum pcap_operation_modes_t
{
    STAND_ALONE = 0,
    PRE_CONFIGURED,
    PURE_SLAVE
};

enum pcap_measurement_modes_t
{
    STANDARD = 0,
    HUMIDITY,
    PRESSURE
};
enum pcap_serial_interface_t
{
    PCAP_I2C_MODE,
    PCAP_SPI_MODE
};


union pcap_i2c_addr_t
{
    unsigned char i2c_addr;
    struct
    {
        unsigned char wr : 1;
        unsigned char addr : 2;
        unsigned char fixed_addr : 5;
    } i2caddr;
};

union pcap_opcode_nvram_t
{
    unsigned int opcode;
    struct __attribute__((packed))
    {
        unsigned int data : 8;
        unsigned int addr : 10;
        unsigned int op_code : 6;
    } nvram;
};

union pcap_opcode_config_t
{
    unsigned int opcode;
    struct __attribute__((packed))
    {
        unsigned int data : 8;
        unsigned int addr : 6;
        unsigned int op_code : 10;
    } config;
};

union pcap_opcode_result_t
{
    unsigned short opcode;
    struct __attribute__((packed))
    {
        unsigned short data : 8;
        unsigned short addr : 6;
        unsigned short op_code : 2;
    } result;
};

union pcap_opcode_command_t
{
    unsigned char opcode;
    struct __attribute__((packed))
    {
        unsigned char op_code : 8;
    } command;
};

union pcap_opcode_testread_t
{
    unsigned short opcode;
    struct __attribute__((packed))
    {
        unsigned short fixed : 8;
        unsigned short op_code : 8;
    } testread;
};

template <typename T>
struct pcap_reg_t
{
    regtype_t reg_type;
    const unsigned char reg_name[16];
    const unsigned char addr;
    T reg;
};
struct pcap_config_t
{
    unsigned char OLF_CTUNE : 2;
    unsigned char OLF_FTUNE : 4;
    unsigned char I2C_A : 2;

    unsigned char OX_RUN : 3;
    // bool OX_STOP;              // ams internal bits
    // bool OX_AUTOSTOP_DIS;      // ams internal bits
    bool OX_DIV4;
    bool OX_DIS;

    bool RDCHG_EXT_EN;
    bool RDCHG_INT_EN;
    unsigned char RDCHG_INT_SEL0 : 2;
    unsigned char RDCHG_INT_SEL1 : 2;

    bool RCHG_SEL;
    bool RDCHG_EXT_PERM;
    bool RDCHG_PERM_EN;
    unsigned char RDCHG_OPEN : 2;
    bool AUX_CINT;
    bool AUX_PD_DIS;

    bool C_FLOATING;
    bool C_DIFFERENTIAL;
    bool C_COMP_INT;
    bool C_COMP_EXT;
    bool C_REF_INT;

    bool C_DC_BALANCE;
    bool CY_PRE_LONG;
    bool CY_DIV4_DIS;
    bool CY_HFCLK_SEL;
    bool C_PORT_PAT;
    bool CY_PRE_MR1_SHORT;

    unsigned char C_PORT_EN : 6;

    unsigned short C_AVRG : 13;

    unsigned int CONV_TIME : 23;

    unsigned short DISCHARGE_TIME : 10;
    unsigned char C_TRIG_SEL : 3;
    unsigned char C_STARTONPIN : 2;

    unsigned short PRECHARGE_TIME : 10;
    unsigned char C_FAKE : 4;

    unsigned short FULLCHARGE_TIME : 10;
    unsigned char C_REF_SEL : 5;

    unsigned char C_G_EN : 6;
    bool C_G_OP_EXT;
    bool C_G_OP_RUN;

    unsigned char C_G_TIME : 4;
    unsigned char C_G_OP_ATTN : 2;
    unsigned char C_G_OP_VU : 2;

    unsigned char C_G_OP_TR : 3;
    bool R_CY;

    unsigned short R_TRIG_PREDIV : 10;
    unsigned char R_AVRG : 2;
    unsigned char R_TRIG_SEL : 3;

    unsigned char R_STARTONPIN : 2;
    bool R_FAKE;
    bool R_PORT_EN_IREF;
    bool R_PORT_EN_IMES;
    unsigned char R_PORT_EN : 2;

    // unsigned char TDC_MUPU_SPEED:2;      // Mandatory : 3
    // bool TDC_NOISE_DIS:1;                // Mandatory : 0
    // bool TDC_ALUPERMOPEN:1;              // Mandatory : 0
    // unsigned char TDC_CHAN_EN:2;         // Mandatory : 3

    // unsigned char TDC_MUPU_NO:6;         // Mandatory : 1

    // unsigned char TDC_NOISE_CY_DIS:1;    // Mandatory : 1
    // unsigned char TDC_QHA_SEL:6;         // Mandatory : 20

    bool PG0xPG2;
    bool PG1xPG3;
    unsigned char DSP_SPEED : 2;
    unsigned char DSP_MOFLO_EN : 2;

    unsigned char WD_DIS;

    unsigned char DSP_FF_IN : 4;
    unsigned char DSP_STARTONPIN : 4;

    unsigned char DSP_START_EN : 3;
    bool PG4_INTN_EN;
    bool PG5_INTN_EN;

    unsigned char PI0_CLK_SEL : 3;
    bool PI0_PDM_SEL;
    unsigned char PI0_RES : 2;
    bool PI0_TOGGLE_EN;
    bool PI1_TOGGLE_EN;

    unsigned char PI1_CLK_SEL : 3;
    unsigned char PI1_PDM_SEL : 1;
    unsigned char PI1_RES : 2;

    unsigned char PG_PU : 4;
    unsigned char PG_DIR_IN : 4;

    // unsigned char AMS_INT_REG:4;        // Mandatory : 7
    bool AUTOSTART;
    bool BG_PERM;
    bool DSP_TRIG_BG;
    bool INT_TRIG_BG;

    unsigned char CDC_GAIN_CORR;

    unsigned char BG_TIME : 8;

    unsigned char PULSE_SEL0 : 4;
    unsigned char PULSE_SEL1 : 4;

    unsigned char C_SENSE_SEL;

    unsigned char R_SENSE_SEL;

    bool C_MEDIAN_EN;
    bool R_MEDIAN_EN;
    bool HS_MODE_SEL;
    bool EN_ASYNC_READ;
    bool ALARM0_SELECT;
    bool ALARM1_SELECT;

    bool RUNBIT;

    unsigned char MEM_LOCK : 4;

    unsigned short SERIAL_NUMBER;

    unsigned char MEM_CTRL;

    // unsigned short CHARGE_PUMP;      // Not allowed to be changed
};
#ifdef PCAP_MEASUREMENT_MODE_STANDARD
struct pcap_results_t
{
#ifdef PCAP_REFERENCE_CAP_EXTERNAL
    float CREF_over_CInternalREF;
#else
    float C0_over_CREF;
#endif
    float C1_over_CREF;
    float C2_over_CREF;
    float C3_over_CREF;
    float C4_over_CREF;
    float C5_over_CREF;
    float PT1_over_PTREF;
    float PTInternal_over_PTREF;
}pcap_results_packed;
#endif
#ifdef PCAP_MEASUREMENT_MODE_STANDARD_FLOATING
struct pcap_results_t
{
#ifdef PCAP_REFERENCE_CAP_EXTERNAL
    float CREF_over_CInternalREF;
#else
    float C0_over_CREF;
#endif
    float C1_over_CREF;
    float C2_over_CREF;
    float PT1_over_PTREF;
    float PTInternal_over_PTREF;
}pcap_results_packed;
#endif  
#ifdef PCAP_MEASUREMENT_MODE_STANDARD_DIFFERENTIAL
struct pcap_results_t
{
    float C1_over_C0;
    float C3_over_C2;
    float C5_over_C6;
    float PT1_over_PTREF;
    float PTInternal_over_PTREF;
};
#endif  
#ifdef PCAP_MEASUREMENT_MODE_HUMIDITY
struct pcap_results_t
{
    float humidity;
    float temperature;
    float Ci_out;
    float R_out;
    float Xi_out;
    float Yi_out;
    float Pulse_Z;
    float Pulse_T;
}pcap_results_packed;
#endif  
#ifdef PCAP_MEASUREMENT_MODE_PRESSURE
struct pcap_results_t
{
    float pressure;
    float temperature;
    float Ci_out;
    float R_out;
    float Xi_out;
    float Yi_out;
    float Pulse_Z;
    float Pulse_T;
}pcap_results_packed;
#endif

struct pcap_status_t
{

    bool RUNBIT;
    bool CDC_ACTIVE;
    bool RDC_READY;
    bool AUTOBOOT_BUSY;
    bool POR_CDC_DSP_COLL;
    bool POR_FLAG_CONFIG;
    bool POR_FLAG_WDOG;
    
    unsigned char NC:4;
    bool RDC_ERR;
    bool MUP_ERR;
    bool ERR_OVERFL;
    bool COMB_ERR;

    bool C_PORT_ERR_INT;
    bool C_PORT_ERR0;
    bool C_PORT_ERR1;
    bool C_PORT_ERR2;
    bool C_PORT_ERR3;
    bool C_PORT_ERR4;
    bool C_PORT_ERR5;
};

struct pcap_data_vector_t{
    unsigned short nvram_loc;
    size_t size;
    unsigned char *data;
};


#endif
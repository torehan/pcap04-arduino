#ifndef pcap_defines_h
#define pcap_defines_h

#define PCAP_MEASUREMENT_MODE_STANDARD
// #define PCAP_MEASUREMENT_MODE_STANDARD_DIFFERENTIAL
// #define PCAP_MEASUREMENT_MODE_HUMIDITY
// #define PCAP_MEASUREMENT_MODE_PRESSURE

#define WR_NVRAM 0x28       // 'b:10 1000
#define RD_NVRAM 0x08       // 'b:00 1000
#define WR_CONFIG 0x028F    // 'b:10 1000 1111
#define RD_CONFIG 0x008F    // 'b:00 1000 1111
#define RD_RESULT 0x01      // 'b:01
#define POR_RESET 0x88      // 'b:1000 1000
#define INITIALIZE 0x8A     // 'b:1000 1010
#define CDC_START 0x8C      // 'b:1000 1100
#define RDC_START 0x8E      // 'b:1000 1110
#define DSP_TRIG 0x8D       // 'b:1000 1101
#define NV_STORE 0x96       // 'b:1001 0110
#define NV_RECALL 0x99      // 'b:1001 1001
#define NV_ERASE 0x9C       // 'b:1001 1100
#define TEST_READ_LOW 0x11  // 'b:1001 1100
#define TEST_READ_HIGH 0x7E // 'b:1001 1100

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)   \
    (byte & 0x80 ? '1' : '0'), \
    (byte & 0x40 ? '1' : '0'), \
    (byte & 0x20 ? '1' : '0'), \
    (byte & 0x10 ? '1' : '0'), \
    (byte & 0x08 ? '1' : '0'), \
    (byte & 0x04 ? '1' : '0'), \
    (byte & 0x02 ? '1' : '0'), \
    (byte & 0x01 ? '1' : '0')

#define GET_NUMBER_OF_DIGITS(i) \
    (i > 0 ? (int) log10 ((double) i) + 1 : 1)
#endif

#define PCAP_RESULTS_SIZE 35

#define PCAP_NVRAM_RUNBIT_INDEX 1007

#define PCAP_NVRAM_FW_SIZE 704
#define PCAP_NVRAM_FW_CAL0_SIZE 128
#define PCAP_NVRAM_FW_CAL1_SIZE 128
#define PCAP_NVRAM_CFG_SIZE 64
#define PCAP_NVRAM_SIZE (PCAP_NVRAM_FW_SIZE + PCAP_NVRAM_FW_CAL0_SIZE + PCAP_NVRAM_FW_CAL1_SIZE + PCAP_NVRAM_CFG_SIZE)

#define PCAP_NVRAM_MAX_INDEX_FW PCAP_NVRAM_FW_SIZE
#define PCAP_NVRAM_MAX_INDEX_FW_CAL0 (PCAP_NVRAM_FW_SIZE + PCAP_NVRAM_FW_CAL0_SIZE)
#define PCAP_NVRAM_MAX_INDEX_FW_CAL1 (PCAP_NVRAM_FW_SIZE + PCAP_NVRAM_FW_CAL0_SIZE + PCAP_NVRAM_FW_CAL1_SIZE)



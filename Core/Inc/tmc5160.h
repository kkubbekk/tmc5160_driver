#ifndef TMC5160_H_
#define TMC5160_H_

#include "main.h"
#include "stdbool.h"

#define I_RS 1.906

// 1. Status bitowy
typedef struct __attribute__((packed)) {
    uint8_t reset_flag     : 1;
    uint8_t driver_error   : 1;
    uint8_t stallguard     : 1;
    uint8_t standstill     : 1;
    uint8_t vel_reached    : 1;
    uint8_t pos_reached    : 1;
    uint8_t stop_l         : 1;
    uint8_t stop_r         : 1;
} TMC5160_Status_t;

typedef enum {
	NONE,
	WRITE,
	READ_PH1,
	READ_PH2,
} TMC5160_DMA_t;

typedef enum {
	    TMC_ERR_NONE = 0,             // Wszystko działa poprawnie
	    TMC_ERR_OVERTEMPERATURE,      //przegrzewanie
	    TMC_ERR_SHORT_TO_GROUND,      // zwarcie masy
	    TMC_ERR_OPEN_LOAD,            //odlaczony kabel
	    TMC_ERR_UNDER_VOLTAGE,        // spadek napiecia
	    TMC_ERR_STALL_DETECTED,        // silnik zablokowany
		TMC_ERR_SHORT_TO_SUPPLY,
		TMC_ERR_TIMEOUT
	} TMC5160_Error_t;


typedef enum {
	    TMC_STATE_UNINITIALIZED = 0,
		TMC_STATE_READY,
		TMC_STATE_ERROR,
		TMC_STATE_MOVING,
		TMC_STATE_CALIBRATING,
		TMC_STATE_TIMEOUT_ERROR,
	} TMC5160_State_t;

// 2. Uchwyt sterownika
typedef struct {
    SPI_TypeDef* spi_instance;
    GPIO_TypeDef* cs_port;
    uint16_t      cs_pin;

    TMC5160_Status_t status; //status 8bitowe info przy kazdej komendzie dociera 8bit
    TMC5160_Error_t  last_error; // ostatni blad
    TMC5160_State_t state; // w jakim stan


    uint32_t total_range_steps; // stepy z kalibracji 0-totalrange

    //dma
    SPI_HandleTypeDef* hspi;
    uint8_t            tx_buf[5];  // nadawczy dla DMA
    uint8_t            rx_buf[5];  //  odbiorczy dla DMA
    bool               is_busy;
    TMC5160_DMA_t	   dma_status;

} TMC5160_t;



// 3. Konfiguracja użytkownika
typedef struct {
    uint8_t  run_current; //prad podczas dzialania
    uint8_t  hold_current; //prad podzac spoczynku
    uint16_t microsteps; // ile mikrokrowko
    uint32_t acceleration; // zadnaie przyspieszena
    uint32_t max_velocity; // maksymalnej predkosci
} TMC5160_Config_t;

// 4. Rejestry
typedef enum {
    REG_GCONF      = 0x00,
    REG_GSTAT      = 0x01,
    REG_IHOLD_IRUN = 0x10,
    REG_RAMPMODE   = 0x20,
    REG_XACTUAL    = 0x21,
    REG_VMAX       = 0x27,
    REG_AMAX       = 0x26,
	REG_SW_MODE		=0x34,
    REG_CHOPCONF   = 0x6C,
	REG_TPOWERDWN  =0x11,
	REG_TPWMTHRS	=0x13,
	REG_COOLCONF   = 0x6D,
	REG_XTARGET    =0x2D,
	REG_DRV_STATUS   = 0X6F,
	REG_TCOOLTHRS =  0x14,
	REG_ENC			=0x38,
	REG_ENC_DEVIATION = 0x3D,
	REG_A1			=0x24,
	REG_V1			=0x25,
	REG_DMAX		=0x28,
	REG_D1 			=0x2A,
	REG_VSTOP		=0x2B,
	REG_TZEROWAIT  =0x2C


} TMC5160_Reg_t;

// Unia do ramek SPI
typedef union {
    struct __attribute__((packed)) {
        uint8_t  addr;
        uint32_t payload;
    } msg;
    uint8_t raw[5];
} TMC_Frame_t;





void TMC5160_Init(TMC5160_t *driver, TMC5160_Config_t *config);
void TMC5160_Write(TMC5160_t *driver, uint8_t reg, uint32_t value);
uint32_t TMC5160_Read(TMC5160_t *driver, uint8_t reg);

// Motion Control
void TMC5160_SetSpeedPercent(TMC5160_t *driver, uint8_t percent);
uint32_t TMC5160_ReadSPeedPercent(TMC5160_t *driver);
void TMC5160_set_target_percent(TMC5160_t *driver, int8_t percent);
int8_t TMC5160_read_target_percent(TMC5160_t *driver);

// Calibration & Features
void TMC5160_calibration_range(TMC5160_t *driver, int16_t sensitivity);
void TMC5160_EnableCoolStep(TMC5160_t *driver, uint32_t semin, uint32_t semax, uint32_t seimin);

// Current reading
float TMC5160_read_current_irun(TMC5160_t *driver);
float TMC5160_read_current_ihold(TMC5160_t *driver);

// Diagnostics (RTOS & Status)
int8_t TMC5160_RTOS_Quick_Check(TMC5160_t* driver);
void TMC5160_DIAGNOSTIC(TMC5160_t* driver);




#endif

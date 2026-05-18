#include "tmc5160.h"
#include <stdlib.h>

#define MAX_SAFE_VELOCITY 200000

/////////////////////////////debilu ////////////////////
//$$I_{RMS} = \frac{IRUN + 1}{32} \cdot \frac{V_{FS}}{R_{SENSE} + 0.02} \cdot \frac{1}{\sqrt{2}}$$
//popytaj sie elektrykow kogos kto ogarnia o ten wzor
//jaki prad i rezystancje ma ta plytka


static void TM_CS_Low(TMC5160_t *driver) {
    driver->cs_port->BSRR = (uint32_t)driver->cs_pin << 16U;
}

static void TM_CS_High(TMC5160_t *driver) {
    driver->cs_port->BSRR = driver->cs_pin;
}

static uint8_t SPI_Direct_Transfer(SPI_TypeDef* SPIx, uint8_t byte) {
    *(volatile uint8_t *)&SPIx->DR = byte;
    while(!(SPIx->SR & SPI_SR_RXNE));
    return *(volatile uint8_t *)&SPIx->DR;
}

void tm_send_data(TMC5160_t *driver, uint8_t *tx_ptr, uint8_t *rx_ptr_out) {
    uint8_t rx_local[5];
    TM_CS_Low(driver);
    for(int i = 0; i < 5; i++) {
        rx_local[i] = SPI_Direct_Transfer(driver->spi_instance, tx_ptr[i]);
        if (rx_ptr_out != NULL) rx_ptr_out[i] = rx_local[i];
    }
    while (driver->spi_instance->SR & SPI_SR_BSY);
    TM_CS_High(driver);
    driver->status = *(TMC5160_Status_t*)&rx_local[0];
}

// Public  API
void TMC5160_Write(TMC5160_t *driver, uint8_t reg, uint32_t value) {
    TMC_Frame_t tx;
    tx.msg.addr = reg | 0x80;
    tx.msg.payload = __REV(value);
    tm_send_data(driver, tx.raw, NULL);
}

uint32_t TMC5160_Read(TMC5160_t *driver, uint8_t reg) {
    TMC_Frame_t tx, rx;
    tx.msg.addr = reg & ~0x80;
    tx.msg.payload = 0;
    tm_send_data(driver, tx.raw, NULL);
    tm_send_data(driver, tx.raw, rx.raw);
    return __REV(rx.msg.payload);
}

void TMC5160_Init(TMC5160_t *driver, TMC5160_Config_t *config) {

	driver->state = TMC_STATE_CALIBRATING;

	uint32_t strzal_val = 12 | (12 << 8) | (6 << 16);
	    TMC5160_Write(driver, REG_IHOLD_IRUN, strzal_val);

	//to do: najpierw trzeba jabnac pradem 800ma w przetwornice spikiem a pozniej mozna obnizyc;


	HAL_Delay(75);

    TMC5160_Write(driver, REG_GCONF, 0x00000004);
    uint32_t current_val = (config->hold_current & 0x1F) |
                           ((config->run_current & 0x1F) << 8) |
                           (0x06 << 16);
//    musze dodac acceleretion do hamowania
//    REG_VSTART dodac i doczytac
//    zeby sie zatrzymac
//    TMC5160_Write(driver,REG)

    HAL_Delay(100);
    TMC5160_Write(driver, REG_IHOLD_IRUN, current_val);
    TMC5160_Write(driver, REG_AMAX, config->acceleration);
    TMC5160_Write(driver, REG_VMAX, config->max_velocity);
    TMC5160_Write(driver, REG_RAMPMODE, 0);


}

//publicc api




void TMC5160_calibration_range(TMC5160_t *driver,int16_t sensitivity){

	driver->state = TMC_STATE_CALIBRATING;
	//to do : moze zmniejszyc napiecie?
	//zmniejszam predkosc do kalibracji zeby niczego nie rozjebac
	TMC5160_SetSpeedPercent(driver,5);
	//uzytkownik dopbiera czulosc to sie przetestuje
	uint32_t value = (sensitivity & 0x7F) << 16;
	TMC5160_Write(driver,REG_COOLCONF,value);
	//zadaje nieskonczenie duzy target
	TMC5160_Write(driver,REG_XTARGET,-2000000000);
	//petla sprawdzajaca status reg_stallguarda

	while(1){
		uint32_t status = TMC5160_Read(driver, REG_DRV_STATUS);
		if(status & (1 <<24)){
			TMC5160_Write(driver, REG_VMAX, 0);
			break;
		}
	}
	TMC5160_Write(driver, REG_XACTUAL, 0);
	//tera w gore
	TMC5160_SetSpeedPercent(driver,5);

	//pauza zeby ogarnac co ijak
	HAL_Delay(100);

	TMC5160_Write(driver,REG_XTARGET,2000000000);
	while(1){
			uint32_t status = TMC5160_Read(driver, REG_DRV_STATUS);
			if(status & (1 <<24)){
				TMC5160_Write(driver, REG_VMAX, 0);
				break;
			}
		}

	HAL_Delay(100); // pauzy testowo narazie

	uint32_t max_dist = TMC5160_Read(driver, REG_XACTUAL);



	//wracamy na srodek
	TMC5160_SetSpeedPercent(driver,10);
	TMC5160_Write(driver,REG_XTARGET,max_dist/2);


	    driver->total_range_steps = max_dist;

		HAL_Delay(100);//


	driver->state = TMC_STATE_READY;
}

void TMC5160_set_target_percent(TMC5160_t *driver,int8_t percent){
	if (percent > 100) percent = 100;
	    if (percent < 0)   percent = 0;

	    uint32_t val = (uint32_t)(((uint64_t)percent * driver->total_range_steps) / 100);


	    TMC5160_Write(driver, REG_XTARGET, val);
	}

int8_t TMC5160_read_target_percent(TMC5160_t *driver){
	if (driver->state != TMC_STATE_READY) {
	        return -1;
	    }

	uint32_t target = TMC5160_Read(driver,REG_XTARGET);
	int8_t percent = ((100*target)/driver->total_range_steps);
	return percent;
}
float TMC5160_read_voltage_irun(TMC5160_t *driver){
	uint32_t data = TMC5160_Read(driver, REG_IHOLD_IRUN);
	data = (data >> 8) & 0x00ff;
//	prad znamionowy jest 1,906A wiec
	float current;
	current = (float)(data+1)/32.0f*I_RS;
	return current;
}


float TMC5160_read_voltage_ihold(TMC5160_t *driver){
	uint32_t data = TMC5160_Read(driver, REG_IHOLD_IRUN);
	data = data & 0x1f;
//	prad znamionowy jest 1,906A wiec
	float current;
	current = (float)(data+1)/32.0f*I_RS;
	return current;

}

void TMC5160_SetSpeedPercent(TMC5160_t *driver, uint8_t percent){
	if (driver->state == TMC_STATE_UNINITIALIZED || driver->state == TMC_STATE_ERROR) {
	        return;
	    }

	if(percent>100){percent = 100;}
	uint32_t val = (MAX_SAFE_VELOCITY*percent/100);
	TMC5160_Write(driver,REG_VMAX , val);

	driver->state = TMC_STATE_MOVING;
}
uint32_t TMC5160_ReadSPeedPercent(TMC5160_t *driver){
	uint32_t data = TMC5160_Read(driver,REG_VMAX);
	data = (100*data)/MAX_SAFE_VELOCITY;
	return data;
}

//----------------------------- cool step-----------------------------------------------------------
/* SEiMIN :minimum current for smart current control 0 for 1/2 of current setting(irun). 1: 1/4 of current setting (IRUN)
 * SEMAX: If the StallGuard2 result is equal to or above (SEMIN+SEMAX+1)*32, the motor current becomesdecreased to save energy. %0000 … %1111: 0 … 15
	SEMIN:If the StallGuard2 result falls below SEMIN*32, the motor
current becomes increased to reduce motor load angle.
%0000: smart current control CoolStep off
%0001 … %1111: 1 … 15


 */




void TMC5160_EnableCoolStep(TMC5160_t *driver,uint32_t semin,uint32_t semax,uint32_t seimin) {
  //odaplic coolstep i zad minimalny procent pod ktory
	uint32_t current_coolconf = TMC5160_Read(driver,REG_COOLCONF);
	current_coolconf &= 0xFFFF0000;


	uint32_t coolstep_config =((seimin & 0x01) << 15) | ((semax & 0x0F)<< 8) | (semin & 0x0F);
//	TMC5160_Write(driver, REG_TCOOLTHRS, tcoolthrs_speed);//trzbe jakos ustalic minimalne velocity na jakim to ma chodzic trzeba bedzie to obadac kiedys ale nejpierw to silnik musi ruszyc
//
	TMC5160_Write(driver, REG_COOLCONF, current_coolconf | coolstep_config);

}

//do rtos zeby w tasku pisac
int8_t TMC5160_RTOS_Quick_Check(TMC5160_t* driver) {
    TMC5160_Read(driver, REG_GCONF); // odswiezam status

    if (driver->status.driver_error) {
        driver->state = TMC_STATE_ERROR;
        return -1; // Coś się spaliło / przegrzało
    }

    if (driver->status.reset_flag) {
//        driver->state = TMC_STATE_UNINITIALIZED; nie jestem pewny narazie
        return 1; // Spadek napięcia
    }

    if (driver->status.stallguard) {
        // Silnik zablokowany mechanicznie
        return 2;
    }

    return 0; // gitarka wszystko smiga
}
void TMC5160_DIAGNOSTIC(TMC5160_t* driver){

	uint32_t drv_status = TMC5160_Read(driver,REG_DRV_STATUS);
	if(drv_status & (1 <<25)){
		driver->last_error = TMC_ERR_OVERTEMPERATURE;
	}
	else if(drv_status &(1 <<24)){
		driver->last_error = TMC_ERR_STALL_DETECTED;
	}
	else if(drv_status &(1 <<28) || drv_status &(1<<27)){
		driver-> last_error = TMC_ERR_SHORT_TO_GROUND;
	}
	else if(drv_status &(1<<29) || drv_status & (1<<30)){
		driver-> last_error = TMC_ERR_OPEN_LOAD;
	}
	else if ((drv_status & (1 << 12)) || (drv_status & (1 << 13))) {
	        driver->last_error = TMC_ERR_SHORT_TO_SUPPLY;
	    }
	else{
		driver -> last_error = TMC_ERR_NONE;
	}

}





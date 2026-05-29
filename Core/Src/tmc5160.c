#include "tmc5160.h"
#include <stdlib.h>

#define MAX_SAFE_VELOCITY 200000
#define TSTEP ((uint32_t)(16777216.0f / (MAX_SAFE_VELOCITY * 0.2f))) //2^24 wzor z dokumentacji

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
//	uint32_t timeout = HAL_GetTick() + 20;
    *(volatile uint8_t *)&SPIx->DR = byte;
    while(!(SPIx->SR & SPI_SR_RXNE))
    {
//    	if(HAL_GetTick()> timeout)
//    	{
//    	return -1;
//    	}
    };
    return *(volatile uint8_t *)&SPIx->DR;
}

void tm_send_data(TMC5160_t *driver, uint8_t *tx_ptr, uint8_t *rx_ptr_out) {
    uint8_t rx_local[5];
    TM_CS_Low(driver);
    for(int i = 0; i < 5; i++) {
        rx_local[i] = SPI_Direct_Transfer(driver->spi_instance, tx_ptr[i]);
//        if(rx_local[1]==-1)
//        {
//        	driver->state = TMC_STATE_TIMEOUT_ERROR;
//        	TM_CS_High(driver);
//        	return;
//        }
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

// w inicie napewno zrobic zeby te wartosci dobieral uzytkownik najprosciej chyba w define to zrobic czy cos;
void TMC5160_Init(TMC5160_t *driver, TMC5160_Config_t *config) {

	driver->state = TMC_STATE_CALIBRATING;



		    TMC5160_Write(driver, REG_GSTAT, 0x00000007); // GSTAT: Czyszczenie flag błędów po starcie zasilania
		    TMC5160_Write(driver, REG_SW_MODE, 0x00000000); // SW_MODE:  wyłączenie krańcówek!
		    TMC5160_Write(driver, REG_XACTUAL, 0x00000000); // XACTUAL: Zerowanie pozycji na starcie


		    TMC5160_Write(driver, REG_CHOPCONF, 0x000100C3); // CHOPCONF: Tryb SpreadCycle (rekomendacja z datasheeta)
		    TMC5160_Write(driver, REG_TPOWERDWN, 0x0000000A); // TPOWERDOWN: Czas do uśpienia silnika
		    TMC5160_Write(driver, REG_GCONF, 0x00000004); // GCONF: Włączenie trybu cichego (StealthChop)
		    TMC5160_Write(driver, REG_TPWMTHRS, 0x000001F4); // TPWM_THRS: Próg prędkości dla trybu cichego


		    uint32_t current_val = (config->hold_current & 0x1F) |
		                           ((config->run_current & 0x1F) << 8) |
		                           (0x06 << 16);
		    TMC5160_Write(driver, REG_IHOLD_IRUN, current_val);


		    TMC5160_Write(driver, REG_A1, 1000);                   // A1: Akceleracja startowa przyspieszenie na poczatku przy starcie najlepiej zeby bylo wieksze no bo trzeba rozzruszac
		    TMC5160_Write(driver, REG_V1, 50000);                  // V1: Próg dla AMAX ponkt odciecia dla akceleracji startowej przy ilu krokkach/s ma sie przelalczyc na to zwykle A
		    TMC5160_Write(driver, REG_AMAX, config->acceleration); // AMAX: Z Twojej konfiguracji glowne przyspieszenie
		    TMC5160_Write(driver, REG_VMAX, config->max_velocity); // VMAX: Z Twojej konfiguracji predkosc przeltowoa podczas przemieszczniaa sie jesli uklad sie do niej dobije no to potem porusza sieruchem jednostajnym
		    TMC5160_Write(driver, REG_DMAX, 700);                    // DMAX: Hamowanie główne przyspieszenia hamowania uklad sam wylicza kiedy ma zaczac hamowac
		    TMC5160_Write(driver, REG_D1, 1400);                   // D1: Hamowanie końcowe jak juz predkosc jest niska no to hamowanie wieksze szeby dorbze wycelowac w punmkt
		    TMC5160_Write(driver, REG_VSTOP, 10);                     // VSTOP: Minimalna prędkość zatrzymania
		    TMC5160_Write(driver, REG_RAMPMODE, 0);		 // tryb rampy 0 czyli pozycyjny se jedzi na target
		    TMC5160_Write(driver, REG_TCOOLTHRS, TSTEP);  // StallGuard od ~10k kroków/s tstep to czas pomiedzy krokiem wiec im nizszy ten szybciej trzeba sie krecic zeby byl stallgu zalecane 20% max velocity
		    TMC5160_Write(driver, REG_TZEROWAIT, 500);	//20ms jak sie zatrzyma do kolejngeo ruchu
		    driver->state = TMC_STATE_READY;
}

//publicc api



//dma_test--------------------------------------------------------------------------------------------------------------------------

static	TMC5160_t* current_driver_pointer = NULL; // pointer pod ktorego wsztrzykujemy w funckjach dma to spi na ktorym chcemy dzialac jak mamy dwa silniki czy cos kombinuje wiem ale staram sie napisac uniwersalna bilbioteke bo why not


bool TMC5160_dma_write(TMC5160_t* driver, uint8_t reg,uint32_t value){
	if(driver->is_busy ==false && (driver->state == TMC_STATE_MOVING || driver->state == TMC_STATE_READY))
		{

		driver->dma_status = WRITE;
		driver->is_busy=true;
		//chyba najlepiej zrobic jakis wskaznik statyczny pod ktory bede wstrzykiwac strukture drivera zeby wiecej niz jedno skrzydlo moglo dzialac?
		current_driver_pointer = driver;

		driver->tx_buf[0] = reg | 0x80;
		driver->tx_buf[1] = (value>>24) & 0xFF;
		driver->tx_buf[2] = (value>>16) & 0xFF;
		driver->tx_buf[3] = (value>>8) & 0xFF;
		driver->tx_buf[4] = value & 0xFF;
		//obnizamy linie cs;
		TM_CS_Low(driver);
		if(HAL_SPI_TransmitReceive_DMA(driver->hspi, driver->tx_buf, driver->rx_buf,5)!=HAL_OK)
		{
			TM_CS_High(driver);
			driver->is_busy=false;
			driver->dma_status = NONE;
			return false;
		}
		return true;
	}
	else
	{
	return false;
	}
}

bool TMC5160_Read_DMA_Start(TMC5160_t *driver, uint8_t reg) {
	if(driver->is_busy ==false && (driver->state == TMC_STATE_MOVING || driver->state == TMC_STATE_READY))
	{
		 	driver->is_busy = true;

		    driver->dma_status = READ_PH1;
		    current_driver_pointer = driver;


		    driver->tx_buf[0] = reg & ~0x80;
		    driver->tx_buf[1] = 0;
		    driver->tx_buf[2] = 0;
		    driver->tx_buf[3] = 0;
		    driver->tx_buf[4] = 0;

		    TM_CS_Low(driver);
		    if (HAL_SPI_TransmitReceive_DMA(driver->hspi, driver->tx_buf, driver->rx_buf, 5) != HAL_OK) {
		        TM_CS_High(driver);
		        driver->is_busy = false;
		        driver->dma_status = NONE;
		        return false;
		    }
		    return true;
	}
	else
	{
		return false;
	}
}

//-------------------------------------------calback co pinc bajtow --------------------------------------------------------------------

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {

}





void TMC5160_calibration_range(TMC5160_t *driver,int16_t sensitivity){
	uint32_t timeout = HAL_GetTick()+5000;
	driver->state = TMC_STATE_CALIBRATING;
	//to do : moze zmniejszyc napiecie?
	//zmniejszam predkosc do kalibracji zeby niczego nie rozwalic
	TMC5160_SetSpeedPercent(driver,5);
	//uzytkownik dopbiera czulosc to sie przetestuje
	uint32_t value = (sensitivity & 0x7F) << 16;
	TMC5160_Write(driver,REG_COOLCONF,value);
	//zadaje nieskonczenie duzy target
	TMC5160_Write(driver,REG_XTARGET,-2000000000);
	//petla sprawdzajaca status reg_stallguarda


	//!! ultra wazne dodac timeout zeby nie utknac w petli hal_get_tick czy cos
	while(1){
		uint32_t status = TMC5160_Read(driver, REG_DRV_STATUS);
		if(status & (1 <<24)){
			TMC5160_Write(driver, REG_VMAX, 0);
			break;
		}
		  if(HAL_GetTick() > timeout){
		        driver->state = TMC_STATE_ERROR;
		        TMC5160_Write(driver, REG_VMAX, 0);
		        driver->last_error = TMC_ERR_TIMEOUT; // jakis inny error timeout czy cos sie doda
		        return;
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
			  if(HAL_GetTick() > timeout){
			        driver->state = TMC_STATE_ERROR;
			        TMC5160_Write(driver, REG_VMAX, 0);
			        driver->last_error = TMC_ERR_TIMEOUT;
			        return;
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
float TMC5160_read_current_irun(TMC5160_t *driver){
	uint32_t data = TMC5160_Read(driver, REG_IHOLD_IRUN);
	data = (data >> 8) & 0x00ff;
//	prad znamionowy jest 1,906A wiec
	float current;
	current = (float)(data+1)/32.0f*I_RS;
	return current;
}


float TMC5160_read_current_ihold(TMC5160_t *driver){
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
//        driver->state = TMC_STATE_UNIALIZED; nie jestem pewny narazie
        return 1; // Spadek napięcia
    }

    if (driver->status.stallguard) {
        // Silnik zablokowany mechanicznie
        return 2;
    }

    return 0; // gitarka wszystko smiga
}

void TMC5160_DIAGNOSTIC(TMC5160_t* driver){

	    uint32_t s = TMC5160_Read(driver, REG_DRV_STATUS);

	    //fix this

	    if (s & (1 << 25))               driver->last_error = TMC_ERR_OVERTEMPERATURE;   // ot
	    else if (s & (1 << 26))          driver->last_error = TMC_ERR_OVERTEMPERATURE;   // otpw (warning)
	    else if (s & (1 << 24))          driver->last_error = TMC_ERR_STALL_DETECTED;    // StallGuard
	    else if (s & ((1<<28)|(1<<27)))  driver->last_error = TMC_ERR_SHORT_TO_GROUND;   // s2ga/s2gb
	    else if (s & ((1<<13)|(1<<12)))  driver->last_error = TMC_ERR_SHORT_TO_SUPPLY;   // s2vsa/s2vsb
	    else if (s & ((1<<30)|(1<<29)))  driver->last_error = TMC_ERR_OPEN_LOAD;         // olb/ola
	    else		                     driver->last_error = TMC_ERR_NONE;
}






//prototyp funckji krokowo enkoderowej enkoder mozna wpiac do silnika i wtedy pod rejestrem enc_daviation sam wypluwa zgubione kroki elegancko
//trzeba bedzie do innita troche dodac rzeczy str 45

//
//void TMC5160_Lost_Steps(TMC5160_t* driver){
//	//trzbea sprawdzac 0x3d
//
//}


// to do jak przyjdzie enkoder no to dodac funckje odzczytujaca zgubione kroki z reg enc_deviation




//dodac przerwania na krancowki ktore beda zatrzymyly nasz silniken czy cos jak zostana zrobione


// dobrac dobrze parametry (sensitivity na stallguardzie,ogolnie wszsystkie parametry po kolei zweryfikowac startujace tez,
// rozkiminic to kurcze blaszka)

//popytac o przelutowanie rezysotow


//pokombinowac z tym fsm zeby sie latwo debugowalo czy cos

///

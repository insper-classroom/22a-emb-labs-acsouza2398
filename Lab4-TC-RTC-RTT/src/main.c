#include <asf.h>

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

/************************************************************************/
/* defines                                                              */
/************************************************************************/

#define LED_PIO           PIOC                 // periferico que controla o LED
#define LED_PIO_ID        ID_PIOC                  // ID do periférico PIOC (controla LED)
#define LED_PIO_IDX       8                    // ID do LED no PIO
#define LED_PIO_IDX_MASK  (1 << LED_PIO_IDX)   // Mascara para CONTROLARMOS o LED

//Configuracoes dos LEDs do display
#define LED_ONE PIOA
#define LED_ONE_ID ID_PIOA
#define LED_ONE_IDX 0
#define LED_ONE_IDX_MASK (1u << LED_ONE_IDX)

#define LED_TWO PIOC
#define LED_TWO_ID ID_PIOC
#define LED_TWO_IDX 30
#define LED_TWO_IDX_MASK (1u << LED_TWO_IDX)

#define LED_THREE PIOB
#define LED_THREE_ID ID_PIOB
#define LED_THREE_IDX 2
#define LED_THREE_IDX_MASK (1u << LED_THREE_IDX)

// Configuracoes dos botoes do display
#define BUT_ONE	PIOD
#define BUT_ONE_ID ID_PIOD
#define BUT_ONE_IDX 28
#define BUT_ONE_IDX_MASK (1u << BUT_ONE_IDX) // esse já está pronto.

/************************************************************************/
/* variaveis globais                                                    */
/************************************************************************/

/************************************************************************/
/* prototypes                                                           */
/************************************************************************/

void init(void);
void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq);
void pin_toggle(Pio *pio, uint32_t mask);
static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource);

/************************************************************************/
/* Handlers                                                             */
/************************************************************************/

/**
*  Interrupt handler for TC1 interrupt.
*/
void TC1_Handler(void) {
	/**
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	* Isso é realizado pela leitura do status do periférico
	**/
	volatile uint32_t status = tc_get_status(TC0, 1);

	/** Muda o estado do LED (pisca) **/
	pin_toggle(LED_PIO, LED_PIO_IDX_MASK);  
}

void TC2_Handler(void) {
	/**
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	* Isso é realizado pela leitura do status do periférico
	**/
	volatile uint32_t status = tc_get_status(TC0, 2);

	/** Muda o estado do LED (pisca) **/
	pin_toggle(LED_ONE, LED_ONE_IDX_MASK);  
}

void RTT_Handler(void) {
	uint32_t ul_status;

	/* Get RTT status - ACK */
	ul_status = rtt_get_status(RTT);

	/* IRQ due to Alarm */
	if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
		RTT_init(0, 4, RTT_MR_RTTINCIEN);
	}
	
	/* IRQ due to Time has changed */
	if ((ul_status & RTT_SR_RTTINC) == RTT_SR_RTTINC) {
		pin_toggle(LED_TWO, LED_TWO_IDX_MASK);    // BLINK Led
	}

}


/************************************************************************/
/* Funcoes                                                              */
/************************************************************************/

/**
* @Brief Inicializa o pino do LED
*/
void LED_init(int estado, Pio *pio, int id, uint32_t mask) {
	pmc_enable_periph_clk(id);
	pio_set_output(pio, mask, estado, 0, 0);
};

/**
* @Brief Inverte o valor do pino 0->1/ 1->0
*/
void pin_toggle(Pio *pio, uint32_t mask) {
	if(pio_get_output_data_status(pio, mask))
	pio_clear(pio, mask);
	else
	pio_set(pio,mask);
}

// Função de inicialização do uC
void init(void){
	// Initialize the board clock
	sysclk_init();

	// Desativa WatchDog Timer
	WDT->WDT_MR = WDT_MR_WDDIS;	
	
	// Inicializa dos botoes do display;
	pmc_enable_periph_clk(BUT_ONE_ID);
	
	pio_set_input(BUT_ONE, BUT_ONE_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);

}
;

static float get_time_rtt(){
	uint ul_previous_time = rtt_read_timer_value(RTT);
}

void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq){
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();

	uint32_t channel = 1;

	/* Configura o PMC */
	/* O TimerCounter é meio confuso
	o uC possui 3 TCs, cada TC possui 3 canais
	TC0 : ID_TC0, ID_TC1, ID_TC2
	TC1 : ID_TC3, ID_TC4, ID_TC5
	TC2 : ID_TC6, ID_TC7, ID_TC8
	*/
	pmc_enable_periph_clk(ID_TC);

	/** Configura o TC para operar em  freq hz e interrupçcão no RC compare */
	tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	tc_init(TC, TC_CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq);

	/* Configura e ativa interrupçcão no TC canal 0 */
	/* Interrupção no C */
	NVIC_EnableIRQ((IRQn_Type) ID_TC);
	tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS);
}

static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource) {

	uint16_t pllPreScale = (int) (((float) 32768) / freqPrescale);
	
	rtt_sel_source(RTT, false);
	rtt_init(RTT, pllPreScale);
	
	if (rttIRQSource & RTT_MR_ALMIEN) {
		uint32_t ul_previous_time;
		ul_previous_time = rtt_read_timer_value(RTT);
		while (ul_previous_time == rtt_read_timer_value(RTT));
		rtt_write_alarm_time(RTT, IrqNPulses+ul_previous_time);
	}

	/* config NVIC */
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, 4);
	NVIC_EnableIRQ(RTT_IRQn);

	/* Enable RTT interrupt */
	if (rttIRQSource & (RTT_MR_RTTINCIEN | RTT_MR_ALMIEN))
	rtt_enable_interrupt(RTT, rttIRQSource);
	else
	rtt_disable_interrupt(RTT, RTT_MR_RTTINCIEN | RTT_MR_ALMIEN);
	
}

/************************************************************************/
/* Main                                                                 */
/************************************************************************/

int main (void)
{
	board_init();
	sysclk_init();
	delay_init();
	init();

	/* Configura Leds */
	LED_init(0,LED_PIO,LED_PIO_ID,LED_PIO_IDX_MASK);
	LED_init(0,LED_ONE,LED_ONE_ID,LED_ONE_IDX_MASK);
	LED_init(0,LED_TWO,LED_TWO_ID,LED_TWO_IDX_MASK);
	
	 /** Configura timer TC0, canal 1 */
	 TC_init(TC0, ID_TC1, 1, 5);
	 TC_init(TC0, ID_TC2, 2, 4);
	 
	 tc_start(TC0, 1);
	 tc_start(TC0, 2);  
	 
	 RTT_init(4, 16, RTT_MR_ALMIEN);

  /* Insert application code here, after the board has been initialized. */
	while(1) {
		pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
	}
}

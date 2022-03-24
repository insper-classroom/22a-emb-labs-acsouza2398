#include <asf.h>

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

/************************************************************************/
/* defines                                                              */
/************************************************************************/

#define ECHO_PIO PIOD
#define ECHO_PIO_ID ID_PIOD
#define ECHO_PIO_IDX 11
#define ECHO_PIO_IDX_MASK (1u << ECHO_PIO_IDX)

#define TRIG_PIO PIOC
#define TRIG_PIO_ID ID_PIOC
#define TRIG_PIO_IDX 13
#define TRIG_PIO_IDX_MASK (1u << TRIG_PIO_IDX)

/************************************************************************/
/* variaveis globais                                                    */
/************************************************************************/

volatile char end_flag = 0;
volatile char f_tempo = 0;

volatile uint32_t tempo_anterior = 0;
volatile uint32_t tempo = 0;
volatile uint32_t inc = 0;

volatile 	char str[32];

/************************************************************************/
/* prototypes                                                           */
/************************************************************************/

void init(void);
static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource);
void gfx_mono_draw_string(const char *str, const gfx_coord_t x, const gfx_coord_t y, const struct font *font);
void echo_callback(void);

/************************************************************************/
/* Handlers                                                             */
/************************************************************************/

void rtt_reset(Rtt *p_rtt){
	p_rtt->RTT_MR = p_rtt->RTT_MR | RTT_MR_RTTRST;
}

void echo_callback(void){
	if (pio_get(ECHO_PIO, PIO_INPUT, ECHO_PIO_IDX_MASK)) {
		//RTT_init(1000, 14, 0);
		//rtt_reset(RTT);
	} else {
		uint32_t tempo_atual = rtt_read_timer_value(RTT);
		
		tempo = tempo_atual - tempo_anterior;
		tempo_anterior = tempo_atual;
		f_tempo = 1;

		
	}
};

void RTT_Handler(void) {
	uint32_t ul_status;

	/* Get RTT status - ACK */
	ul_status = rtt_get_status(RTT);

	/* IRQ due to Alarm */
	if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
		
	}
	
	/* IRQ due to Time has changed */
	if ((ul_status & RTT_SR_RTTINC) == RTT_SR_RTTINC) {
		inc++;
	}

}

/************************************************************************/
/* Funcoes                                                              */
/************************************************************************/

// Função de inicialização do uC
void init(void){
	// Initialize the board clock
	sysclk_init();

	// Desativa WatchDog Timer
	WDT->WDT_MR = WDT_MR_WDDIS;
	
	pio_configure(ECHO_PIO, PIO_INPUT, ECHO_PIO_IDX_MASK, PIO_DEBOUNCE); //tipo botao
	pio_configure(TRIG_PIO, PIO_OUTPUT_0, TRIG_PIO_IDX_MASK, PIO_DEBOUNCE); //tipo led
	
	pio_handler_set(ECHO_PIO,
	ECHO_PIO_ID,
	ECHO_PIO_IDX_MASK,
	PIO_IT_EDGE,
	echo_callback);
	
	pio_enable_interrupt(ECHO_PIO, ECHO_PIO_IDX_MASK);
	pio_get_interrupt_status(ECHO_PIO);
	
	NVIC_EnableIRQ(ECHO_PIO_ID);
	NVIC_SetPriority(ECHO_PIO_ID, 4); // Prioridade 4
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

int main (void)
{
	float dist = 0;
	
	
	board_init();
	sysclk_init();
	delay_init();
	init();
	RTT_init(1000.0, 14, 0);


	// Init OLED
	gfx_mono_ssd1306_init();
	
	//sprintf(str, "%6.0lf", dist);
	//gfx_mono_draw_string(str, 0, 0, &sysfont);
	
	delay_s(1);
	int i = 0 ;
	
	/* Insert application code here, after the board has been initialized. */
	while(1) {
			delay_ms(100);

		if(end_flag == 0){
			pio_set(TRIG_PIO,TRIG_PIO_IDX_MASK);
			delay_us(10);
			pio_clear(TRIG_PIO,TRIG_PIO_IDX_MASK);
			end_flag = 1;
		}
		
		
		if (f_tempo) {
			i++;
			dist = tempo/2 * 340;
			sprintf(str, "%02d / %05d", inc, tempo);
			gfx_mono_draw_string(str, 0, 0, &sysfont);
			end_flag = 0;
			f_tempo = 0;
		}
	}
}

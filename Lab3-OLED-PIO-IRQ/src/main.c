#include <asf.h>

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

// Configuracoes dos botoes do display
#define BUT_ONE	PIOD
#define BUT_ONE_ID ID_PIOD
#define BUT_ONE_IDX 28
#define BUT_ONE_IDX_MASK (1u << BUT_ONE_IDX) // esse já está pronto.

#define BUT_TWO	PIOC
#define BUT_TWO_ID ID_PIOC
#define BUT_TWO_IDX 31
#define BUT_TWO_IDX_MASK (1u << BUT_TWO_IDX) // esse já está pronto.

#define BUT_THREE	PIOA
#define BUT_THREE_ID ID_PIOA
#define BUT_THREE_IDX 19
#define BUT_THREE_IDX_MASK (1u << BUT_THREE_IDX) // esse já está pronto.

//Configuracoes dos LEDs do display
#define LED_ONE PIOA
#define LED_ONE_ID ID_PIOA
#define LED_ONE_IDX 0
#define LED_ONE_IDX_MASK (1u << LED_ONE_IDX)

volatile char but_flag = 0;
volatile char stop = 0;
volatile char but_flag_dim = 0;

void gfx_mono_draw_string(const char *str, const gfx_coord_t x, const gfx_coord_t y, const struct font *font);
void gfx_mono_generic_draw_filled_rect	(	gfx_coord_t 	x,
gfx_coord_t 	y,
gfx_coord_t 	width,
gfx_coord_t 	height,
enum gfx_mono_color 	color
);

void but_one_callback(void){
	if (pio_get(BUT_ONE, PIO_INPUT, BUT_ONE_IDX_MASK)) {
		but_flag = 0;
	} else {
		but_flag = 1;
	}
}

void but_two_callback(void){
	if(stop == 0){
		stop = 1;
	} else{
		stop = 0;
	}
}

void but_three_callback(void){
	but_flag_dim = 1;
}

void pisca_led(int n, int t){
	for (int i=0;i<=n;i++){
		pio_clear(LED_ONE, LED_ONE_IDX_MASK);
		delay_us(t/2);
		if(but_flag == 1 || stop == 1) break;
		pio_set(LED_ONE, LED_ONE_IDX_MASK);
		delay_us(t/2);
		if(but_flag == 1 || stop == 1) break;
 		gfx_mono_generic_draw_filled_rect(10,20,30,10, 0);
		gfx_mono_generic_draw_filled_rect(10,20,n-i,10, 1);
	}
}

void io_init(void)
{

	// Configura led
	pmc_enable_periph_clk(LED_ONE_ID);
	pio_configure(LED_ONE, PIO_OUTPUT_1, LED_ONE_IDX_MASK, PIO_DEFAULT);

	// Inicializa clock do periférico PIO responsavel pelo botao
	pmc_enable_periph_clk(BUT_ONE_ID);
	pmc_enable_periph_clk(BUT_TWO_ID);
	pmc_enable_periph_clk(BUT_THREE_ID);

	// Configura PIO para lidar com o pino do botão como entrada
	// com pull-up
	pio_configure(BUT_ONE, PIO_INPUT, BUT_ONE_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT_ONE, BUT_ONE_IDX_MASK, 60);
	
	pio_configure(BUT_TWO, PIO_INPUT, BUT_TWO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT_TWO, BUT_TWO_IDX_MASK, 60);
	
	pio_configure(BUT_THREE, PIO_INPUT, BUT_THREE_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT_THREE, BUT_THREE_IDX_MASK, 60);


	// Configura interrupção no pino referente ao botao e associa
	// função de callback caso uma interrupção for gerada
	// a função de callback é a: but_callback()
	pio_handler_set(BUT_ONE,
	BUT_ONE_ID,
	BUT_ONE_IDX_MASK,
	PIO_IT_EDGE,
	but_one_callback);
	
	pio_handler_set(BUT_TWO,
	BUT_TWO_ID,
	BUT_TWO_IDX_MASK,
	PIO_IT_RISE_EDGE,
	but_two_callback);
	
	pio_handler_set(BUT_THREE,
	BUT_THREE_ID,
	BUT_THREE_IDX_MASK,
	PIO_IT_RISE_EDGE,
	but_three_callback);

	// Ativa interrupção e limpa primeira IRQ gerada na ativacao
	pio_enable_interrupt(BUT_ONE, BUT_ONE_IDX_MASK);
	pio_get_interrupt_status(BUT_ONE);
	
	pio_enable_interrupt(BUT_TWO, BUT_TWO_IDX_MASK);
	pio_get_interrupt_status(BUT_TWO);
	
	pio_enable_interrupt(BUT_THREE, BUT_THREE_IDX_MASK);
	pio_get_interrupt_status(BUT_THREE);
	
	// Configura NVIC para receber interrupcoes do PIO do botao
	// com prioridade 4 (quanto mais próximo de 0 maior)
	NVIC_EnableIRQ(BUT_ONE_ID);
	NVIC_SetPriority(BUT_ONE_ID, 4); // Prioridade 4
	
	NVIC_EnableIRQ(BUT_TWO_ID);
	NVIC_SetPriority(BUT_TWO_ID, 4); // Prioridade 4
	
	NVIC_EnableIRQ(BUT_THREE_ID);
	NVIC_SetPriority(BUT_THREE_ID, 4); // Prioridade 4
}


int main (void)
{
	double freq = 1000;
	double tempo =  1000 / (1.0/freq);
	double count = 0;
	char str[10];
	
	board_init();
	sysclk_init();
	delay_init();
	io_init();

  // Init OLED
	gfx_mono_ssd1306_init();
	
	sprintf(str, "%6.0lf", freq); //
	gfx_mono_draw_string(str, 0, 0, &sysfont);

  /* Insert application code here, after the board has been initialized. */
	while(1) {		
		sprintf(str, "%6.0lf", freq);
		gfx_mono_draw_string(str, 0, 0, &sysfont);
		
		if(but_flag == 1){
			delay_ms(300);
			if(but_flag == 1)
				freq += 100;
			else 
				freq -= 100;
			sprintf(str, "%6.0lf", freq);
			gfx_mono_draw_string(str, 0, 0, &sysfont);
			tempo = 1000 / (1/freq);	
		}
		
		if(but_flag_dim == 1){
			freq -= 100;
			but_flag_dim = 0;
		}
		
		if(stop == 0){
			int n = 30;
			pisca_led(n,tempo);
			gfx_mono_generic_draw_filled_rect(10,20,30,10, 0);
			pio_set(LED_ONE, LED_ONE_IDX_MASK);
		} else {
			if(but_flag == 0){
				pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
			}
		}
		
	}
	return 0;
}

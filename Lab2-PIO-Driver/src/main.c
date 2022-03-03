/**
 * 5 semestre - Eng. da Computação - Insper
 * Rafael Corsi - rafael.corsi@insper.edu.br
 *
 * Projeto 0 para a placa SAME70-XPLD
 *
 * Objetivo :
 *  - Introduzir ASF e HAL
 *  - Configuracao de clock
 *  - Configuracao pino In/Out
 *
 * Material :
 *  - Kit: ATMEL SAME70-XPLD - ARM CORTEX M7
 */

/************************************************************************/
/* includes                                                             */
/************************************************************************/

#include "asf.h"

/************************************************************************/
/* defines                                                              */
/************************************************************************/

#define LED_PIO           PIOC                 // periferico que controla o LED
// #
#define LED_PIO_ID        ID_PIOC                  // ID do periférico PIOC (controla LED)
#define LED_PIO_IDX       8                    // ID do LED no PIO
#define LED_PIO_IDX_MASK  (1 << LED_PIO_IDX)   // Mascara para CONTROLARMOS o LED  

// Configuracoes do botao
#define BUT_PIO	PIOA
#define BUT_PIO_ID ID_PIOA
#define BUT_PIO_IDX 11
#define BUT_PIO_IDX_MASK (1u << BUT_PIO_IDX) // esse já está pronto.

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

#define BUT_TWO	PIOC
#define BUT_TWO_ID ID_PIOC
#define BUT_TWO_IDX 31
#define BUT_TWO_IDX_MASK (1u << BUT_TWO_IDX) // esse já está pronto.

#define BUT_THREE	PIOA
#define BUT_THREE_ID ID_PIOA
#define BUT_THREE_IDX 19
#define BUT_THREE_IDX_MASK (1u << BUT_THREE_IDX) // esse já está pronto.

/*  Default pin configuration (no attribute). */
#define _PIO_DEFAULT             (0u << 0)
/*  The internal pin pull-up is active. */
#define _PIO_PULLUP              (1u << 0)
/*  The internal glitch filter is active. */
#define _PIO_DEGLITCH            (1u << 1)
/*  The internal debouncing filter is active. */
#define _PIO_DEBOUNCE            (1u << 3)



/************************************************************************/
/* constants                                                            */
/************************************************************************/

/************************************************************************/
/* variaveis globais                                                    */
/************************************************************************/

/************************************************************************/
/* prototypes                                                           */
/************************************************************************/

void init(void);

/************************************************************************/
/* interrupcoes                                                         */
/************************************************************************/

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

void _pio_set(Pio *p_pio, const uint32_t ul_mask)
{
	p_pio->PIO_SODR = ul_mask;
}

void _pio_clear(Pio *p_pio, const uint32_t ul_mask)
{
	p_pio->PIO_CODR = ul_mask;
}

void _pio_pull_up(Pio *p_pio, const uint32_t ul_mask, const uint32_t ul_pull_up_enable)
{
	p_pio->PIO_PUER = ul_mask;
	
}

void _pio_set_input(Pio *p_pio, const uint32_t ul_mask, const uint32_t ul_attribute)
{
	if(ul_attribute & _PIO_PULLUP){
		_pio_pull_up(p_pio, ul_mask, _PIO_PULLUP);
	}
	
	if(ul_attribute & _PIO_DEGLITCH){
		p_pio->PIO_IFSCDR = ul_mask;
	}
	
	if(ul_attribute & _PIO_DEBOUNCE){
		p_pio->PIO_IFSCER = ul_mask;
	}
}

void _pio_set_output(Pio *p_pio, const uint32_t ul_mask, const uint32_t ul_default_level, const uint32_t ul_multidrive_enable, const uint32_t ul_pull_up_enable)
{
	p_pio->PIO_PER = ul_mask;
	p_pio->PIO_OER = ul_mask;
	
	if(ul_default_level){
		_pio_set(p_pio,ul_mask);
	} else{
		_pio_clear(p_pio,ul_mask);
	}
	
	if(ul_multidrive_enable){
		p_pio->PIO_MDER = ul_mask;
	}
	
	if(ul_pull_up_enable){
		_pio_pull_up(p_pio, ul_mask, ul_pull_up_enable);
	}
	
}

uint32_t _pio_get(Pio *p_pio, const pio_type_t ul_type, const uint32_t ul_mask){
	
	if(ul_type == PIO_INPUT){
		if((p_pio->PIO_PDSR & ul_mask) == 0){
			return 0;
		} else{
			return 1;
		}
	} else if(ul_type == PIO_OUTPUT_0){
		if((p_pio->PIO_ODSR & ul_mask) == 0){
			return 0;
		} else{
			return 1;
		}
	}
}

void _delay_ms(int ms){
	for (int i = 0; i <= ms*25; i++){
		for (int j = 0; j <= ms*25; j++);
	}
}

// Função de inicialização do uC
void init(void){
	// Initialize the board clock
	sysclk_init();

	// Desativa WatchDog Timer
	WDT->WDT_MR = WDT_MR_WDDIS;
	
	// Ativa o PIO na qual o LED foi conectado
	// para que possamos controlar o LED.
	pmc_enable_periph_clk(LED_PIO_ID);
	
	// Inicializa PIO do botao
	pmc_enable_periph_clk(BUT_PIO_ID);
	
	// Inicializa dos botoes do display;
	pmc_enable_periph_clk(BUT_ONE_ID);
	pmc_enable_periph_clk(BUT_TWO_ID);
	pmc_enable_periph_clk(BUT_THREE_ID);
	
	_pio_set_input(BUT_ONE, BUT_ONE_IDX_MASK, _PIO_PULLUP | _PIO_DEBOUNCE);
	_pio_set_input(BUT_TWO, BUT_TWO_IDX_MASK, _PIO_PULLUP | _PIO_DEBOUNCE);
	_pio_set_input(BUT_THREE, BUT_THREE_IDX_MASK, _PIO_PULLUP | _PIO_DEBOUNCE);
	
	
	// Inicializa os LEDs do display
	pmc_enable_periph_clk(LED_ONE_ID);
	pmc_enable_periph_clk(LED_TWO_ID);
	pmc_enable_periph_clk(LED_THREE_ID);
	
	_pio_set_output(LED_ONE, LED_ONE_IDX_MASK, 1, 0, 0);
	_pio_set_output(LED_TWO, LED_TWO_IDX_MASK, 1, 0, 0);
	_pio_set_output(LED_THREE, LED_THREE_IDX_MASK, 1, 0, 0);
	
	// configura pino ligado ao botão como entrada com um pull-up.
	_pio_set_input(BUT_PIO, BUT_PIO_IDX_MASK, PIO_DEFAULT);
	
	//Inicializa PC8 como saída
	_pio_set_output(LED_PIO, LED_PIO_IDX_MASK, 1, 0, 0);
	
	_pio_set_input(BUT_PIO, BUT_PIO_IDX_MASK, _PIO_PULLUP | _PIO_DEBOUNCE);
	

}
;
/************************************************************************/
/* Main                                                                 */
/************************************************************************/

// Funcao principal chamada na inicalizacao do uC.
int main(void)
{	
	init();

  // super loop
  // aplicacoes embarcadas não devem sair do while(1).
	while (1){
	if(!_pio_get(BUT_PIO,PIO_INPUT,BUT_PIO_IDX_MASK)){
		for(int i = 0; i < 5; i++){
			_pio_clear(PIOC, LED_PIO_IDX_MASK);      // Coloca 1 no pino LED
			_delay_ms(100);                        // Delay por software de 200 ms
			_pio_set(PIOC, LED_PIO_IDX_MASK);    // Coloca 0 no pino do LED
			_delay_ms(300);                        // Delay por software de 200 ms
		}
	}
	
	
	if(!_pio_get(BUT_ONE,PIO_INPUT,BUT_ONE_IDX_MASK)){
		for(int i = 0; i < 5; i++){
			_pio_clear(PIOA, LED_ONE_IDX_MASK);
			_delay_ms(100);
			_pio_set(PIOA, LED_ONE_IDX_MASK);
			_delay_ms(300); 
		}
	} 
	
	
	if(!_pio_get(BUT_TWO,PIO_INPUT,BUT_TWO_IDX_MASK)){
		for(int i = 0; i < 5; i++){
			_pio_clear(PIOC, LED_TWO_IDX_MASK);
			_delay_ms(100);
			_pio_set(PIOC, LED_TWO_IDX_MASK);
			_delay_ms(300); 
		}
	} 
	
	
	  if(!_pio_get(BUT_THREE,PIO_INPUT,BUT_THREE_IDX_MASK)){
		for(int i = 0; i < 5; i++){
		  _pio_clear(PIOB, LED_THREE_IDX_MASK);
		  _delay_ms(100);
		  _pio_set(PIOB, LED_THREE_IDX_MASK);
		  _delay_ms(300); 
		}
	  } 
	  
	  
  }
  return 0;
}

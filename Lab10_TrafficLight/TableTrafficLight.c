// ***** 0. Documentation Section *****
// TableTrafficLight.c for Lab 10
// Runs on LM4F120/TM4C123
// Index implementation of a Moore finite state machine to operate a traffic light.  
// Daniel Valvano, Jonathan Valvano
// January 15, 2016

// east/west red light connected to PB5
// east/west yellow light connected to PB4
// east/west green light connected to PB3
// north/south facing red light connected to PB2
// north/south facing yellow light connected to PB1
// north/south facing green light connected to PB0
// pedestrian detector connected to PE2 (1=pedestrian present)
// north/south car detector connected to PE1 (1=car present)
// east/west car detector connected to PE0 (1=car present)
// "walk" light connected to PF3 (built-in green LED)
// "don't walk" light connected to PF1 (built-in red LED)

// ***** 1. Pre-processor Directives Section *****
#include "TExaS.h"
#include "tm4c123gh6pm.h"

// ***** 2. Global Declarations Section *****
#define LIGHT                   (*((volatile unsigned long *)0x400050FC))
#define SWITCH                  (*((volatile unsigned long *)0x4002401C))
#define WALK										(*((volatile unsigned long *)0x40025028))
#define NVIC_ST_CTRL_R      (*((volatile unsigned long *)0xE000E010))
#define NVIC_ST_RELOAD_R    (*((volatile unsigned long *)0xE000E014))
#define NVIC_ST_CURRENT_R   (*((volatile unsigned long *)0xE000E018))
void Port_Init(void);
void SysTick_Init(void);
void SysTick_Wait(unsigned long delay);
void SysTick_Wait10ms(unsigned long delay);	
void SysTick_Init(void){
  NVIC_ST_CTRL_R = 0;               // disable SysTick during setup
  NVIC_ST_CTRL_R = 0x00000005;      // enable SysTick with core clock
}
// The delay parameter is in units of the 80 MHz core clock. (12.5 ns)
void SysTick_Wait(unsigned long delay){
  NVIC_ST_RELOAD_R = delay-1;  // number of counts to wait
  NVIC_ST_CURRENT_R = 0;       // any value written to CURRENT clears
  while((NVIC_ST_CTRL_R&0x00010000)==0){ // wait for count flag
  }
}
// 10000us equals 10ms
void SysTick_Wait10ms(unsigned long delay){
  unsigned long i;
  for(i=0; i<delay; i++){
    SysTick_Wait(800000);  // wait 10ms
  }
}

void Port_Init(void){
	volatile unsigned long delay;
  SYSCTL_RCGC2_R |= 0x32;     // B E F clock
  delay = SYSCTL_RCGC2_R; 	// delay 

  //PORT_F INIT
	GPIO_PORTF_PCTL_R = 0x00;       //  PCTL GPIO on PF3, PF1
  GPIO_PORTF_DIR_R |= 0x0A;       //  PF3, PF1 are outputs
  GPIO_PORTF_AFSEL_R = 0x00;      //  disable alternate function
  GPIO_PORTF_PUR_R = 0x00;        // disable pull-up resistor
  GPIO_PORTF_DEN_R |= 0x0A;       //  enable digital I/O on PF3, PF1
	
	//PORT_B INIT
	GPIO_PORTB_AMSEL_R &= ~0x3F; // disable analog function on PB5-0
  GPIO_PORTB_PCTL_R &= ~0x00FFFFFF; //  enable regular GPIO
  GPIO_PORTB_DIR_R |= 0x3F;    // outputs on PB5-0
  GPIO_PORTB_AFSEL_R &= ~0x3F; // regular function on PB5-0
  GPIO_PORTB_DEN_R |= 0x3F;    // enable digital on PB5-0	
	
	//PORT_E INIT
	GPIO_PORTE_AMSEL_R &= ~0x07; // disable analog function on PE1-0
  GPIO_PORTE_PCTL_R &= ~0x000000FF; // enable regular GPIO
  GPIO_PORTE_DIR_R &= ~0x07;   //  inputs on PE1-0
  GPIO_PORTE_AFSEL_R &= ~0x07; //  regular function on PE1-0
  GPIO_PORTE_DEN_R |= 0x07;    //  enable digital on PE1-2
}
// FUNCTION PROTOTYPES: Each subroutine defined
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts

// ***** 3. Subroutines Section *****
struct State{
  unsigned long Out;
	unsigned long time;
	unsigned long out2;
	unsigned long Next[8];
};
typedef const struct State STyp;
#define go_w				0
#define wait_w			1
#define go_s				2	
#define wait_s			3
#define walk_grn		4
#define	walk_red		5
#define	walk_off		6
#define	walk_r2			7
#define	walk_off2		8

#define OFF 0
#define RED 2
#define GREEN 8
STyp FSM[9]={
 {0x21,200,RED,{go_w,go_w,wait_w,wait_w,wait_w,wait_w,wait_w,wait_w}},
 {0x22,100,RED,{go_s,go_w,go_s,go_s,walk_grn,go_w,walk_grn,go_w}},
 {0x0C,200,RED,{go_s,wait_s,go_s,wait_s,wait_s,wait_s,wait_s,wait_s}},
 {0x14,100,RED,{go_w,go_w,go_s,go_w,walk_grn,walk_grn,go_s,walk_grn}},
 {0x24,50,GREEN,{walk_r2,walk_r2,walk_r2,walk_r2,walk_grn,walk_r2,walk_r2,walk_off}},
 {0x24,50,RED,{walk_off,walk_off,walk_off,walk_off,walk_off,walk_off,walk_off,go_s}},
 {0x24,50,OFF,{wait_s,wait_w,wait_s,wait_w,walk_grn,wait_w,wait_s,walk_r2}},
 {0x24,50,RED,{walk_off2,walk_off2,walk_off2,walk_off2,walk_off2,walk_off2,walk_off2,walk_off2}},
 {0x24,50,OFF,{walk_red,walk_red,walk_red,walk_red,walk_grn,walk_red,walk_red,walk_red}}	 
};

unsigned long current;
unsigned long input;

int main(void){ 
  TExaS_Init(SW_PIN_PE210, LED_PIN_PB543210,ScopeOff); // activate grader and set system clock to 80 MHz
 SysTick_Init(); //Timer init
	Port_Init();  //Ports B,E,F init
	current=go_w;
  
  EnableInterrupts();
  while(1){
		LIGHT=FSM[current].Out;
		WALK=FSM[current].out2;
		SysTick_Wait10ms(FSM[current].time);
		input=SWITCH;
		current=FSM[current].Next[input];
     
  }
}


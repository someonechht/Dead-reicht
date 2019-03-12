#include "simpletools.h"                      // Include simple tools
#include "servo.h"

#define IN            0
#define OUT           14

#define PIN_ON        1  
#define PIN_OFF       0

#define U             128

typedef struct {
  volatile uint8_t enc_A;
  volatile uint8_t enc_B;
  volatile int32_t position_current;
  volatile int32_t position_target;
  volatile int32_t position_differnce;
  volatile uint8_t motor_pin;
  volatile int32_t speed_current;
  volatile int32_t speed_target;
  volatile int32_t speed_difference;
  
} enc_data_t, *penc_data_t;
  
penc_data_t create_enc_ws(
  uint8_t enc_A_pin, 
  uint8_t enc_B_pin, 
  int32_t position_current, 
  int32_t position_target, 
  uint8_t motor_pin, 
  float speed_current, 
  float speed_target)
  {
      // Create struct
    penc_data_t penc = (penc_data_t) malloc(sizeof(enc_data_t));
    
    // Populate variables
    penc -> enc_A = enc_A_pin;
    penc -> enc_B = enc_B_pin;
    penc -> position_current = position_current;
    penc -> position_target = position_target;
    penc -> motor_pin = motor_pin;
    penc -> speed_target = speed_target;
    penc -> speed_current = speed_current;
  
    // Return
    return penc;
  }

void enc_task(void*);

void start_enc(penc_data_t penc){
  const uint32_t memory_allocation_size = 400;
  void *stack = malloc(memory_allocation_size);
  cogstart(&enc_task, penc, stack, memory_allocation_size);
}  
  
void enc_task(void *arg){
  // Variable initialization
  const int8_t direction_array[16] = {0, 1, -1, U, -1, 0, U, 1, 1, U, 0, -1, U, -1, 1, 0}; // Create array for directions 
  penc_data_t penc = (penc_data_t) arg; // Cast arg to enc_data
  uint8_t state_current, state_previous;
  
  // Set pre-requisites
  set_directions(penc -> enc_A, penc -> enc_B, IN);  // Set pins to input
  
  // Populate initial variables
  state_previous = get_states(penc -> enc_A, penc -> enc_B); // Get initial state
  
  // Read pulses
  while(1){
    state_current = get_states(penc -> enc_A, penc -> enc_B); // Get state
    penc -> position_current -= direction_array[state_previous * 4 + state_current]; // Add direction to pos, based on graycode  SHOULD BE+=, NOT -=
    state_previous = state_current;
  }    
}  

inline int32_t squasch(int32_t turn_value){
    if(turn_value < -500) 
      return -500;
    else if(turn_value > 500) 
      return 500;
    else
      return turn_value;
}  



void set_position(penc_data_t penc){
  int32_t squasch_value,
          difference,
          difference_prev = penc -> position_target - penc -> position_current,
          difference_sum,
          s;
          
  uint32_t period = 800000,
           counter = CNT + period;
           
   difference_sum = 0; 
   s = period / 800000000;

    difference = penc -> position_target - penc -> position_current;
    squasch_value = difference * 2.95 + difference_sum * 5 + ((difference - difference_prev)/s) * 0.95;
    
    difference_prev = difference;
    if ( ((difference > 0) && (difference < 200))  || ((difference < 0) && (difference > -200)) )
      difference_sum += difference * s;
    
    pulse_out(penc -> motor_pin, 1500 + squasch(squasch_value));
}

int TEST_SPEED = -1;


void set_speed(void *arg)
{
  penc_data_t penc = (penc_data_t) arg;
  int32_t squasch_value = 0,
  speed_target,
  speed_difference_prev = 0,
  speed_difference_sum = 0,
  position_prev = penc -> position_current,
  motor_pin = penc -> motor_pin;

  uint32_t period = 800000,
           counter = CNT + period;
  
  while(1){
    high(15);
    if(penc -> speed_target == 0){
      pulse_out(penc -> motor_pin, 1500);
      break;
    }      
    penc->speed_current = (penc->position_current - position_prev);
    position_prev = penc->position_current;

    penc->speed_difference = penc->speed_target - penc->speed_current;
    squasch_value = (penc->speed_difference * 60) + (speed_difference_sum*49/2) ; //+ ((penc->speed_difference - speed_difference_prev)/period);
    
    speed_difference_prev = penc->speed_difference;
    
    
    if ( ((penc->speed_difference > 0) && (penc->speed_difference < 200))  || ((penc->speed_difference < 0) && (penc->speed_difference > -200)) ){
      speed_difference_sum += penc -> speed_difference;
      
      
    }

    
    TEST_SPEED = squasch_value;
    pulse_out(penc->motor_pin,  1500 + (squasch(squasch_value)));
    low(15);
    counter = waitcnt2(counter, period);
  }    
}  



int main()                                    
{

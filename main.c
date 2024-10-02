#include "nu32dip.h"         // config bits, constants, funcs for startup and UART
// include other header files here
#include "encoder.h"
#include "utilities.h"
#include "ina219.h"
#include <stdlib.h>

#define BUF_SIZE 200
static volatile int duty_cycle;
static volatile float Kp = 1;
static volatile float Ki = 10;

static volatile float pKp = 100;
static volatile float pKi = 0;
static volatile float pKd = 10;

#define PLOTPTS 100
#define TRACKPTS 1000
static volatile int REFarray[PLOTPTS];
static volatile int CURarray[PLOTPTS];
static volatile int POSarray[PLOTPTS];
static volatile float REFTarray[TRACKPTS]; 
static volatile float REFarray2[TRACKPTS];
static volatile float POSarray2[TRACKPTS];
static volatile int StoringData = 0;
static volatile int count = 0, count1 = 0, countt = 0;
static volatile float angle = 0.0;

static volatile float eint = 0.0, eint_pos = 0.0, eprev_pos = 0.0;
static volatile float commanded_current = 0.0;

volatile int N = 0;

void __ISR(_TIMER_4_VECTOR, IPL5SOFT) Position_Controller(void)
{
  switch(get_mode()) {
    case 3: { // HOLD

      WriteUART2("a"); 
      while(!get_encoder_flag()){} 
      set_encoder_flag(0); 
      int p = get_encoder_count(); 

      float enc_ang = (float) p * 360.0/1400.0;
      int error = angle - enc_ang;
      eint_pos = eint_pos + error;
      float u = pKp*error + pKi*eint_pos + pKd*(error - eprev_pos)*10000;

      commanded_current = u;

      eprev_pos = error;

      POSarray[count1] = enc_ang;    
      REFarray[count1] = angle;   
      count1++;

      if (count_h == PLOTPTS) {
        count_h = 0;
        eint_pos = 0;
        eprev_pos = 0;
        set_mode(IDLE);
      }
      break;
    }
    case 4: { // TRACK
      WriteUART2("a");
      while(!get_encoder_flag()){} 
      set_encoder_flag(0); 
      int p = get_encoder_count(); 

      float ang = (float) p * 360.0/1400.0;
      float ref_traj = REFTarray[countt];
      int error = ref_traj - ang;
      eint_pos = eint_pos + error;
      float u = pKp*error + pKi*eint_pos + pKd*(error - eprev_pos)*10000;

      commanded_current = u;
      eprev_pos = error;

      POSarray2[countt] = ang; 
      REFarray2[countt] = REFTarray[countt];
      countt++;

      if (countt == TRACKPTS) {
        countt = 0;
      }
      break;
    }
  }
  IFS0bits.T4IF = 0;   
}

void __ISR(_TIMER_2_VECTOR, IPL5SOFT) Current_Controller(void)
{
  switch(get_mode()) {
    case 0: {  // IDLE
      LATBbits.LATB2 = 1;
      OC1RS = 0;
      break;
    }
    case 1: {  // PWM
      if (duty_cycle < 0) {
        LATBbits.LATB2 = 0;
        OC1RS = (int) ((float)duty_cycle/-100.0 * (2400));
      }
      else {
        LATBbits.LATB2 = 1;
        OC1RS = (int) ((float)duty_cycle/100.0 * (2400));
      }
      break;
    }
    case 2: {  // ITEST
      unsigned int val = 0;
      if (count < 25) {val = 200;} 
      else if (count < 50) {val = -200;}
      else if (count < 75) {val = 200;}
      else {val = -200;}

      int current_sensor = INA219_read_current();
      int error = -1*(val - current_sensor);
      float eint = eint + error;
      float u = Kp*error + Ki*eint;
      
      if (u > 100.0) {
        u = 100.0;
      }
      else if (u < -100.0) {
          u = -100.0;
      }
      if (u < 0)
      {
        OC1RS = (int) ((u/-100.0) * 2400);
        LATBbits.LATB2 = 0;
      }
      else
      {
        OC1RS = (int) ((u/100.0) * 2400);
        LATBbits.LATB2 = 1;
      }
      if (StoringData) {
        CURarray[count] = current_sensor;
        REFarray[count] = val;
        count++;
      }
      if (count == PLOTPTS) {
        count = 0;
        eint = 0;
        StoringData = 0; // reset the flag
        set_mode(IDLE);
      }

      break;
    }
    case 3: {  // HOLD
      int current_sensor = INA219_read_current();
      float error = -1*(commanded_current - current_sensor);
      eint = eint + error;
      float u = Kp*error + Ki*eint;
      
      if (u > 100.0) {
        u = 100.0;
      }
      else if (u < -100.0) {
          u = -100.0;
      }
      if (u < 0)
      {
        OC1RS = (int) ((u/-100.0) * 2400);
        LATBbits.LATB2 = 1;
      }
      else
      {
        OC1RS = (int) ((u/100.0) * 2400);
        LATBbits.LATB2 = 0;
      }
      break;
    }
    case 4: {  // TRACK
      int current_sensor = INA219_read_current();
      float error = -1*(commanded_current - current_sensor);
      eint = eint + error;
      float u = Kp*error + Ki*eint;
      
      if (u > 100.0) {
        u = 100.0;
      }
      else if (u < -100.0) {
        u = -100.0;
      }
      if (u < 0)
      {
        OC1RS = (int) ((u/-100.0) * 2400);
        LATBbits.LATB2 = 1;
      }
      else
      {
        OC1RS = (int) ((u/100.0) * 2400);
        LATBbits.LATB2 = 0;
      }
      break;
    }
  }
  IFS0bits.T2IF = 0; // Clear
}

int main() 
{
  char buffer[BUF_SIZE];
  NU32DIP_Startup(); // cache on, min flash wait, interrupts on, LED/button init, UART init
  NU32DIP_GREEN = 1;                       // toggle the LEDs
  NU32DIP_YELLOW = 1;     

  UART2_Startup();
  INA219_Startup();

  set_mode(IDLE);

  __builtin_disable_interrupts();

  IPC2bits.T2IP = 5;               
  IPC2bits.T2IS = 0;
  IFS0bits.T2IF = 0;              
  IEC0bits.T2IE = 1;              

  TRISBCLR = 0x4;
  T3CONbits.TCKPS = 0;     // Timer3 prescaler N=1
  PR3 = 2400;              // period = (PR3+1) * N * 20.833 ns = 50000ns, 20 kHz
  TMR3 = 0;                // initial TMR3 count is 0
  T3CONbits.ON = 1;        // turn on Timer3

  RPB7Rbits.RPB7R = 5;      // set pin 7 to OC1
  T2CONbits.TCKPS = 0;      // Timer2 prescaler N=1
  PR2 = 9600;               // period = (PR2+1) * N * 20.833 ns = 200000ns, 5 kHz
  TMR2 = 0;                 // initial TMR2 count is 0
  OC1CONbits.OCM = 0b110;   // PWM mode without fault pin; other OC1CON bits are defaults
  OC1RS = 0;                // duty cycle = 0
  OC1R = 0;                 // initialize before turning OC1 on; afterward it is read-only
  T2CONbits.ON = 1;         // turn on Timer2
  OC1CONbits.OCTSEL = 1;    // Use Timer3
  OC1CONbits.ON = 1;        // turn on OC1

  IPC4bits.T4IP = 5;                // set priority 5 subpriority 0
  IPC4bits.T4IS = 1;
  IFS0bits.T4IF = 0;                // clear T4 flag status
  IEC0bits.T4IE = 1;              // enable T4 interrupts

  T4CONbits.TCKPS = 4;     // Timer4 prescaler
  PR4 = 15000;              // 
  TMR4 = 0;                // initial TMR4 count is 0
  T4CONbits.ON = 1;        // turn on Timer4

  // in future, initialize modules or peripherals here
  __builtin_enable_interrupts();

  while(1)
  {
    NU32DIP_ReadUART1(buffer,BUF_SIZE); // we expect the next character to be a menu command
    NU32DIP_GREEN = 1;                   // clear the error LED
    switch (buffer[0]) {
      
      case 'b':                     
      {
        sprintf(buffer, "%f\r\n", INA219_read_current());
        NU32DIP_WriteUART1(buffer);
        break;
      }

      case 'c':               
      {
        WriteUART2("a");
        while(!get_encoder_flag()) {}
        set_encoder_flag(0);
        char m[50];
        int p = get_encoder_count();
        sprintf(m, "%d\r\n", p); 
        NU32DIP_WriteUART1(m);
        break;
      }

      case 'd':                      // dummy command for demonstration purposes
      {
        int n = 0;
        NU32DIP_ReadUART1(buffer,BUF_SIZE);
        sscanf(buffer, "%d", &n);
        sprintf(buffer,"%d\r\n", n + 1); // return the number + 1
        NU32DIP_WriteUART1(buffer);
        break;
      }


      case 'e':
      {
        WriteUART2("b"); 
        break;
      }

      case 'f':
      {
        set_mode(PWM);
        int n = 0;
        NU32DIP_ReadUART1(buffer,BUF_SIZE);
        sscanf(buffer, "%d", &n);
        sprintf(buffer,"%d\r\n", n);
        NU32DIP_WriteUART1(buffer);

        duty_cycle = n;
        break;
      }
      case 'g':
      {
        NU32DIP_ReadUART1(buffer,BUF_SIZE);
        sscanf(buffer, "%f %f", &Kp, &Ki);
        break;
      }
      case 'h':
      {
        sprintf(buffer, "%f\r\n", Kp);
        NU32DIP_WriteUART1(buffer);

        sprintf(buffer, "%f\r\n", Ki);
        NU32DIP_WriteUART1(buffer);
        break;
      }
      case 'i':
      {
        NU32DIP_ReadUART1(buffer,BUF_SIZE);
        sscanf(buffer, "%f %f %f", &pKp, &pKi, &pKd);
        break;
      }
      case 'j':
      {
        sprintf(buffer, "%f\r\n", pKp);
        NU32DIP_WriteUART1(buffer);

        sprintf(buffer, "%f\r\n", pKi);
        NU32DIP_WriteUART1(buffer);

        sprintf(buffer, "%f\r\n", pKd);
        NU32DIP_WriteUART1(buffer);
        break;
      }
      case 'k':
      {
        StoringData = 1;
        set_mode(ITEST);
        while (StoringData) {;}

        sprintf(buffer, "%d\r\n", PLOTPTS);
        NU32DIP_WriteUART1(buffer);

        for (int i=0; i<PLOTPTS; i++) {
          sprintf(buffer, "%d %d \r\n", CURarray[i], REFarray[i]);
          NU32DIP_WriteUART1(buffer);
        }

        break;
      }
      case 'l':
      {
        WriteUART2("b");
        NU32DIP_ReadUART1(buffer,BUF_SIZE);
        sscanf(buffer, "%f", &angle);

        sprintf(buffer, "%d\r\n", PLOTPTS);
        NU32DIP_WriteUART1(buffer);

        set_mode(HOLD);

        while (get_mode() == HOLD) {;}

        for (int i=0; i<PLOTPTS; i++) { 
          sprintf(buffer, "%f %f \r\n", POSarray[i], REFarray[i]);
          NU32DIP_WriteUART1(buffer);
        }
        break;
      }
      case 'm':
      {
      }
      case 'n':
      {
        NU32DIP_ReadUART1(buffer,BUF_SIZE);
        sscanf(buffer, "%d", &N);
        for(int i = 0; i < N; i++) {
        NU32DIP_ReadUART1(buffer,BUF_SIZE);
        sscanf(buffer, "%f", &REFTarray[i]);
        }
        break;
      }
      case 'o':
      {
        sprintf(buffer, "%d\r\n", TRACKPTS);
        NU32DIP_WriteUART1(buffer);

        set_mode(TRACK);

        for (int i=0; i<TRACKPTS; i++) {
          sprintf(buffer, "%f %f \r\n", POSarray2[i], REFarray2[i]);
          NU32DIP_WriteUART1(buffer);
        }
        break;
      }
      case 'p':
      {
        set_mode(IDLE);
        duty_cycle = 0;
        break;
      }

      case 'r':
      {
        sprintf(buffer, "%d\r\n", get_mode());
        NU32DIP_WriteUART1(buffer);
        break;
      }
      
      case 'q':
      {
        // handle q for quit. Later you may want to return to IDLE mode here. 
        break;
      }
      default:
      {
        NU32DIP_GREEN = 0;  // turn on LED2 to indicate an error
        break;
      }
    }
  }
  return 0;
}

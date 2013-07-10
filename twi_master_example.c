//-----------------------IMU------------------------------------------///


#include "defines.h"


//------------------  C O N F I G U R A T I O N S IMU3000 -------------------

#define EEPROM_ADDRESS        0x68        // EEPROM's TWI address
#define EEPROM_ADDR_LGT       1           // Address length of the EEPROM memory
#define VIRTUALMEM_ADDR_START 0x68    // Address of the virtual mem in the EEPROM
#define TWI_SPEED             50000       // Speed of TWI

//------------------  D E F I N I T I O N S IMU3000 -------------------




 // USART Fix Parameters
 #define USART_0		                 	(&AVR32_USART0) ////!!!!!!!!!!!USART0 bei interrupt und USART1 bei anderer ausgabe!!!!!
 #define USART_0_IRQ             			AVR32_USART0_IRQ
 #define USART_BAUDRATE      		  		57600

 // TC Fix Parameters
 #define RC									(FOSC0/ 8 / 1000)
 #define TC_CHANNEL   						0
 volatile avr32_tc_t  *tc = &AVR32_TC;
 int tc_tick_disp;					// Time in ms
 int tc_tick_pid;					// Time in ms
 int tc_tick_mainLoop;

 //UART INTERRUPT
 volatile char ringbuffer[1000];
 volatile int writeBuffer = 0;
 volatile int readBuffer = 0;

 //gyro Werte in °/s
volatile double gx =0;
volatile double gy= 0;
volatile double gz = 0;
//gyro Werte in ° absolut
volatile double gxDeg = 0;
volatile double gyDeg = 0;
volatile double gzDeg = 0;
//gyro Wert in d°(veränderung des Winkel)
volatile double dGx = 0;
volatile double dGy = 0;
volatile double dGz = 0;
//Kalman
kalmannData KDx;
kalmannData KDy;



volatile bool dispUpdate = false;
volatile bool exe = false;
//motorEnable variable
volatile bool motors_enable = false;


//PID Allgemein
volatile bool pidOn = false;
volatile double frequency = 100.0;
//volatile double u = 0.0;
//PID Pitch
volatile double err = 0.0;
volatile double propotionalGain = 0.070;
volatile double derivateGain = 0.020;
volatile double u_pitch = 0.0;
volatile double r = 0.0;
//PID Pitch
volatile double err_r = 0.0;
volatile double propotionalGain_r = 0.070;
volatile double derivateGain_r = 0.020;
volatile double u_roll = 0.0;
volatile double ro = 0.0;
//PID Yaw
volatile double err_y = 0.0;
volatile double propotionalGain_y = 0.020;
volatile double derivateGain_y = 0.005;
volatile double u_yaw = 0.0;
volatile double y = 0.0;


volatile double motor1 = 0.0;
volatile double motor2 = 0.0;
volatile double motor3 = 0.0;
volatile double motor4 = 0.0;

volatile double defSpeed = 50.0;



void pid(){

	//P
	if(isnanf(KDx.X[0]) || isinff(KDx.X[0])){
		return;
	}
	else if(isnanf(KDy.X[0]) || isinff(KDy.X[0])){
		return;
	}

	err = r-KDx.X[0];
	double temp = err * propotionalGain;

	err_r = ro - KDy.X[0];
	double temp2 = err_r * propotionalGain_r;


	double tempU = temp;
	double tempU2 = temp2;

	//D
	temp = -KDx.X[1]*derivateGain;
	tempU += temp;

	temp2 = -KDy.X[1]*derivateGain_r;
	tempU2 += temp2;

	u_pitch = tempU;
	u_roll = tempU2;
}

void pid_yaw(){

	//P

	err_y = y-gzDeg;
	double temp_y = err_y * propotionalGain_y;

	double tempUy = temp_y;

	//D
	temp_y = -gz * derivateGain_y;
	tempUy += temp_y;

	if(tempUy > 0.5){
		tempUy = 0.5;
	}
	else if(tempUy < -0.5){
		tempUy = -0.5;
	}

	u_yaw = tempUy;
}




void saturationFunction(double up, double uy, double ur){

	double tempSpeed1;
	double tempSpeed2;
	double tempSpeed3;
	double tempSpeed4;

	tempSpeed2 = defSpeed + (defSpeed * up) - (defSpeed * uy);
	tempSpeed1 = defSpeed + (defSpeed * ur)+ (defSpeed * uy);
	tempSpeed4 = defSpeed - (defSpeed * up) - (defSpeed * uy);
	tempSpeed3 = defSpeed - (defSpeed * ur)+ (defSpeed * uy);

	if(tempSpeed1 > 250){
		tempSpeed1 = 250.0;
	}
	else if(tempSpeed1 < 30.0){
		tempSpeed1 = 30.0;
	}

	if(tempSpeed2 > 250){
		tempSpeed2 = 250.0;
	}
	else if(tempSpeed2 < 30.0){
		tempSpeed2 = 30.0;
	}

	if(tempSpeed3 > 250){
		tempSpeed3 = 250.0;
	}
	else if(tempSpeed3 < 30.0){
		tempSpeed3 = 30.0;
	}

	if(tempSpeed4 > 250){
		tempSpeed4 = 250.0;
	}
	else if(tempSpeed4 < 30.0){
		tempSpeed4 = 30.0;
	}

	motor1 = tempSpeed1;
	motor2 = tempSpeed2;
	motor3 = tempSpeed3;
	motor4 = tempSpeed4;


}



 // ISR, USART INTC: Speichere hier in den Ringbuffer
 __attribute__((__interrupt__))	// Folgende Funktion ist Interrupt
 static void usart_int_handler(void)
 {
	    int c;  // Empfangenes Datum

	    usart_read_char(USART_0, &c);
	    ringbuffer[writeBuffer] = c;
	    writeBuffer++;



 }
 // ISR, TC: Zähle hier den Counter hoch
  // Attribute nicht vergessen!
__attribute__((__interrupt__))
static void tc_irq(void)
{
	// Increment the ms seconds counter
	tc_tick_disp++;
	tc_tick_pid++;
	tc_tick_mainLoop++;


    if(tc_tick_pid >= (int)((1.0/frequency)*1000.0)){
    	tc_tick_pid = 0;
			exe = true;

	    }



	if(tc_tick_disp >= 100){
		tc_tick_disp = 0;
		dispUpdate = true;
	}




	// Clear the interrupt flag. This is a side effect of reading the TC SR.
	tc_read_sr(tc, TC_CHANNEL);
}

/*!
 * \brief The Push Buttons interrupt handler.
 */
#if __GNUC__
__attribute__((__interrupt__))
#elif __ICCAVR32__
__interrupt
#endif
static void dip204_example_PB_int_handler(void)
{



  if (gpio_get_pin_interrupt_flag(BUTTON0))
  {
	  //toggle motors
	  if(motors_enable == true){
		  motors_enable = false;
	  }
	  else{
		  motors_enable = true;
	  }




    gpio_clear_pin_interrupt_flag(BUTTON0);
  }

  if (gpio_get_pin_interrupt_flag(BUTTON1))
  {


    gpio_clear_pin_interrupt_flag(BUTTON1);
  }

  if (gpio_get_pin_interrupt_flag(BUTTON2))
  {




    gpio_clear_pin_interrupt_flag(BUTTON2);
  }
}

/*!
 * \brief function to configure push button to generate IT upon rising edge
 */
void dip204_example_configure_push_buttons_IT(void)
{
  gpio_enable_pin_interrupt(BUTTON0 , GPIO_RISING_EDGE);

  gpio_enable_pin_interrupt(BUTTON1 , GPIO_RISING_EDGE);

  gpio_enable_pin_interrupt(BUTTON2 , GPIO_RISING_EDGE);

  /* Disable all interrupts */
  Disable_global_interrupt();
  /* register PB0 handler on level 1 */
  INTC_register_interrupt( &dip204_example_PB_int_handler, AVR32_GPIO_IRQ_0 + (BUTTON0/8), AVR32_INTC_INT1);
  INTC_register_interrupt( &dip204_example_PB_int_handler, AVR32_GPIO_IRQ_0 + (BUTTON1/8), AVR32_INTC_INT1);
  INTC_register_interrupt( &dip204_example_PB_int_handler, AVR32_GPIO_IRQ_0 + (BUTTON2/8), AVR32_INTC_INT1);
  /* Enable all interrupts */
  Enable_global_interrupt();
}





int main(void)
{



	//-------------------------USART INTERRUPT REGISTRATION.------------//
	// Set Clock: Oscillator needs to initialized once: First
		 pcl_switch_to_osc(PCL_OSC0, FOSC0, OSC0_STARTUP);

		 // --------------		USART INIT		-----------------------------------------------
		 static const gpio_map_t USART_GPIO_MAP =
		  {
			{AVR32_USART0_RXD_0_0_PIN, AVR32_USART0_RXD_0_0_FUNCTION},
			{AVR32_USART0_TXD_0_0_PIN, AVR32_USART0_TXD_0_0_FUNCTION}
		  };

		  // USART options.
		  static const usart_options_t USART_OPTIONS =
		  {
			.baudrate     = USART_BAUDRATE,
			.charlength   = 8,
			.paritytype   = USART_NO_PARITY,
			.stopbits     = USART_1_STOPBIT,
			.channelmode  = USART_NORMAL_CHMODE
		  };

		// Assign GPIO to USART
		gpio_enable_module(USART_GPIO_MAP, sizeof(USART_GPIO_MAP) / sizeof(USART_GPIO_MAP[0]));

		// Init USART
		usart_init_rs232(USART_0, &USART_OPTIONS, FOSC0);

		Disable_global_interrupt();
		INTC_init_interrupts();			// Init Interrupt Table: Once at first

		// Register USART Interrupt (hinzufügen)
		INTC_register_interrupt(&usart_int_handler, AVR32_USART0_IRQ, AVR32_INTC_INT3);

		USART_0->ier = AVR32_USART_IER_RXRDY_MASK; // Activate ISR on RX Line
		Enable_global_interrupt();
	// -----------------------------------------------------------------------------------

		// --------------------------		Display INIT		----------------------------------
			// Map SPI Pins
			static const gpio_map_t DIP204_SPI_GPIO_MAP =
			  {
				{DIP204_SPI_SCK_PIN,  DIP204_SPI_SCK_FUNCTION },  // SPI Clock.
				{DIP204_SPI_MISO_PIN, DIP204_SPI_MISO_FUNCTION},  // MISO.
				{DIP204_SPI_MOSI_PIN, DIP204_SPI_MOSI_FUNCTION},  // MOSI.
				{DIP204_SPI_NPCS_PIN, DIP204_SPI_NPCS_FUNCTION}   // Chip Select NPCS.
			  };

			// add the spi options driver structure for the LCD DIP204
			  spi_options_t spiOptions =
			  {
				.reg          = DIP204_SPI_NPCS,
				.baudrate     = 1000000,
				.bits         = 8,
				.spck_delay   = 0,
				.trans_delay  = 0,
				.stay_act     = 1,
				.spi_mode     = 0,
				.modfdis      = 1
			  };


			// SPI Inits: Assign I/Os to SPI
			gpio_enable_module(DIP204_SPI_GPIO_MAP,
			                     sizeof(DIP204_SPI_GPIO_MAP) / sizeof(DIP204_SPI_GPIO_MAP[0]));

			// Initialize as master
			spi_initMaster(DIP204_SPI, &spiOptions);

			// Set selection mode: variable_ps, pcs_decode, delay
			spi_selectionMode(DIP204_SPI, 0, 0, 0);

			// Enable SPI
			spi_enable(DIP204_SPI);

			// setup chip registers
			spi_setupChipReg(DIP204_SPI, &spiOptions, FOSC0);

			// initialize delay driver: Muss vor dip204_init() ausgeführt werden
			delay_init( FOSC0 );

			// initialize LCD
			dip204_init(backlight_PWM, TRUE);
			// ---------------------------------------------------------------------------------------

			// -----------------			Timer Counter Init		---------------------------------
				// Timer Configs:  Options for waveform generation.
				static const tc_waveform_opt_t WAVEFORM_OPT =
				{
				.channel  = TC_CHANNEL,                        // Channel selection.

				.bswtrg   = TC_EVT_EFFECT_NOOP,                // Software trigger effect on TIOB.
				.beevt    = TC_EVT_EFFECT_NOOP,                // External event effect on TIOB.
				.bcpc     = TC_EVT_EFFECT_NOOP,                // RC compare effect on TIOB.
				.bcpb     = TC_EVT_EFFECT_NOOP,                // RB compare effect on TIOB.

				.aswtrg   = TC_EVT_EFFECT_NOOP,                // Software trigger effect on TIOA.
				.aeevt    = TC_EVT_EFFECT_NOOP,                // External event effect on TIOA.
				.acpc     = TC_EVT_EFFECT_NOOP,                // RC compare effect on TIOA: toggle.
				.acpa     = TC_EVT_EFFECT_NOOP,                // RA compare effect on TIOA: toggle

				.wavsel   = TC_WAVEFORM_SEL_UP_MODE_RC_TRIGGER,//  Count till RC and reset (S. 649): Waveform selection
				.enetrg   = FALSE,                             // External event trigger enable.
				.eevt     = 0,                                 // External event selection.
				.eevtedg  = TC_SEL_NO_EDGE,                    // External event edge selection.
				.cpcdis   = FALSE,                             // Counter disable when RC compare.
				.cpcstop  = FALSE,                             // Counter clock stopped with RC compare.

				.burst    = FALSE,                             // Burst signal selection.
				.clki     = FALSE,                             // Clock inversion.
				.tcclks   = TC_CLOCK_SOURCE_TC3                // Internal source clock 3, connected to fPBA / 8.
				};


				// TC Interrupt Enable Register
				static const tc_interrupt_t TC_INTERRUPT =
				{ .etrgs = 0, .ldrbs = 0, .ldras = 0, .cpcs  = 1, .cpbs  = 0, .cpas  = 0, .lovrs = 0, .covfs = 0
				};
				// 0 = No Effect | 1 = Enable ( CPCS = 1 enables the RC Compare Interrupt )

				// *****************   Timer Setup ***********************************************
				// Initialize the timer/counter.
				tc_init_waveform(tc, &WAVEFORM_OPT);         // Initialize the timer/counter waveform.

				// Set the compare triggers.
				tc_write_rc(tc, TC_CHANNEL, RC); // Set RC value.

				tc_configure_interrupts(tc, TC_CHANNEL, &TC_INTERRUPT);

				// Start the timer/counter.
				tc_start(tc, TC_CHANNEL);                    // And start the timer/counter.
				// *******************************************************************************

				Disable_global_interrupt();
				// Register TC Interrupt
				INTC_register_interrupt(&tc_irq, AVR32_TC_IRQ0, AVR32_INTC_INT0);

				Enable_global_interrupt();
				// ---------------------------------------------------------------------------------------

				//initialize Buttons
				  dip204_example_configure_push_buttons_IT();



//-------------------------------TWI R/W ---------------------------------------------------


  imu_init();
  initMotorControler();

  sensorDaten imu_data = {0};
  char disp1[30], disp2[30], disp3[30], disp4[30];
  double GX,GY,GZ, AX, AY, AZ;			//shifted comlete Data
  double filtredDataY = 0.0;
  double filtredDataX = 0.0;
  int z;
  double gAccx = 0.0;
  double gAccy = 0.0;
  double startValue = 0.0;

  RPY currMoveRPY;
  Quaternion currQuat;
  currQuat.q0 = 1.0;
  currQuat.q1 = 0;
  currQuat.q2 = 0;
  currQuat.q3 = 0;
  Quaternion deltaQuat;
  RotMatrix rot = {0};
  RPY reconverted;


  double currX[2];
  double currY[2];


  calibrate_all(&imu_data);
  initializeKalmann(&KDx);
  initializeKalmann(&KDy);


  double fifo[10] = {0};
  double fifo2[10] = {0};

  pidOn = true;

  while(1){


	  if(writeBuffer > readBuffer){

	  		  switch(ringbuffer[readBuffer]){

	  		  //ziel des PID
	  		  case 'v':
					  readBuffer++;
					  char buffervolt[10];
					  int vol = 0;
					  while(readBuffer < writeBuffer){
						  buffervolt[vol] = ringbuffer[readBuffer];
						  vol++;
						  readBuffer++;
					  }
					 defSpeed = atoff(buffervolt);
	  		  case 'r':
	  			  readBuffer++;
	  			  char buffer[10];
	  			  int q = 0;
	  			  while(readBuffer < writeBuffer){
	  				  buffer[q] = ringbuffer[readBuffer];
	  				  q++;
	  				  readBuffer++;
	  			  }
	  			  ro = atoff(buffer);
	  			  break;

	  		//PID-freq
	  		case 'f':

	  			  readBuffer++;
	  			  char buffer1[10];
	  			  int w = 0;
	  			  while(readBuffer < writeBuffer){
	  			  		buffer1[w] = ringbuffer[readBuffer];
	  			  		w++;
	  			  		readBuffer++;
	  			  }
	  			  frequency = atoff(buffer1);
	  			  break;

//	  		//differentiatior gain
//	  		case 'd':
//
//	  			readBuffer++;
//	  			char buffer3[20];
//	  			int qqq = 0;
//	  			while(readBuffer < writeBuffer){
//	  			  		buffer3[qqq] = ringbuffer[readBuffer];
//	  			  		qqq++;
//	  			  		readBuffer++;
//	  			}
//	  			derivateGain_r = atoff(buffer3);
//	  			break;


			case 'w':
				r++;
				break;
			case 'a':
				ro--;
				break;
			case 'd':
				ro++;
				break;
			case 's':
				r--;
				break;



	  		//proportional gain
	  		case 'p':
	  			z = 0;
	  			readBuffer++;
	  			char buffer4[20];

	  			int qqqq = 0;
	  			while(readBuffer < writeBuffer){
	  			  	buffer4[qqqq] = ringbuffer[readBuffer];
	  			  	qqqq++;
	  			  	readBuffer++;
	  			}
	  			propotionalGain_r = atoff(buffer4);
	  			z = 0;
	  		break;
	  		//desired yaw
	  		case 'y':
	  			z = 0;
	  			readBuffer++;
	  			char buffer5[20];

	  			int qqqqa = 0;
	  			while(readBuffer < writeBuffer){
	  			  	buffer5[qqqqa] = ringbuffer[readBuffer];
	  			  	qqqqa++;
	  			  	readBuffer++;
	  			}
	  			y = atoff(buffer5);
	  			z = 0;
	  		break;

	  		  }

	  		readBuffer = writeBuffer;
	  }




	  if(exe){


		  exe = false;

		  read_sensor(&imu_data);


//FILTER ACC Y
		  double g = imu_data.acc_y;
		  fifo[9] = fifo[8];
		  fifo[8] = fifo[7];
		  fifo[7] = fifo[6];
		  fifo[6] = fifo[5];
		  fifo[5] = fifo[4];
		  fifo[4] = fifo[3];
		  fifo[3] = fifo[2];
		  fifo[2] = fifo[1];
		  fifo[1] = fifo[0];
		  fifo[0] = g;


	  filtredDataY = ((fifo[9]+fifo[8]+fifo[7]+fifo[6]+fifo[5]+fifo[4]+fifo[3]+fifo[2]+fifo[1]+fifo[0])/10.0);

	  //FILTER ACC X

	  double gg = imu_data.acc_x;
	  fifo2[9] = fifo2[8];
	  fifo2[8] = fifo2[7];
	  fifo2[7] = fifo2[6];
	  fifo2[6] = fifo2[5];
	  fifo2[5] = fifo2[4];
	  fifo2[4] = fifo2[3];
	  fifo2[3] = fifo2[2];
	  fifo2[2] = fifo2[1];
	  fifo2[1] = fifo2[0];
	  fifo2[0] = gg;


  filtredDataX = ((fifo2[9]+fifo2[8]+fifo2[7]+fifo2[6]+fifo2[5]+fifo2[4]+fifo2[3]+fifo2[2]+fifo2[1]+fifo2[0])/10.0);

//
//
//
//
		  AX = filtredDataX + imu_data.acc_x_bias;
		  AY = filtredDataY + imu_data.acc_y_bias;
//		  AZ = imu_data.acc_z + imu_data.acc_z_bias;
//
		  GX = imu_data.gyro_x + imu_data.gyro_x_bias;
		  GY = imu_data.gyro_y + imu_data.gyro_y_bias;
		  GZ = imu_data.gyro_z + imu_data.gyro_z_bias;
//
//
//
//
//		  //convert to 1G
		  double ax = (double)AX * (-4.0);
		  double ay = (double)AY * (-4.0); //wegen 2^11= 2048, /2 = 1024 entspricht 4G -> 1G = (1024/4)
//		  float az = (float)AZ * (-4.0);


		  //vonvert Acc to Deg for Y
		  		  gAccy = ay / 1000.0;

		  		  if(gAccy > 1){
		  			  gAccy = 1.0;
		  		  }
		  		  if(gAccy < -1){
		  			  gAccy = -1.0;
		  		  }

		  		  gAccy = asinf(gAccy) * 180.0/M_PI;

		 //vonvert Acc to Deg for X
		  		  gAccx = ax / 1000.0;

		  		  if(gAccx > 1){
		  			  gAccx = 1.0;
		  		  }
		  		  if(gAccx < -1){
		  			  gAccx = -1.0;
		  		  }

		  		  gAccx = asinf(gAccx) * 180.0/M_PI;

//
//
		  //convert to 1°/s
		  gx = ((double)GX/ 14.375); // in °/s
		  gy = ((double)GY/ 14.375);
		  gz = ((double)GZ/ 14.375);

		  //Integration over time
//		  dGx = (gx*(1.0/frequency));
//		  dGy = (gy*0.01);
		  dGz = (gz*0.01);



					  currX[0] = gAccy;
					  currX[1] = gx;
					  currY[0] = -gAccx;
					  currY[1] = gy;


					  kalmannFilter(&KDx,currX);
					  kalmannFilter(&KDy,currY);







		  //aufaddieren auf den aktuellen Winkel IN GRAD
//			gxDeg += dGx;
//			gyDeg += dGy;
			gzDeg += dGz;









			if(pidOn){
				pid();
				pid_yaw();
			}


			//PID wird alle 10ms ausgeführt
			//werte kontrollieren:
			saturationFunction(u_pitch, u_yaw, u_roll);



			if(motors_enable){
				if(startValue < 1){
					startValue += 0.002;
				}
				writeMotorValue(1,motor1 * startValue);
				writeMotorValue(2,motor2 * startValue);
				writeMotorValue(3,motor3 * startValue);
				writeMotorValue(4,motor4 * startValue);
			}
			else{
				stopAllMotors();
				startValue = 0.0;
			}




	  }
	  if(dispUpdate){
		  dispUpdate = false;


		   sprintf(disp1, "x:%.1f,y:%.1f,z:%.1f ", KDx.X[0],KDy.X[0],gzDeg);
		   sprintf(disp2,"M1:%.0f,M2:%.0f,r:%.0f",motor1, motor2,r);
		   sprintf(disp3,"M3:%.0f,M4:%.0f,y:%.0f",motor3, motor4,y);
		   sprintf(disp4,"P:%.3f,D:%.3f",propotionalGain_r, derivateGain_r);







		   dip204_clear_display();

		   dip204_set_cursor_position(1,1);
		   dip204_write_string(disp1);
		   dip204_set_cursor_position(1,2);
		   dip204_write_string(disp2);
		   dip204_set_cursor_position(1,3);
		   dip204_write_string(disp3);
		   dip204_set_cursor_position(1,4);
		   dip204_write_string(disp4);



	  }


  }
}

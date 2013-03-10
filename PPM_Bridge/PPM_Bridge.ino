/*
 * PPM Bridge.
 *   Transform Turnigy9X mode 2 channels to Turnigy Q-BOT Micro Quadcopter
 *
 * Source code based on :
 *  read-any-ppm: http://code.google.com/p/read-any-ppm/downloads/list
 *  generate-ppm-signal: http://code.google.com/p/generate-ppm-signal/downloads/list
 * 
 * Channel mapping:  Camboui's posts the HK product discussion
 *  http://www.hobbyking.com/hobbyking/store/__29595__Q_BOT_Micro_Quadcopter_w_2_4gHz_RF_Module_Spektrum_JR_Futaba_compatible_.html
 *  quote:
 *   message 1: 
 *    Ok I could make it work. Q-Bot must first be binded. It need to be as close as
 *   possible to the module the first time (<10cm). WARNING ! There is no stick
 *   check when powering up, so if channel order is not correct or throttle is at
 *   lowest position, the Q-Bot will devastate your desk. For JR mode, channel
 *   order is TAER, and throttle rudder and aileron had to be reversed (at least
 *   for me and my TH9X with er9x).
 *   message 2:
 *   I meant "throttle is NOT at lowest position".
 *   message 3:
 *   Again mistake, aileron is not reversed, but elevator is.
 * 
 *
 */


#define PPM_INPUT_PIN 3
#define PPM_OUTPUT_PIN 10

#define INPUT_CHANNELS 8
#define OUTPUT_CHANNELS 5

#define PRINTS_PER_SECONDS 4
#define USECS_PER_PRINT 1000000L/PRINTS_PER_SECONDS

//////////////////////CONFIGURATION///////////////////////////////
#define default_servo_value 1500  //set the default servo value
#define PPM_FrLen 22500  //set the PPM frame length in microseconds (1ms = 1000µs)
#define PPM_PulseLen 300  //set the pulse length
#define onState 1  //set polarity: 1 is positive, 0 is negative
//////////////////////////////////////////////////////////////////

//serial
unsigned long nextRefresh;
//input
byte input_channel;
unsigned int input_pulse;
unsigned long input_counter;
int ppm_channel_input[INPUT_CHANNELS];
//output
boolean output_pulse;
byte output_counter;
unsigned long lastFrLen;
unsigned long lastServo;
unsigned long lastPulse;
boolean PPM_run;
boolean pulseStart = true;
byte part = true;
unsigned int ppm_channel_output[OUTPUT_CHANNELS];

void setup() {

  nextRefresh=0; // force new refresh

  Serial.begin(57600);
  Serial.println("ready");

  /* setup reader */
  
  pinMode(PPM_INPUT_PIN, INPUT);
  attachInterrupt(PPM_INPUT_PIN - 2, read_ppm, CHANGE);
  
  TCCR1A = 0x00;	   // COM1A1=0, COM1A0=0 => Disconnect Pin OC1 from Timer/Counter 1 -- PWM11=0,PWM10=0 => PWM Operation disabled
  TCCR1B = B00000010;     //0x02;	   // 16MHz clock with prescaler means TCNT1 increments every .5 uS (cs11 bit set
  TIMSK1 = _BV(ICIE1);   // enable input capture interrupt for timer 1

  /* setup writer */

  for(int i=0; i<OUTPUT_CHANNELS; i++){
    ppm_channel_output[i]= default_servo_value;
  }

  pinMode(PPM_OUTPUT_PIN, OUTPUT);
  digitalWrite(PPM_OUTPUT_PIN, !onState);  //set the PPM signal pin to the default state (off)

}

void read_ppm(){

  input_counter = TCNT1;
  TCNT1 = 0;

  if(input_counter < 1020){  //must be a pulse
    input_pulse = input_counter;
  } else if(input_counter > 3820){  //sync
    input_channel = 0;
  } else if ( input_channel < INPUT_CHANNELS ) { //servo values between 810 and 2210 will end up here
    ppm_channel_input[input_channel] = (input_counter + input_pulse)/2;
    transform_channel(input_channel);
    input_channel++;
  }
}

void transform_channel(byte channel) {
  int value=ppm_channel_input[channel];
  
  switch ( channel ) {
    case 0: // roll to channel 2
      //raw (-500/500)
      value=value-1500;
      // soft (50%)
      value=value*0.5;
      // unraw
      value=value+1500;
      // put
      ppm_channel_output[1]=value;
      break;
    case 1: // pitch reversed to 3
          //raw
      value=value-1500;
      // soft
      value=value*0.5;
      // reverse
      value=-value;
      // unraw
      value=value+1500;
      // put
      ppm_channel_output[2]=value;
      break;
    case 2: // throtle reversed to 1
      ppm_channel_output[0]=3000-value;
      break;
    case 3: // yaw reversed to 4
      ppm_channel_output[3]=3000-value;
      break;
    case 6: // reversed to 5
      ppm_channel_output[4]=3000-value;
      break;
    default:
      break;
      // nothing
  }
  
//  if ( channel < OUTPUT_CHANNELS )
//    ppm_channel_output[channel]=ppm_channel_input[channel];
}

void write_ppm(){  //generate PPM signal
  if(micros() - lastFrLen >= PPM_FrLen){  //start PPM signal after PPM_FrLen has passed
    lastFrLen = micros();
    PPM_run = true;
  }

  if(output_counter >= OUTPUT_CHANNELS){
    PPM_run = false;
    output_counter = 0;
    output_pulse = true;  //put out the last pulse
  }

  if(PPM_run){
    if (part){  //put out the pulse
      output_pulse = true;
      part = false;
      lastServo = micros();
    }
    else{  //wait till servo signal time (values from the ppm array) has passed
      if(micros() - lastServo >= ppm_channel_output[output_counter]){
        output_counter++;  //do the next channel
        part = true;
      }
    }
  }

  if(output_pulse){
    if(pulseStart == true){  //start the pulse
      digitalWrite(PPM_OUTPUT_PIN, onState);
      pulseStart = false;
      lastPulse = micros();
    }
    else{  //will wait till PPM_PulseLen has passed and finish the pulse
      if(micros() - lastPulse >= PPM_PulseLen){
        digitalWrite(PPM_OUTPUT_PIN, !onState);
        output_pulse = false;
        pulseStart = true;
      }
    }
  }
}

void loop(){

  if ( micros() > nextRefresh ) {
	nextRefresh=micros()+USECS_PER_PRINT;
        Serial.print(" INPUT: ");
	for ( int i=0 ; i<INPUT_CHANNELS ; i++) {
		Serial.print(i,DEC);
                Serial.print(":");
                Serial.print(ppm_channel_input[i],DEC);
                Serial.print(" ");
	}
	Serial.println();
        Serial.print("OUTPUT: ");
	for ( int i=0 ; i<OUTPUT_CHANNELS ; i++) {
		Serial.print(i,DEC);
                Serial.print(":");
                Serial.print(ppm_channel_output[i],DEC);
                Serial.print(" ");
	}
	Serial.println();
  }

  write_ppm();

}

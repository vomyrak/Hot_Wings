#include "mbed.h"
#include "SHA256.h"

//Photointerrupter input pins
#define I1pin D3
#define I2pin D6
#define I3pin D5

//Incremental encoder input pins
#define CHApin   D12
#define CHBpin   D11

//Motor Drive output pins   //Mask in output byte
#define L1Lpin D1           //0x01
#define L1Hpin A3           //0x02
#define L2Lpin D0           //0x04
#define L2Hpin A6          //0x08
#define L3Lpin D10           //0x10
#define L3Hpin D2          //0x20

#define PWMpin D9

//Motor current sense
#define MCSPpin   A1
#define MCSNpin   A0

#define Outpin D4

//Mapping from sequential drive states to motor phase outputs
/*
State   L1  L2  L3
0       H   -   L
1       -   H   L
2       L   H   -
3       L   -   H
4       -   L   H
5       H   L   -
6       -   -   -
7       -   -   -
*/
//Drive state to output table
const int8_t driveTable[] = {0x12,0x18,0x09,0x21,0x24,0x06,0x00,0x00};

//Mapping from interrupter inputs to sequential rotor states. 0x00 and 0x07 are not valid
const int8_t stateMap[] = {0x07,0x05,0x03,0x04,0x01,0x00,0x02,0x07};  
//const int8_t stateMap[] = {0x07,0x01,0x03,0x02,0x05,0x00,0x04,0x07}; //Alternative if phase order of input or drive is reversed

//Phase lead to make motor spin
int8_t lead = 2;  //2 for forwards, -2 for backwards

//Status LED
DigitalOut led1(LED1);

//Photointerrupter inputs
InterruptIn I1(I1pin);
InterruptIn I2(I2pin);
InterruptIn I3(I3pin);

//Motor Drive outputs
DigitalOut L1L(L1Lpin);
DigitalOut L1H(L1Hpin);
DigitalOut L2L(L2Lpin);
DigitalOut L2H(L2Hpin);
DigitalOut L3L(L3Lpin);
DigitalOut L3H(L3Hpin);
DigitalOut Measurement(Outpin);

//Threading
Mutex newKey_mutex;
Thread output_thread;
Thread command_thread;
Thread motor_control_thread(osPriorityNormal, 1024);

typedef struct {
    double rate;
    uint64_t * nonce;
} mail_t;

PwmOut pwm(PWMpin);
RawSerial pc(SERIAL_TX, SERIAL_RX);
Timer timer;
Mail<mail_t, 64>mail_box;
Queue<void, 8> inCharQ;


/* command setting */
float velocity_in = 0;
float position_in = 0;

// SHA
SHA256 crypt = SHA256();
uint8_t sequence[] = {0x45,0x6D,0x62,0x65,0x64,0x64,0x65,0x64,
 0x20,0x53,0x79,0x73,0x74,0x65,0x6D,0x73, 
 0x20,0x61,0x72,0x65,0x20,0x66,0x75,0x6E, 
 0x20,0x61,0x6E,0x64,0x20,0x64,0x6F,0x20, 
 0x61,0x77,0x65,0x73,0x6F,0x6D,0x65,0x20, 
 0x74,0x68,0x69,0x6E,0x67,0x73,0x21,0x20, 
 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 
 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; 
uint64_t* key = (uint64_t*)((int)sequence + 48); 
uint64_t* nonce = (uint64_t*)((int)sequence + 56); 
uint64_t newKey = 0;
uint8_t hash[32];

// Rotor state and positions
int8_t orState = 0;    //Rotot offset at motor state 0
int8_t intState = 0;
int8_t initState = 0;
int8_t prevState = 0;
int8_t stateDiff = 0;
int16_t position = 0;
int16_t prev_pos = 0;
int32_t rev_count = 0;
int8_t stage_count = 0;
int8_t cur_state = 0;
int8_t direction_estimate = 0;


/* constants for tuning */    
float k_pr = 25;//25
float k_dr = 19;//19, 17
float k_ps = 27;//27, 21
float k_is = 0.0001;//0.0001;

/* system state variable */
float e_r = 0;  // position error
float e_r_derivative = 0;   // position error derivative
float e_s = 0;  // velocity error
float e_s_integral = 0; // velocity derivative
float cumulative_revolution = 0;    // total revolution count
float revolution_increment = 0;     // revolution in iteration
float current_velocity = 0; // velocity calculated in iteration


//Set a given drive state
void motorOut(int8_t driveState){
    
    //Lookup the output byte from the drive state.
    int8_t driveOut = driveTable[driveState & 0x07];
    
    //Turn off first
    if (~driveOut & 0x01) L1L = 0;
    if (~driveOut & 0x02) L1H = 1;
    if (~driveOut & 0x04) L2L = 0;
    if (~driveOut & 0x08) L2H = 1;
    if (~driveOut & 0x10) L3L = 0;
    if (~driveOut & 0x20) L3H = 1;
    
    //Then turn on
    if (driveOut & 0x01) L1L = 1;
    if (driveOut & 0x02) L1H = 0;
    if (driveOut & 0x04) L2L = 1;
    if (driveOut & 0x08) L2H = 0;
    if (driveOut & 0x10) L3L = 1;
    if (driveOut & 0x20) L3H = 0;
}
    
//Convert photointerrupter inputs to a rotor state
inline int8_t readRotorState(){
    return stateMap[I1 + 2*I2 + 4*I3];
}

//Basic synchronisation routine    
int8_t motorHome() {
    //Put the motor in drive state 0 and wait for it to stabilise
    motorOut(0);
    wait(2.0);
    //Get the rotor state
    return readRotorState();
}
    

    
void ISR(){
    intState = readRotorState();

    stateDiff = intState - prevState;

	if (stateDiff == 1 || stateDiff == -5) rev_count++;
	else if (stateDiff == -1 || stateDiff == 5) rev_count--;

    prevState = intState;
    motorOut((intState-orState+lead+6)%6); //+6 to make sure the remainder is positive
}    

void motorCtrlTick(){
    motor_control_thread.signal_set(0x1);
}

void velocity_monitor(){
    
    /* torque decision */
    float T_r = 0;
    float T_s = 0;
    float T_out = 0;
    Ticker ticker;
    uint8_t iter_count = 0;
    ticker.attach_us(&motorCtrlTick, 50000);    // to have better performance with counting for every ISR, higher sampling rate is used
    while(1){
        motor_control_thread.signal_wait(0x1);  // timer count 0.1s
        core_util_critical_section_enter();
        revolution_increment = rev_count / 6.0;
        rev_count = 0;
        cur_state = intState;
        core_util_critical_section_exit();
        cumulative_revolution += revolution_increment;
        e_r_derivative = revolution_increment * (-20);
        e_s = fabsf(e_r_derivative) - velocity_in;
        e_s_integral += e_s;
        if (e_s_integral > 1) e_s_integral = 1;
        e_r = position_in - cumulative_revolution;
        
        T_r = k_pr * e_r + k_dr * e_r_derivative;
        T_s = k_ps * e_s + k_is * e_s_integral;
        if (position_in == 0) T_out = T_s;
        else if (velocity_in == 0) T_out = T_r;
        else{
            if (e_r_derivative < 0) {
                T_s *= -1;
                T_out = min(T_r, T_s);
            }
            else T_out = max(T_r, T_s);
        }
        if (T_out < 0) {
            lead = -2;
            T_out *= -1;
        }
        else lead = 2;
        if (T_out > 1) T_out = 1;
        pwm.write(T_out);
        if (++iter_count == 20){
            iter_count = 0;

        }
    }
}



void output (void){
    while (1) {
        osEvent evt = mail_box.get();
        if (evt.status == osEventMail) {
            mail_t *mail = (mail_t*) evt.value.p;
            if (mail -> nonce == NULL) pc.printf("rate: %.f hashes per second\n\r", mail->rate);
            else pc.printf("%016llX\n\r", *(mail->nonce));
            mail_box.free(mail);
        }
    }
}

void serialISR(){
    uint8_t newChar = pc.getc();
    inCharQ.put((void*)newChar);
}
char char2hex(char a){
    if (a >= '0' && a <= '9'){
        return (a - '0');
    }
    else if (a >= 'A' && a <= 'F'){
        return (a - 'A' + 10);
    }
    else if (a >= 'a' && a <= 'f'){
        return (a - 'a' + 10);
    }
    else return a;
}

void decode_thread(void){
    pc.attach(&serialISR);
    uint8_t * new_chars = new uint8_t[17];
    while (1){
        int char_index = 0;
        osEvent newEvent = inCharQ.get();
        char char_ptr = (char)newEvent.value.p; 
        while(char_ptr != '\r') {
            new_chars[char_index] = (uint8_t)char_ptr;
            newEvent = inCharQ.get();
            char_ptr = (char)newEvent.value.p;
            char_index++;
        }
        if (new_chars[0] == 'K') {
            if (char_index == 17){
                for (int i = 1; i < 9; i++){
                    new_chars[i] = char2hex(new_chars[2 * i]) + (char2hex(new_chars[2 * i - 1]) << 4);
                }
                newKey_mutex.lock();
                memcpy(key, new_chars + 1, sizeof(uint64_t));
                newKey_mutex.unlock();
            }
        }
        else if (new_chars[0] == 'V'){
            if (char_index < 10){
                new_chars[char_index] = '\0';
                newKey_mutex.lock();
                velocity_in = (float)atof((char *)(new_chars + 1));
                if (velocity_in == 0) velocity_in = 200;
                newKey_mutex.unlock();
                new_chars[char_index] = '0';
            }
        }
        else if (new_chars[0] == 'R'){
            if (char_index < 10){
                new_chars[char_index] = '\0';
                newKey_mutex.lock();
                position_in = (float)atof((char *)(new_chars + 1));
                cumulative_revolution = 0;    // total revolution count
                e_s_integral = 0; // velocity derivative
                newKey_mutex.unlock();
                new_chars[char_index] = '0';
            }
        }
    }
}
//Main
int main() {


    //Initialise the serial port


    /* Initialise Motor */
    velocity_in = 100;
    position_in = 1000;
    pwm.period_ms(2);
    pwm.write(1.0f);
    pc.printf("Hello\n\r");
    orState = motorHome();
    prevState = orState;
    pc.printf("Rotor origin: %x\n\r",orState);

    float rate = 0;
    float timer_time = 0;
    int count = 0;
    output_thread.start(&output);
    command_thread.start(&decode_thread);
    motor_control_thread.start(&velocity_monitor);

    I1.rise(&ISR);
    I2.rise(&ISR);
    I3.rise(&ISR);
    I1.fall(&ISR);
    I2.fall(&ISR);
    I3.fall(&ISR);
    
    // Immediately start the motor if position_in and velocity_in are preset.
    if (position_in >= 0) lead = 2;
    else if (position_in < 0) lead = -2;
    motorOut((lead + 6) % 6);

    
    //Poll the rotor state and set the motor outputs accordingly to spin the motor
    timer.start();
    while (1) {

         timer_time = timer.read();
         if (timer_time >= 1){
             mail_t *mail = mail_box.alloc();
             rate = count / timer_time;
             mail -> rate = rate;
             mail -> nonce = NULL;
             timer.reset();
             count = 0;
             timer_time = 0;
             mail_box.put(mail);
         }
         newKey_mutex.lock();
         crypt.computeHash(hash, sequence, 64);
         newKey_mutex.unlock();
         if (hash[0] == 0 && hash[1] == 0) {
            mail_t *mail = mail_box.alloc();
            mail -> rate = NULL;
            mail -> nonce = nonce;
            mail_box.put(mail);
         }
         (*nonce) += 1;
         count += 1;
    }
}
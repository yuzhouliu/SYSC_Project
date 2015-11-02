Pin 01 is Thumb
Pin 64 is Index finger
Pin 02 is Middle

PWM instructions:
/*
// The PWM works based on the following settings:
//     Timer reload interval -> determines the time period of one cycle
//     Timer match value -> determines the duty cycle
// The computation for PWM is as described below:
// System frequency = 80 Mhz.
// For a time period of 20 ms (Servo), 
//    Timer reload value = 80,000,000/(1/20ms) = 1,600,000 cycles
//    Cannot store this in 16-bit reload register, so must use prescaler
//    1,600,000 = 0x186A00. Store lower 16 bits into Load, higher 8 bits into prescale
//    PRESCALE = 0x18, RELOAD = 0x6A00
// Servos rotate using duty cycle, 0.5ms for 0 degrees, 2.5ms for 180 degrees.
//    Timer Match for 0.5ms: 80,000,000/(1/0.5ms) = 40,000 cycles
//      40,000 = 0x9C40, MatchPrescale = 0x0, MatchReload = 0x9C40
//    Timer Match for 2.5ms: 80,000m000/(1/2.5ms) = 200,000 cycles
//      200,000 = 0x30D40, MatchPrescale = 0x3, MatchReload = 0x0D40
*/


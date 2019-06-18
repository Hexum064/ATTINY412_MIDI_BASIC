/* 
 * File:   main.c
 * Author: Branden
 *
 * Created on June 2, 2019, 5:02 PM
 */
#define F_CPU 20000000
#define USART0_BAUD_RATE(BAUD_RATE) ((float)(F_CPU * 64 / (16 * (float)BAUD_RATE)) + 0)

#include "../include/avr/io.h"
#include "../include/avr/interrupt.h"
//#include <string.h>

#define NUM_VOICES 3
#define STATUS_BYTE_MASK 0xF0
#define NOTE_ON_STATUS 0x90
#define NOTE_OFF_STATUS 0x80
#define PITCH_BEND_STATUS 0xE0
#define LED_COUNT 24

volatile uint8_t _voice0Out;
volatile uint8_t _voice1Out;
volatile uint8_t _voice2Out;
volatile uint8_t _noteIn = 0;
volatile uint8_t _rxNoteStatus = 0;
volatile uint16_t _updateDisplayFlag = 0;
volatile uint8_t _displayMode = 0;
volatile uint8_t _colorStep = 0;
volatile uint8_t _displayEnabled = 0;
volatile uint16_t _pinDebounce = 0;
volatile int16_t _pitchBend0; //signed
volatile int16_t _pitchBend1; //signed
volatile int16_t _pitchBend2; //signed
volatile uint8_t _pitchBendVal;

typedef struct voice
{
	uint8_t waveHi;
	uint8_t noteNumber;
	uint16_t freq;
	uint8_t velocity;
	
} voice_t;

typedef struct colors
{
    uint8_t red;
    uint8_t green;
    uint8_t blue;   
}colors_t;

static const uint16_t noteFreqLUT[12] =
{
	158,168,178,188,199,211,224,237,251,266,282,299
};

static voice_t _voices[NUM_VOICES] = 
{
	{0, 0, 0, 0},
	{0, 0, 0, 0},
	{0, 0, 0, 0}
};

static colors_t _colors[LED_COUNT + 1] = 
{

    {112, 0, 0},
    {96, 16, 0},
    {80, 32, 0},
    {64, 48, 0},
    {48, 64, 0},
    {32, 80, 0},
    {16, 96, 0},
    {0, 112, 0},
            
    {0, 112, 0}, 
    {0, 96, 16}, 
    {0, 80, 32}, 
    {0, 64, 48}, 
    {0, 48, 64}, 
    {0, 32, 80}, 
    {0, 16, 96}, 
    {0, 0, 112},
    
    {0, 0, 112},
    {16, 0,96},
    {32, 0, 80},
    {48, 0, 64},
    {64, 0, 48},
    {80, 0, 32},
    {96, 0, 16},
    {112, 0, 0},
    
    {0, 0, 0}
};

void initClock(void)
{
    CCP = 0xD8;
    CLKCTRL.MCLKCTRLB = 0x00;
}

void initDAC()
{
    PORTA.DIRSET = PIN6_bm;         //SET PORTA6 as output for DAC
    VREF.CTRLA |= VREF_DAC0REFSEL_4V34_gc;
	DAC0.CTRLA = DAC_OUTEN_bm | DAC_ENABLE_bm;
}

void initTimers()
{
    TCA0.SINGLE.INTCTRL = TCA_SINGLE_CMP0EN_bm | TCA_SINGLE_CMP1EN_bm | TCA_SINGLE_CMP2EN_bm;
    TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV16_gc | TCA_SINGLE_ENABLE_bm;
    

    TCB0.CTRLB = TCB_CNTMODE_INT_gc;
    TCB0.INTCTRL = TCB_CAPT_bm;
    TCB0.CCMP = 200; //With CLKDIV2, this should file the IRQ at 50kHz.
    TCB0.CTRLA = TCB_CLKSEL_CLKDIV2_gc | TCB_ENABLE_bm;
	
}

void initMIDIUART()
{
    PORTA.DIR &= ~PIN7_bm;
    USART0.BAUD = (uint16_t)USART0_BAUD_RATE(31500);
    USART0.CTRLA |= USART_RXCIE_bm;
    USART0.CTRLB |= USART_RXEN_bm;
}

void initRGBSPI()
{
    PORTA.DIR |= PIN1_bm | PIN3_bm;
    SPI0.CTRLB = SPI_SSD_bm ;
    SPI0.CTRLA = SPI_MASTER_bm | SPI_ENABLE_bm;// | SPI_DORD_bm;    
}

void initDisplayModePin()
{
    PORTA.DIRCLR = PIN2_bm;
    PORTA.PIN2CTRL = PORT_PULLUPEN_bm;
}


void initInterrupts()
{
    SREG = 1;
    sei();    
}

//void testTimingWithBlinkingLED(void)
//{
//    PORTA.DIRSET = PIN1_bm;
//    
//    while(1)
//    {
//        PORTA.OUTTGL = PIN1_bm;
//    }
//}





/*
 * 
 */

//void USART0_sendChar(char c)
//{
//    while (!(USART0.STATUS & USART_DREIF_bm))
//    {
//        ;
//    }
//    USART0.TXDATAL = c;
//}
//
//void USART0_sendString(char *str)
//{
//    for(size_t i = 0; i < strlen(str); i++)
//    {
//        USART0_sendChar(str[i]);
//    }
//}
//
//uint8_t USART0_read()
//{
//    while (!(USART0.STATUS & USART_RXCIF_bm))
//    {
//        
//    }
//    PORTA.OUTTGL = PIN1_bm;
//    return USART0.RXDATAL;
//}

void sendSPI(uint8_t val)
{
    while(!(SPI0.INTFLAGS & SPI_IF_bm)) {}
   
    SPI0.DATA = val;    
}
//volatile uint8_t _tempBlue = 0;
//volatile uint8_t _tempGreen = 0;
//volatile uint8_t _tempRed = 0;
//
//void testRGB()
//{
//    //Start frame
//    sendSPI(0);
//    sendSPI(0);
//    sendSPI(0);
//    sendSPI(0);
//    
//    for(int i = 0; i < 24; i++)
//    {
//        sendSPI(0xE7);// 0b1110 1111
//        sendSPI(_tempBlue);// blue
//        sendSPI(_tempGreen);// green
//        sendSPI(_tempRed);// red
//        
////        sendSPI(0xE7);// 0b1110 1111
////        sendSPI(0x00);// blue
////        sendSPI(0xEF);// green
////        sendSPI(0x00);// red
////        
////        sendSPI(0xE7);// 0b1110 1111
////        sendSPI(0x00);// blue
////        sendSPI(0x00);// green
////        sendSPI(0xEF);// red
//    }
//    _tempBlue++;
//    _tempRed+=7;
//    _tempGreen+=3;
//     
//    //End frame
//    sendSPI(0xFF);
//    sendSPI(0);
////    sendSPI(0);
////    sendSPI(0);
//
//}



//void uartTest()
//{
//    PORTA.DIR |= PIN1_bm;
//    PORTA.DIR |= PIN6_bm; // &= ~PIN2_bm;
//    PORTA.DIR &= ~PIN7_bm;
//
//    USART0.BAUD = (uint16_t)USART0_BAUD_RATE(31500);
//    USART0.CTRLA |= USART_RXCIE_bm;
//    USART0.CTRLB |= USART_RXEN_bm;
//    
////    while(1)
////    {
////        USART0_read();
////    }
//    
//    SREG = 1;
//    sei();
//    
//    while (1)
//    {
//        //USART0_sendString("UUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUU");
//        //USART0_sendChar('U');
//        
//
//    }
//}


//----NOTE CONTROL----

uint16_t getNoteFreq(uint8_t note)
{
	if (note == 108)
		return 149;
		
	note++;
	uint8_t offset = (108-note)/12;
	uint8_t index = 108-(note + offset * 12); //Need to calc offset first. This works because we drop the decimal portion of offset which gives us our index
	return noteFreqLUT[index] << offset;
}
uint16_t calcPitchBend(uint8_t voiceNum) //we only expect pitchVal to be a 6 bit number
{
    uint16_t nextFreq;
    
    if (!_voices[voiceNum].velocity)
    {
        return 0;
    }
    //Pitch needs to increase so we need to subtract a number from the frequency (because the freq is actually what the CC reg gets incremented by)
    if (_pitchBendVal < 32)
    {
        nextFreq = getNoteFreq(_voices[voiceNum].noteNumber - 1);
        return ((nextFreq - _voices[voiceNum].freq)/16) * (32 - _pitchBendVal);
    }
    else
    {
        nextFreq = getNoteFreq(_voices[voiceNum].noteNumber + 1);
        return (( _voices[voiceNum].freq - nextFreq)/16) * (_pitchBendVal - 32) * -1;// -1 * (( _voices[0].freq - nextFreq) / ((64 - pitchVal) >> 1));  
    }
}

//Using this to calculate a value between the current note and the next or previous note;
void calcPitchBends() //we only expect pitchVal to be a 6 bit number
{

    
    if (_pitchBendVal == 32) //Center pitch so no change
    {
        _pitchBend0 = 0;
        _pitchBend1 = 0;
        _pitchBend2 = 0;
        return;
    }
    
    _pitchBend0 = calcPitchBend(0);
    _pitchBend1 = calcPitchBend(1);
    _pitchBend2 = calcPitchBend(2);
}

int8_t getNextVoice(uint8_t note)
{
	for(int i = 0; i < NUM_VOICES; i++)
	{
		if (_voices[i].noteNumber == 0 || _voices[i].noteNumber == note)
		{
			return i;
		}
	}
	
	return -1;
}

void activateVoice(uint8_t voiceNum)
{
	
	//*(voices[voiceNum].compareReg) = noteFreqLUT[voices[voiceNum].noteNumber];
	//TODO, set frequency for voice Timer

	
	switch(voiceNum)
	{
		case 0:
			TCA0.SINGLE.CMP0 = _voices[voiceNum].freq;
			//TCC0.INTCTRLB |= TC_TC0_CCAINTLVL_HI_gc;
			break;
		case 1:
			TCA0.SINGLE.CMP1 = _voices[voiceNum].freq;
			//TCC0.INTCTRLB |= TC_TC0_CCBINTLVL_HI_gc;
			break;
		case 2:
			TCA0.SINGLE.CMP2 = _voices[voiceNum].freq;
			//TCC0.INTCTRLB |= TC_TC0_CCCINTLVL_HI_gc;
			break;
		//case 3:
			//TCC1.CCA = voices[voiceNum].freq;

//			break;						
		
	}
}

void deactivateVoice(uint8_t voiceNum)
{
	
	switch(voiceNum)
	{
		case 0:
		TCA0.SINGLE.CMP0 = 0;
		_voice0Out = 0;
		break;
		case 1:
		TCA0.SINGLE.CMP1 = 0;
		_voice1Out = 0;
		break;
		case 2:
		TCA0.SINGLE.CMP2 = 0;
		_voice2Out = 0;
		//break;
		//case 3:

		break;
		
	}	
}

void noteOff(uint8_t note)
{

	for (uint8_t i = 0; i < NUM_VOICES; i++)
	{
		if (_voices[i].noteNumber == note)
		{
			deactivateVoice(i);
			_voices[i].noteNumber = 0;
			_voices[i].velocity = 0;
		}
	}

}

void noteOn(uint8_t note, uint8_t velocity)
{
	uint8_t voiceNum;
    
	if (velocity == 0)
	{
		noteOff(note);
		return;
	}
	
	voiceNum = getNextVoice(note);

	if (voiceNum < 0)
	{	
		return;
	}
	
	_voices[voiceNum].velocity = velocity;
	_voices[voiceNum].noteNumber = note;
	_voices[voiceNum].freq = getNoteFreq(note);
	
	activateVoice(voiceNum);
}

//----END NOTE CONTROL----

void sendStartOfFrame()
{
    sendSPI(0);
    sendSPI(0);
    sendSPI(0);
    sendSPI(0);
}

void sendEndOfFrame()
{
    sendSPI(0xFF);
    sendSPI(0);

}

void blankDisplay()
{
    sendStartOfFrame();
    for(int i=0;i<LED_COUNT;i++)
    {
        sendSPI(0xE0);//L
        sendSPI(0);   //B
        sendSPI(0);   //G
        sendSPI(0);   //R
    }
    sendEndOfFrame();
            
}

void updateDisplay()
{
    uint8_t red = 0;
    uint8_t green = 0;
    uint8_t blue = 0;
    uint8_t step = 0;
    
    if (!_displayEnabled)
    {
        blankDisplay();
        return;
                
    }
    
    _updateDisplayFlag = 0;
    sendStartOfFrame();
    switch((_displayMode & 0x07))
    {
        case 0: //Full String Color Fade
            
            step = (_colorStep & 0x3F)<< 1;
            
            switch((_colorStep & 0xC0))
            {
                case 0xC0:
                    _colorStep = 0;
                case 0:
                    red = 128 - step;
                    blue = step;                    
                    break;
                case 0x40:
                    blue = 128 - step;
                    green = step;                    
                    break;
                case 0x80:
                    green = 128 - step;
                    red = step;                    
                    break;

            }
            
            _colorStep++;
            
            for(int i=0;i<LED_COUNT;i++)
            {
  
                sendSPI(0xE7);                          //L
                sendSPI(blue);      //B
                sendSPI(green);      //G
                sendSPI(red);           //R
            }            
            
            break;         
        case 1: //Multi color fade
            
            step = _colorStep;
            
            for(int i=0;i<LED_COUNT;i++)
            {
                step += i;
                
                step %= 24;
                
                sendSPI(0xE7);                          //L
                sendSPI(_colors[step].blue);      //B
                sendSPI(_colors[step].green);      //G
                sendSPI(_colors[step].red);           //R
            }
            
            _colorStep++;
            
            break;
        case 2: //Red VU
            
            red = (_voices[0].velocity + _voices[1].velocity + _voices[2].velocity) >> 2;

            for(int i=0;i<LED_COUNT;i++)
            {                
                sendSPI(0xE7);                          //L
                sendSPI(0);      //B
                sendSPI(0);      //G
                sendSPI(red);           //R
            }
            break;
        case 3: //Green VU
            green = (_voices[0].velocity + _voices[1].velocity + _voices[2].velocity) >> 2;

            for(int i=0;i<LED_COUNT;i++)
            {                
                sendSPI(0xE7);                          //L
                sendSPI(0);      //B
                sendSPI(green);      //G
                sendSPI(0);           //R
            }
            break;
        case 4: //Blue VU
            blue = (_voices[0].velocity + _voices[1].velocity + _voices[2].velocity) >> 2;

            for(int i=0;i<LED_COUNT;i++)
            {                
                sendSPI(0xE7);                          //L
                sendSPI(blue);      //B
                sendSPI(0);      //G
                sendSPI(0);           //R
            }
            break;
            
        case 5: //Bar VU
            step = 0;
            
            step += (_voices[0].velocity && 1);
            step += (_voices[1].velocity && 1);
            step += (_voices[2].velocity && 1);
            
            step *= 8;
            
            if (_colorStep < step)
            {
                _colorStep = step;
            }
            
            for(int i=0;i<_colorStep;i++)
            {                
                sendSPI(0xE7);                          //L
                sendSPI(_colors[i].blue);      //B
                sendSPI(_colors[i].green);      //G
                sendSPI(_colors[i].red);           //R
            }
            
            for(int i=_colorStep;i<LED_COUNT;i++)
            {                
                sendSPI(0xE7);                          //L
                sendSPI(0);      //B
                sendSPI(0);      //G
                sendSPI(0);           //R
            }            
            if (_colorStep > 0)
            {
                _colorStep-=2;
            }
            break;            
        case 6: //Frequency VU
            red = (24* _voices[0].noteNumber)/108;
            green = (24* _voices[1].noteNumber)/108;
            blue = (24* _voices[2].noteNumber)/108;            

            if (_pitchBendVal < 16)
            {
                if (red > 0)
                {
                    red -= 1;
                }
                
                if (green > 0)
                {
                    green -= 1;
                }
                
                if (blue > 0)
                {
                    blue -= 1;
                }                
            }
            
            if (_pitchBendVal < 32)
            {
                if (red > 0)
                {
                    red -= 1;
                }
                
                if (green > 0)
                {
                    green -= 1;
                }
                
                if (blue > 0)
                {
                    blue -= 1;
                }                
            }
            
            if (_pitchBendVal > 32)
            {
                red++ ;
                green++ ;
                blue++ ;
            }
            
            if (_pitchBendVal > 48)
            {
                red++ ;
                green++ ;
                blue++ ;
            }
            
            for(int i=0;i<LED_COUNT;i++)
            {       
                if (i+1 == red || i+1 == green || i+1 == blue)
                {
                    sendSPI(0xE7);   
                    sendSPI(_colors[i].blue >> 0);      //B
                    sendSPI(_colors[i].green >> 0);      //G
                    sendSPI(_colors[i].red >> 0);           //R                    
                }   
                else
                {
                    sendSPI(0xE1);   
                    sendSPI(_colors[i].blue >> 3);      //B
                    sendSPI(_colors[i].green >> 3);      //G
                    sendSPI(_colors[i].red >> 3);           //R
                }
                
                //L

            }            
            break;
        case 7: //Voice VU
            
            step = 0;
            
            if (_voices[0].velocity)
            {
                step++;
            }

            if (_voices[1].velocity)
            {
                step++;
            }

            if (_voices[2].velocity)
            {
                step++;
            }
            
            if (step == 1)
            {
                _colors[LED_COUNT].red = 0;
                _colors[LED_COUNT].green = 127;
                _colors[LED_COUNT].blue = 0;
            }
            
            if (step == 2)
            {
                _colors[LED_COUNT].red = 0;
                _colors[LED_COUNT].green = 0;
                _colors[LED_COUNT].blue = 127;
            } 

            if (step == 3)
            {
                _colors[LED_COUNT].red = 127;
                _colors[LED_COUNT].green = 0;
                _colors[LED_COUNT].blue = 0;
            } 
            
            for(int i=0;i<LED_COUNT;i++)
            {       
                sendSPI(0xE7);   
                sendSPI(_colors[LED_COUNT].blue >> 0);      //B
                sendSPI(_colors[LED_COUNT].green >> 0);      //G
                sendSPI(_colors[LED_COUNT].red >> 0);           //R                   
            }  
            
            _colors[LED_COUNT].red >>= 1;
            _colors[LED_COUNT].green >>= 1;
            _colors[LED_COUNT].blue >>= 1;
                    
            break;            
    }
    sendEndOfFrame();
}

int main(int argc, char** argv) {

    initClock();
    //testTimingWithBlinkingLED();
    
    //uartTest();
    
    initDAC();
    
    initRGBSPI();
    SPI0.DATA = 0xFF;
//    testRGB();
//    
//    while(1){}
    
    initMIDIUART();
    initTimers();
    initDisplayModePin();
    initInterrupts();
//    PORTA.DIRSET = PIN6_bm; //TEST CODE

//    noteOn(49, 0x07);
    _displayEnabled = 0xFF;
    while(1)
    {
        if (_updateDisplayFlag >= 2500)
        {
            updateDisplay();
        }
    }
    
}

ISR(USART0_RXC_vect)
{
    uint8_t data = USART0.RXDATAL;
    
	if ((data & STATUS_BYTE_MASK) == NOTE_ON_STATUS)
	{
		_rxNoteStatus = 1;
		return;
	}
	
	if ((data & STATUS_BYTE_MASK) == NOTE_OFF_STATUS)
	{
		_rxNoteStatus = 2;
		return;
	}

	if ((data & STATUS_BYTE_MASK) == PITCH_BEND_STATUS)
    {
        _rxNoteStatus = 4;
        return;
    }
    
	if (_rxNoteStatus == 1)
	{
		//PORTD_OUTSET = PIN1_bm;
		_noteIn = data;
		_rxNoteStatus = 3;
		return;
	}
		
	if (_rxNoteStatus == 2)
	{
		//PORTD_OUTCLR = PIN1_bm;
		noteOff(data);
		_rxNoteStatus = 0;
		return;
	}
	
	if (_rxNoteStatus == 3)
	{
		noteOn(_noteIn, data);		
		_rxNoteStatus = 0;
		return;	
	}
    
    if (_rxNoteStatus == 4) //Just going to wait for next byte because we want the MSB of the pitch bend and that byte is sent second
    {
        _rxNoteStatus = 5;
        return;
    }
    
    if (_rxNoteStatus == 5)
    {
        _pitchBendVal = data >> 1; //we want the pitch bend value to be a 6 bit number
        _rxNoteStatus = 0;
        calcPitchBends();
        return;
    }
}
 
ISR(TCA0_CMP0_vect)
{
    TCA0.SINGLE.INTFLAGS = TCA_SINGLE_CMP0_bm;
    TCA0.SINGLE.CMP0 += _voices[0].freq + _pitchBend0;
	_voices[0].waveHi ^= 0xFF;	
	_voice0Out = _voices[0].waveHi & _voices[0].velocity;	
//    PORTA.OUTTGL = PIN6_bm;
}

ISR(TCA0_CMP1_vect)
{
    TCA0.SINGLE.INTFLAGS = TCA_SINGLE_CMP1_bm;
    TCA0.SINGLE.CMP1 += _voices[1].freq + _pitchBend1;
	_voices[1].waveHi ^= 0xFF;
	_voice1Out = _voices[1].waveHi & _voices[1].velocity;
}

ISR(TCA0_CMP2_vect)
{
    TCA0.SINGLE.INTFLAGS = TCA_SINGLE_CMP2_bm;
    TCA0.SINGLE.CMP2 += _voices[2].freq + _pitchBend2;
	_voices[2].waveHi ^= 0xFF;
	_voice2Out = _voices[2].waveHi & _voices[2].velocity;
}

ISR(TCB0_INT_vect)
{
    uint16_t out = _voice0Out + _voice1Out + _voice2Out;
    TCB0.INTFLAGS = TCB_CAPTEI_bm;
    
	if (out > 0xFF)
        out = out >> 1;

    if (out > 0xFF)
        out = out >> 1;
    
    DAC0.DATA = out;
    
   
    
    if (!(PORTA.IN & PIN2_bm)) //NOT because we are triggering low
    {
        _pinDebounce++;
        

    }
    else 
    {
        
        //Add this here so that the button is only recognized on release
        if (_pinDebounce > 12000)
        {
            _displayEnabled ^= 0xFF; //Toggle the display on a long hold;
        }
        else if (_pinDebounce > 3000 && _displayEnabled)
        {
            _colorStep = 0;
            _displayMode++;
        }
          

        
        _pinDebounce = 0;
    }
    
     _updateDisplayFlag++;
    
}
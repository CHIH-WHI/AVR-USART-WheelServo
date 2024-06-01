#ifndef Arduino_h

#include <stdbool.h>

#define __AVR_ATmega328P__
#include <avr/interrupt.h>

#endif

void USART_Init(void);
bool USART_NewData(void);
uint8_t USART_Receive(void);
void USART_Transmit(uint8_t Data);
void USART_Receive_Byte(void *DataPtr, uint8_t Count);
void USART_Transmit_Byte(void *DataPtr, uint8_t Count);

bool ServoStart(uint8_t *ServoPin, uint16_t *ServoMicroseconds, uint8_t ServoCount);
bool ServoMicrosecond(uint8_t ServoIndex, uint16_t SetMicroseconds, uint16_t SpeedMicroseconds);
void ServoWait(uint8_t ServoIndex);

void WheelStart(void);
void WheelSpeed(int16_t LeftSpeed, int16_t RightSpeed);
void WheelBrake(void);

int main(void)
{
	USART_Init();
	WheelStart();

	while (true)
	{
		uint8_t String[13] = { 0 };
		USART_Transmit(USART_Receive());
	}

	return 0;
}

static struct
{
	uint8_t ReceiveBuffer[256];
	volatile uint8_t ReceiveFront;
	uint8_t ReceiveRear;
	uint8_t TransmitBuffer[256];
	uint8_t TransmitFront;
	volatile uint8_t TransmitRear;
}
USART_t;

void USART_Init(void)
{
	UBRR0 = 0;
	UCSR0A = _BV(U2X0);
	UCSR0B = _BV(RXCIE0) | _BV(RXEN0) | _BV(TXEN0);
	UCSR0C = _BV(UCSZ00) | _BV(UCSZ01); //Asynchronous USART | Disabled | 1-bit | 8-bit
	sei();
}

bool USART_NewData(void)
{
	return USART_t.ReceiveFront != USART_t.ReceiveRear ? true : false;
}

ISR(USART_RX_vect)
{
	uint8_t Data = UDR0;

	USART_t.ReceiveFront++;
	if (USART_t.ReceiveFront != USART_t.ReceiveRear)
	{
		USART_t.ReceiveBuffer[USART_t.ReceiveFront] = Data;
	}
}

ISR(USART_UDRE_vect)
{
	if (USART_t.TransmitFront != USART_t.TransmitRear)
	{
		USART_t.TransmitRear++;
		UDR0 = USART_t.TransmitBuffer[USART_t.TransmitRear];
	}
	else
	{
		UCSR0B &= ~_BV(UDRIE0);
	}
}

uint8_t USART_Receive(void)
{
	UCSR0B &= ~_BV(RXCIE0);

	if (USART_t.ReceiveFront == USART_t.ReceiveRear)
	{
		while (!(UCSR0A & _BV(RXC0)));
		uint8_t Data = UDR0;
		UCSR0B |= _BV(RXCIE0);
		return Data;
	}
	else
	{
		UCSR0B |= _BV(RXCIE0);

		USART_t.ReceiveRear++;
		return USART_t.ReceiveBuffer[USART_t.ReceiveRear];
	}
}

void USART_Transmit(uint8_t Data)
{
	UCSR0B &= ~_BV(UDRIE0);

	if (USART_t.TransmitFront == USART_t.ReceiveRear && UCSR0A & _BV(UDRE0))
	{
		UDR0 = Data;
	}
	else
	{
		USART_t.TransmitFront++;
		while (USART_t.TransmitFront == USART_t.TransmitRear);

		USART_t.TransmitBuffer[USART_t.TransmitFront] = Data;
		UCSR0B |= _BV(UDRIE0);
	}
}

void USART_Transmit_Byte(void *DataPtr, uint8_t Count)
{
	for (uint8_t Index = 0; Index < Count; Index++)
	{
		USART_Transmit(((uint8_t*)DataPtr)[Index]);
	}
}

void USART_Receive_Byte(void *DataPtr, uint8_t Count)
{
	for (uint8_t Index = 0; Index < Count; Index++)
	{
		((uint8_t*)DataPtr)[Index] = USART_Receive();
	}
}

#define CPU_HZ 16000000L

#define SERVO_PERIOD 20000
#define SERVO_UNIT 2500
#define SERVO_MAX_WIDTH 2200
#define SERVO_MIN_WIDTH 800
#define SERVO_COUNT_MAX (SERVO_PERIOD / SERVO_UNIT)
#define TICKS(Width) (CPU_HZ / 1000000L / 8 * Width)

static struct
{
	volatile uint8_t Level;
	volatile uint8_t Index;
	uint8_t Count;

	struct
	{
		volatile uint8_t *pPORTxn;
		uint8_t Bit;
		volatile uint16_t NowTick;
		uint16_t SetTick;
		uint16_t SpeedTick;
	}Pin[SERVO_COUNT_MAX];
}
Servo_t;

bool ServoStart(uint8_t *ServoPin, uint16_t *ServoMicroseconds, uint8_t ServoCount)
{
	for (uint8_t Index = 0; Index < ServoCount; Index++)
	{
		volatile uint8_t *pDDxn;
		volatile uint8_t *pPORTxn;
		uint8_t Bit;

		uint8_t Pin = ServoPin[Index];
		if (8 > Pin)
		{
			pDDxn = &DDRD;
			pPORTxn = &PORTD;
			Bit = _BV(Pin);
		}
		else if (14 > Pin)
		{
			pDDxn = &DDRB;
			pPORTxn = &PORTB;
			Bit = _BV(Pin - 8);
		}
		else if (20 > Pin)
		{
			pDDxn = &DDRC;
			pPORTxn = &PORTC;
			Bit = _BV(Pin - 14);
		}
		else // PinIndex >= 20
		{
			return false;
		}

		uint16_t Microseconds = ServoMicroseconds[Index];
		if (Microseconds < SERVO_MIN_WIDTH || SERVO_MAX_WIDTH < Microseconds)
		{
			return false;
		}

		*pDDxn |= Bit;
		Servo_t.Pin[Index].pPORTxn = pPORTxn;
		Servo_t.Pin[Index].Bit = Bit;
		Servo_t.Pin[Index].SetTick = TICKS(Microseconds);
		Servo_t.Pin[Index].NowTick = TICKS(Microseconds);
		Servo_t.Pin[Index].SpeedTick = 0;
	}

	Servo_t.Level = 0;
	Servo_t.Index = 0;
	Servo_t.Count = ServoCount;

	TCCR1A = 0;
	TCCR1B = _BV(CS11);
	TCNT1 = 0;
	TIFR1 |= _BV(OCF1A);
	TIMSK1 |= _BV(OCIE1A);

	return true;
}

ISR(TIMER1_COMPA_vect)
{
	if (0 == Servo_t.Level) //LOW == Servo_t.Level
	{
		if (0 == Servo_t.Index)
		{
			OCR1A = 0;
			TCNT1 = 0;
		}
		*Servo_t.Pin[Servo_t.Index].pPORTxn |= Servo_t.Pin[Servo_t.Index].Bit;

		if (0 == Servo_t.Pin[Servo_t.Index].SpeedTick)
		{
			OCR1A += Servo_t.Pin[Servo_t.Index].SetTick;
		}
		else
		{
			int16_t DiffTick = Servo_t.Pin[Servo_t.Index].NowTick - Servo_t.Pin[Servo_t.Index].SetTick;
			int16_t SpeedTick = (int16_t)Servo_t.Pin[Servo_t.Index].SpeedTick;
			if (-DiffTick > SpeedTick)
			{
				Servo_t.Pin[Servo_t.Index].NowTick += Servo_t.Pin[Servo_t.Index].SpeedTick;
			}
			else if (DiffTick > SpeedTick)
			{
				Servo_t.Pin[Servo_t.Index].NowTick -= Servo_t.Pin[Servo_t.Index].SpeedTick;
			}
			else //abs(DiffTick) < SpeedTick
			{
				Servo_t.Pin[Servo_t.Index].NowTick = Servo_t.Pin[Servo_t.Index].SetTick;
			}
			OCR1A += Servo_t.Pin[Servo_t.Index].NowTick;
		}
		Servo_t.Level = 1;
	}
	else //HIGH == Servo_t.Level
	{
		*Servo_t.Pin[Servo_t.Index].pPORTxn &= ~Servo_t.Pin[Servo_t.Index].Bit;
		if (Servo_t.Count == Servo_t.Index + 1)
		{
			Servo_t.Index = Servo_t.Index + 1;
			OCR1A += TICKS(SERVO_UNIT) - Servo_t.Pin[Servo_t.Index++].NowTick;
		}
		else
		{
			Servo_t.Index = 0;
			OCR1A = TICKS(SERVO_PERIOD);
		}
		Servo_t.Level = 0;
	}
}

bool ServoMicrosecond(uint8_t ServoIndex, uint16_t SetMicroseconds, uint16_t SpeedMicroseconds)
{
	if (SetMicroseconds < SERVO_MIN_WIDTH || SERVO_MAX_WIDTH < SetMicroseconds)
	{
		return false;
	}

	TIMSK1 &= ~_BV(OCIE1A);
	Servo_t.Pin[ServoIndex].SetTick = TICKS(SetMicroseconds);
	Servo_t.Pin[ServoIndex].SpeedTick = TICKS(SpeedMicroseconds);
	TIMSK1 |= _BV(OCIE1A);

	return true;
}

void ServoWait(uint8_t ServoIndex)
{
	if (0 == Servo_t.Pin[ServoIndex].SpeedTick)
	{
		return;
	}

	while (Servo_t.Pin[ServoIndex].SetTick != Servo_t.Pin[ServoIndex].NowTick);
}

#define MID_Address (0 << 1)

static struct
{
	uint8_t MID;
	uint8_t CID;
	uint8_t CheckSum1;
	uint8_t DutyCycleA_L;
	uint8_t DutyCycleA_H;
	uint8_t DutyCycleB_L;
	uint8_t DutyCycleB_H;
	uint8_t CheckSum2;
	uint8_t Dummy;
}
SetVelAB = { MID_Address, 118U, 137U };

static const struct
{
	uint8_t MID;
	uint8_t CID;
	uint8_t CheckSum1;
	uint8_t Dummy;
}
BrakeDual = { MID_Address, 101U, 154U };

static struct
{
	volatile uint8_t *DataAddress;
	uint8_t *DataAddressEnd;
}
TWI_t;

void WheelStart(void)
{
	PORTC |= _BV(PORTC4) | _BV(PORTC5);
	TWBR = 12; //SCL frequency = CPU Clock frequency / (16 + 2 * TWBR * PrescalerValue)
	TWCR = _BV(TWEN);
}

ISR(TWI_vect)
{
	switch (TWSR) //Prescaler = 0
	{
	case 0x08: //A START condition has been transmitted
	case 0x10: //A repeated START condition has been transmitted
	case 0x18: //SLA+W has been transmitted; ACK has been received
	case 0x28: //Data byte has been transmitted; ACK has been received
		if (TWI_t.DataAddress != TWI_t.DataAddressEnd)
		{
			TWDR = *TWI_t.DataAddress++;
			TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWIE);
		}
		else
		{
			TWCR = _BV(TWINT) | _BV(TWSTO) | _BV(TWEN);
		}
		break;

	case 0x38: //Arbitration lost in SLA+W or data bytes
		TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN) | _BV(TWIE);
		break;

	case 0x20: //SLA+W has been transmitted; NOT ACK has been received
	case 0x30: //Data byte has been transmitted; NOT ACK has been received
	default:
		TWCR = _BV(TWEN);
		break;
	}
}

void WheelSpeed(int16_t LeftSpeed, int16_t RightSpeed)
{
	*(int16_t*)&SetVelAB.DutyCycleA_L = LeftSpeed;
	*(int16_t*)&SetVelAB.DutyCycleB_L = RightSpeed;
	SetVelAB.CheckSum2 = 255 - (SetVelAB.DutyCycleA_L + SetVelAB.DutyCycleA_H + SetVelAB.DutyCycleB_L + SetVelAB.DutyCycleB_H);

	while (TWCR & _BV(TWIE));

	TWI_t.DataAddress = (volatile uint8_t*)&SetVelAB;
	TWI_t.DataAddressEnd = (uint8_t*)&SetVelAB + sizeof(SetVelAB);

	TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN) | _BV(TWIE);
}

void WheelBrake(void)
{
	while (TWCR & _BV(TWIE));

	TWI_t.DataAddress = (volatile uint8_t*)&BrakeDual;
	TWI_t.DataAddressEnd = (uint8_t*)&BrakeDual + sizeof(BrakeDual);

	TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN) | _BV(TWIE);
}

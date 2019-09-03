//uart2.cpp

/**
 * Robert McKenzie 7 byte payload comms protol with 0x7F as start byte
 * and 0xF7 as finish bytes.
 * ACK is with 0x06
 * NACK is 0x15
 */

#include "uart2.h"
#include "system_timer.h"

volatile unsigned char readRxBuffer, rxData1 = 0, rxData2 = 0, rxData3 = 0,
                                     rxData4 = 0, rxData5 = 0;
volatile bool confirmedPayload = false, txNAKNext = false,
              txACKNext = false, txRESEND = false, pendingACK = false, MMU_IRSENS = false;
enum class rx
{
    Idle,
    Data1,
    Data2,
    Data3,
    Data4,
    Data5,
    End
};

inline rx& operator++(rx& byte, int)
{
    const int i = static_cast<int>(byte) + 1;
    byte = static_cast<rx>((i) % 7);
    return byte;
}

rx rxCount = rx::Idle;
unsigned char lastTxPayload[] = {0, 0, 0, 0, 0}; // used to try resend once

void uart2_init(void)
{
    cli();
    UCSR2A = (0 << U2X2); // baudrate multiplier
    UCSR2B = (1 << RXEN2) | (1 << TXEN2) | (0 << UCSZ22); // enable receiver and transmitter
    UCSR2C = (0 << UMSEL21) | (0 << UMSEL20) | (0 << UPM21) |
             (0 << UPM20) | (1 << USBS2) |(1 << UCSZ21) | (1 << UCSZ20); // Use 8-bit character sizes

    UBRR2H = (BAUD_PRESCALE >> 8); // Load upper 8-bits of the baud rate value into the high byte of the UBRR register
    UBRR2L = BAUD_PRESCALE; // Load lower 8-bits of the baud rate value into the low byte of the UBRR register

    UCSR2B |= (1 << RXCIE2); // enable rx interrupt
    sei();
}

ISR(USART2_RX_vect)
{
    cli();
    readRxBuffer = UDR2;
    switch (rxCount) {
    case rx::Idle:
        if (readRxBuffer == 0x7F) rxCount++;
        if (readRxBuffer == 0x06) pendingACK = false;
        if (readRxBuffer == 0x15) txRESEND = true;
        break;
    case rx::Data1:
        rxData1 = readRxBuffer;
        rxCount++;
        break;
    case rx::Data2:
        rxData2 = readRxBuffer;
        rxCount++;
        break;
    case rx::Data3:
        rxData3 = readRxBuffer;
        rxCount++;
        break;
    case rx::Data4:
        rxData4 = readRxBuffer;
        rxCount++;
        break;
    case rx::Data5:
        rxData5 = readRxBuffer;
        rxCount++;
        break;
    case rx::End:
        if (readRxBuffer == 0xF7) { confirmedPayload = true; txACKNext = true;
        if (rxData1 == 'I' && rxData2 == 'R' && rxData3 == 'S' && rxData4 == 'E' && rxData5 == 'N') MMU_IRSENS = true; }
        else txNAKNext = true;
        rxCount = rx::Idle;
        break;
    }
    sei();
}

void uart2_txPayload(unsigned char payload[], bool retry)
{
#ifdef MMU_DEBUG
    printf_P(PSTR("\nUART2 TX 0x%2X %2X %2X\n"), payload[0], payload[1], payload[2]);
#endif //MMU_DEBUG
    if (retry) { // Allow only one retry then give up
        confirmedPayload = false;
        txRESEND         = false;
        if (lastTxPayload == payload || (payload[0] == 'P' && payload[1] == '0')) {
            pendingACK = false;
            return;
        }
    }
    mmu_last_request = _millis();
    for (uint8_t i = 0; i < 5; i++) lastTxPayload[i] = payload[i];
    loop_until_bit_is_set(UCSR2A, UDRE2);     // Do nothing until UDR is ready for more data to be written to it
    UDR2 = 0x7F;
    for (uint8_t i = 0; i < 5; i++) {
        loop_until_bit_is_set(UCSR2A, UDRE2); // Do nothing until UDR is ready for more data to be written to it
        if (!txRESEND) UDR2 = (0xFF & (int)payload[i]);
    }
    loop_until_bit_is_set(UCSR2A, UDRE2);     // Do nothing until UDR is ready for more data to be written to it
    UDR2 = 0xF7;
    pendingACK = true;
}

void uart2_txACK(bool ACK)
{
    confirmedPayload = false;
    if (ACK) {
        loop_until_bit_is_set(UCSR2A, UDRE2); // Do nothing until UDR is ready for more data to be written to it
        UDR2 = 0x06; // ACK HEX
        confirmedPayload = false;
        txACKNext = false;
    } else {
        loop_until_bit_is_set(UCSR2A, UDRE2); // Do nothing until UDR is ready for more data to be written to it
        UDR2 = 0x15; // NACK HEX
        txNAKNext = false;
    }
}

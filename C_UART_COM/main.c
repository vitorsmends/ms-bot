#include <msp430.h>

volatile int state = 0;

void interfaceSetup();
void serialSetup(int baudrate_mode);
void stateMachine();
void serialOutput(int state);

#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void) {
    if (P1IFG & BIT3) { // Verifica se a interrup��o foi causada pelo bot�o
        __delay_cycles(50000); // Debouncing
        state++;
        if (state > 3) {
            state = 0; // Reinicia state para 0 ap�s atingir 4
        }
        P1IFG &= ~BIT3; // Limpa a flag de interrup��o do bot�o
    }
}

void main(void) {
    WDTCTL = WDTPW | WDTHOLD;
    interfaceSetup();
    serialSetup(52);
    __enable_interrupt(); // Habilita interrup��es globais

    while (1) {
        if (P1IN & BIT3) {
            P1OUT &= ~BIT0; // Desliga o LED vermelho se o bot�o n�o est� pressionado
        } else {
            P1OUT |= BIT0; // Acende o LED vermelho se o bot�o est� pressionado
        }
        stateMachine(state);
    }
}

void interfaceSetup() {
    P1DIR |= BIT0;
    P1DIR &= ~BIT3;
    P1REN |= BIT3;
    P1OUT |= BIT3;
    P1OUT &= ~(BIT0);

    P1IE |= BIT3; // Habilita interrup��o no bot�o
    P1IES |= BIT3; // Configura interrup��o para borda de descida
    P1IFG &= ~BIT3; // Limpa a flag de interrup��o do bot�o
}

void serialSetup(int baudrate_mode){
    BCSCTL1 = CALBC1_1MHZ;  // Configura a frequ�ncia do clock para 1MHz
    DCOCTL = CALDCO_1MHZ;

    P1SEL = BIT1 | BIT2;  // Configura os pinos P1.1 (RX) e P1.2 (TX) para comunica��o serial
    P1SEL2 = BIT1 | BIT2;

    UCA0CTL1 |= UCSWRST + UCSSEL_2;  // Configura o m�dulo de comunica��o serial UCA0
    UCA0BR0 = baudrate_mode;  // Configura os divisores de frequ�ncia para 19200 baud
    UCA0BR1 = 0;
    UCA0MCTL = UCBRS_0;
    UCA0CTL1 &= ~UCSWRST;  // Libera o m�dulo de comunica��o serial do reset
}

void stateMachine(int state){
    serialOutput(state);
    // states:
    //          1: go ahead
    //          2: square trajectory
    //          3: sphere trajectory
}

void serialOutput(int state){
    UCA0TXBUF = state;

}


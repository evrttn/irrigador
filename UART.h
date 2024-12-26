/*
 *CONFIGURATION: BAUD RATE   -   9600
 *              NBITS       -   8
 *              STOP BITS   -   1 
 */

void UART0_config()
{
	//Baud Rate de 9600bps para um cristal de 16MHz (Datasheet)
	UBRR0 = 103;
	
	//Habilita a interrupcao de recepcao e os pinos TX e RX
	UCSR0B =  (1<<RXEN0) | (1<<TXEN0) | (1<<RXCIE0) ;
	
	//Configura a UART com 8 bits de dados
	UCSR0C =  (1<<UCSZ01) | (1<<UCSZ00);
}

void UART0_enviaCaractere(unsigned char ch)
{
	UDR0 = ch;

	//Aguarda o buffer ser desocupado
	while (! (UCSR0A & (1<<UDRE0)) );
}

void UART0_enviaString(char *s)
{
	unsigned int i=0;
	while (s[i] != '\0')
	{
		UART0_enviaCaractere(s[i++]);
	};
}

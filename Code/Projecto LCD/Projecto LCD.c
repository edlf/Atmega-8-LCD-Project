#include <avr\io.h>
#include <avr\interrupt.h>
#include <avr\pgmspace.h>
#include <util\delay.h>
#include <inttypes.h>
#include <compat/twi.h>
#include <string.h>
#include <stdlib.h>
#include "fonts.h"
#include "image.h"

// -DF_CPU=14745600UL
// Clock Speed
#define FOSC             14745600
// UART Baud rate
#define BAUD             57600
//I2C Defines
#define FSCL             (204800)
#define LCD_ADDRESS      (0x78)
#define I2C_READ         (1)
#define I2C_WRITE        (0)
#define i2c_read         (ack)  (ack) ? i2c_readAck() : i2c_readNak(); 
//LCD Defines
#define BUFFER_WIDTH     (96)
#define BUFFER_HEIGHT    (40)
#define BUFFER_SIZE      (BUFFER_WIDTH*(BUFFER_HEIGHT/8)) //Divide by 8 (storage in 8 bit)
#define LCD_PIXEL_AR     (1.25) //Pixel not a perfect square
#define LCD_PIXEL_AR_I   ((uint8_t)(LCD_PIXEL_AR*64))
#define LCD_XOR          (2)
#define LCD_SET          (1)
#define LCD_CLEAR        (0)
#define LCD_X_CHARS      (15) // 16-1 scroll bar
#define LCD_Y_CHARS      (4)  //  5-1 status bar
// RESET is active low
#define STE_RESET_PIN PINC2
#define STE_RESET_PORT PORTC
#define STE_RESET_DDR DDRC
// DC high=write to ram    DC low=command
#define DC               (64)
// CO high=command    CO low=stream
#define CO               (128)
#define FLIP_Y
// values for MX MY
// Since those are overwritten on page select, it is good to set them ok each time.
#ifdef FLIP_Y
	#define PageSelectorMXMY (_BV(5)+_BV(3))
	#define YBANKSTART (2)
	#define XSTART (6)
#else
	#define PageSelectorMXMY (_BV(5))
	#define XSTART (6)
#endif

//Delays
void          long_delay_ms           (uint16_t t);
void          long_delay_s            (uint8_t t);
//UART Stuff
void          init_usart              (void);
void          send_char               (uint8_t b);
uint8_t       read_char               (void);
void          write_str               (char * p);
void          write_enter             (void);
void          write_hex               (uint8_t b);
//I2C
void          i2c_init                (void);
unsigned char i2c_start               (unsigned char address);
void          i2c_start_wait          (unsigned char address);
unsigned char i2c_rep_start           (unsigned char address);
void          i2c_stop                (void);
unsigned char i2c_write               (unsigned char data);
unsigned char i2c_readAck             (void);
unsigned char i2c_readNak             (void);
void          i2c_write_check         (uint8_t v);
//LCD
void          LCD_Init                (void);
void          LCD_Reset               (void);
void          LCD_clearFrameBuffer    (void);
void          LCD_displayFrameBuffer  (void);
void          LCD_setPixelXY          (uint8_t x,uint8_t y,uint8_t mode);

void          start_write             (void);
void          setFunction             (uint8_t page,uint8_t cmd);
void          command                 (uint8_t command_in);
void          prepare_write           (uint8_t x,uint8_t y);

//LCD Draw
void          LCD_drawLine            (uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t mode);
#define       LCD_drawRect            (x1, y1, x2, y2, mode) LCD_drawLine(x1,y1,x2,y1,mode); LCD_drawLine(x2,y1,x2,y2,mode); LCD_drawLine(x2,y2,x1,y2,mode); LCD_drawLine(x1,y2,x1,y1,mode);
void          LCD_fillRect            (uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2,uint8_t mode);
uint8_t       clampX                  (uint8_t x);
uint8_t       clampY                  (uint8_t y);

//LCD Write Strings
void          LCD_char                (uint8_t* fontSet, uint8_t ch,uint8_t x,uint8_t y,uint8_t mode); 
void          LCD_drawString          (uint8_t fontSet, uint8_t* buf, uint8_t is_pstr,uint8_t x,uint8_t y,uint8_t mode) ;
void          LCD_text_mode           (void);
void          LCD_text_mode_shiftar   (uint8_t shift_n_chars,uint8_t shift_direction, uint8_t posicao_cursor);
void          LCD_text_status         (void);

void          LCD_print_image         (void);

void          LCD_cleartextBuffer     (void);
void          preenche_ecra           (uint8_t text_page);
void		  LCD_desenha_cursor      (void);

//Debug
void dump_framebuffer(void);


uint8_t frameBuffer [BUFFER_SIZE];
char    textBuffer  [(LCD_X_CHARS*LCD_Y_CHARS*3)]; // 3 pages
uint8_t posicao_cursor=0, text_page=0, status=0, linha, coluna; 
uint16_t pisca=0;

int     main(){
	init_usart();
	static char ok[] = " Ok";
	static char error[] = " Error";
	static char step_one[] = "I2C Init:";
	static char step_two[] = "LCD Init:";
	char main_menu="Escolha o modo:", mm_invalid="Opccao Invalida!", op1="1) Modo de Texto", op2="2) Mostrar imagem", op8="8) Limpar LCD";
		
	DDRC=0xFF;
	DDRB=0xFF;
	PORTB=~0xF;
	
	write_str(&step_one);
	i2c_init();
	write_str(&ok);
	write_enter();
	write_str(&step_two);
	LCD_Init();
	write_str(&ok);
	write_enter();
	
	while(1){				
		int input=0;
		write_str(op1);
		write_enter();
		write_str(op2);
		write_enter();
		write_str(op8);
		write_enter();
		write_str(main_menu);	
		input=read_char();
		send_char(input);
		write_enter();
		input=input-48;
		status=input;
		switch(input){
			case 8 :
				LCD_clearFrameBuffer();
				LCD_displayFrameBuffer();
				break;
			case 2:
				LCD_print_image();
				break;
			case 1 :
				write_enter();
				LCD_text_mode();
				break;
			default:
				write_str(mm_invalid);
				write_enter();
		}
	}
	return 0;
}

//DELAYS
void long_delay_ms(uint16_t t){
	t=t/16;
	while(t-->0){
		_delay_ms(16);
	}
};

void long_delay_s(uint8_t t){
	while(t-->0){
		long_delay_ms(1000);
	}
};

//UART
void    init_usart(void){
    //57600 baud, No Parity, 8 Data, 1 Stop Bit (15)
    UBRRH=0x00;
    UBRRL=0x0F;
    UCSRB = (1<<RXEN)|(1<<TXEN);
    UCSRC = (1<<URSEL)|(0<<USBS)|(3<<UCSZ0);
	//Clean boot loader magic
	char clean[]= {8,13,0};
    write_str(&clean);
};

void    send_char(uint8_t b){
    while ( !( UCSRA & (1<<UDRE)) );
    UDR = b;
}

uint8_t read_char(void){
    while ( !(UCSRA & (1<<RXC)) );
    return UDR;
}

void    write_str(char * p){
    while(*p!=0){
        send_char (*p);
        p++;
    };
}

void    write_enter(void){
	char enter[3]= {10,13,0};
    write_str(enter);
};

//I2C
void i2c_init(void){
  TWSR = 0;                   /* no prescaler */
  TWBR = ((FOSC/FSCL)-16)/2;  /* must be > 10 for stable operation */
};

unsigned char i2c_start(unsigned char address){
    uint8_t   twst;

	// send START condition
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);

	// wait until transmission completed
	while(!(TWCR & (1<<TWINT)));

	// check value of TWI Status Register. Mask prescaler bits.
	twst = TW_STATUS & 0xF8;
	if ( (twst != TW_START) && (twst != TW_REP_START)) return 1;

	// send device address
	TWDR = address;
	TWCR = (1<<TWINT) | (1<<TWEN);

	// wail until transmission completed and ACK/NACK has been received
	while(!(TWCR & (1<<TWINT)));

	// check value of TWI Status Register. Mask prescaler bits.
	twst = TW_STATUS & 0xF8;
	if ( (twst != TW_MT_SLA_ACK) && (twst != TW_MR_SLA_ACK) ) return 1;
	return 0;
};

void i2c_start_wait(unsigned char address){
    uint8_t   twst;
    while ( 1 ){
	    // send START condition
	    TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
    
    	// wait until transmission completed
    	while(!(TWCR & (1<<TWINT)));
    
    	// check value of TWI Status Register. Mask prescaler bits.
    	twst = TW_STATUS & 0xF8;
    	if ( (twst != TW_START) && (twst != TW_REP_START)) continue;
    
    	// send device address
    	TWDR = address;
    	TWCR = (1<<TWINT) | (1<<TWEN);
    
    	// wail until transmission completed
    	while(!(TWCR & (1<<TWINT)));
    
    	// check value of TWI Status Register. Mask prescaler bits.
    	twst = TW_STATUS & 0xF8;
    	if ( (twst == TW_MT_SLA_NACK )||(twst ==TW_MR_DATA_NACK) ) {    	    
    	    /* device busy, send stop condition to terminate write operation */
	        TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
	        
	        // wait until stop condition is executed and bus released
	        while(TWCR & (1<<TWSTO)); 
    	    continue;
    	}
    	//if( twst != TW_MT_SLA_ACK) return 1;
    	break;
     }
};

unsigned char i2c_rep_start(unsigned char address){
    return i2c_start( address );
};

void i2c_stop(void){
    /* send stop condition */
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
	// wait until stop condition is executed and bus released
	while(TWCR & (1<<TWSTO));
};

unsigned char i2c_write(unsigned char data){
    uint8_t   twst;
    
	// send data to the previously addressed device
	TWDR = data;
	TWCR = (1<<TWINT) | (1<<TWEN);

	// wait until transmission completed
	while(!(TWCR & (1<<TWINT)));

	// check value of TWI Status Register. Mask prescaler bits
	twst = TW_STATUS & 0xF8;
	if( twst != TW_MT_DATA_ACK) return 1;
	return 0;
}

unsigned char i2c_readAck(void){
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
	while(!(TWCR & (1<<TWINT)));    
    return TWDR;
};

unsigned char i2c_readNak(void){
	TWCR = (1<<TWINT) | (1<<TWEN);
	while(!(TWCR & (1<<TWINT)));
    return TWDR;
};

void i2c_write_check(uint8_t v){
	if (i2c_write(v)!=0)
		while(1){
			PORTB=0xFF;
			long_delay_ms(200);
			PORTB=0;
			long_delay_ms(200);
		}
};

void LCD_Init(void){
	STE_RESET_DDR|=_BV(STE_RESET_PIN);	// RESET pin output
	LCD_Reset();
	LCD_clearFrameBuffer();
	LCD_displayFrameBuffer();
};

void LCD_Reset(void){
	uint8_t t;
	STE_RESET_PORT&=~_BV(STE_RESET_PIN); 	// set RESET to LOW to reset lcd.
	long_delay_ms(200);
	STE_RESET_PORT|=_BV(STE_RESET_PIN);		// set RESET to high to activate lcd.
	long_delay_ms(200);
	i2c_start_wait(LCD_ADDRESS+I2C_WRITE);  // set device address and write mode
	PORTB=(uint8_t)~0xF1;
	setFunction(0,_BV(3)|_BV(2));           // lcd normal mode
	setFunction(1,_BV(0));                  // checkerboard
	_delay_ms(40);
	setFunction(1,_BV(3)+_BV(2));           // DO=1 LSB first
	setFunction(0,_BV(2)+2+1);              // PRS VLDC 10.62v
	setFunction(0,_BV(4)+2);                // Charge pump X4
	setFunction(1,_BV(4)+4);                // Bias ratio 3

#ifdef FLIP_Y
	setFunction(0,_BV(1));                  // scroll to adjust 1 line when in Y flip
#endif

	i2c_write_check(CO);                    // NO commands follow, command mode (CO=0 DC=0)
	i2c_write_check(PageSelectorMXMY);      // page 0
	i2c_stop();                
	i2c_start_wait(LCD_ADDRESS+I2C_READ);   // set device address and write mode
	t=i2c_readNak();
	i2c_stop();   
	i2c_start_wait(LCD_ADDRESS+I2C_WRITE);  // set device address and write mode
	setFunction(0,_BV(6)+0x2);              // set bank (Y)
	setFunction(0,_BV(7)+0x0);              // set X
	i2c_write_check(CO);                    // NO commands follow, command mode (CO=0 DC=0)
	i2c_write_check(PageSelectorMXMY);      // page 0
	i2c_stop();                
};

void LCD_clearFrameBuffer(void){
	uint16_t cnt=0;
	while(cnt<BUFFER_SIZE) frameBuffer[cnt++]=0; 
};

void LCD_cleartextBuffer(void){
	uint16_t cnt=0;
	while(cnt<(LCD_X_CHARS*LCD_Y_CHARS*3)) textBuffer[cnt++]=32; 
};

void LCD_displayFrameBuffer(void){
	uint8_t bank,x;
	uint16_t p;
	bank=0;
	while(bank<5){
		p=BUFFER_WIDTH*bank;
		prepare_write(0,bank);
		_delay_ms(1);
		start_write();
		x=0;
		while(x++<BUFFER_WIDTH) i2c_write_check(frameBuffer[p++]);
		i2c_stop();                
		_delay_ms(1);
		bank++;
	}
};

void LCD_setPixelXY(uint8_t x,uint8_t y,uint8_t mode){
	uint8_t bank;
	if ((x>=BUFFER_WIDTH)||(y>=BUFFER_HEIGHT)) return;
	bank=y>>3;
	y=y-(bank<<3);
	if (mode==LCD_SET)
		frameBuffer[bank*BUFFER_WIDTH+x]|=1<<y;
	else
	if (mode==LCD_CLEAR)
		frameBuffer[bank*BUFFER_WIDTH+x]&=(uint8_t)~(1<<y);
	else 
	//if (mode==LCD_XOR)
		frameBuffer[bank*BUFFER_WIDTH+x]^=(1<<y);
};

void start_write(void){
 	i2c_start_wait(LCD_ADDRESS+I2C_WRITE);     // set device address and write mode
	i2c_write_check(DC); // stream of bytes (CO=0 DC=1)
};

void setFunction(uint8_t page,uint8_t cmd){
	command(PageSelectorMXMY+page);
	command(cmd);
};

void command(uint8_t command_in){
	i2c_write_check(0); // more commands follow, command mode (CO=0 DC=0)
	i2c_write_check(command_in); // page 1, more commands follow, command mode (CO=0 DC=0)
};

void prepare_write(uint8_t x,uint8_t y){
	i2c_start_wait(LCD_ADDRESS+I2C_WRITE);     // set device address and write mode
	i2c_write_check(0);
	if (y>=2) y++; // skip bank 3, because it is not displayed...
	i2c_write_check(_BV(6)+(YBANKSTART+y)); // set bank (Y)
	i2c_write_check(0);
	i2c_write_check(_BV(7)+(XSTART+x)); // set X
	i2c_stop();                
};

void LCD_drawLine ( uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t mode ){
    int dx, dy, stepx, stepy, fraction;
    dy = y2 - y1;
    dx = x2 - x1;
    if ( dy < 0 ){
        dy    = -dy;
        stepy = -1;
    }
    else{
        stepy = 1;
    }
    if ( dx < 0 ){
        dx    = -dx;
        stepx = -1;
    }
    else{
        stepx = 1;
    }
    dx <<= 1;
    dy <<= 1;
    LCD_setPixelXY( x1, y1, mode );
    if ( dx > dy ){
        fraction = dy - (dx >> 1);
        while ( x1 != x2 ){
            if ( fraction >= 0 ){
                y1 += stepy;
                fraction -= dx;
            }
            x1 += stepx;
            fraction += dy;
            LCD_setPixelXY( x1, y1, mode );
        }
    }
    else{
        fraction = dx - (dy >> 1);
        while ( y1 != y2 ){
            if ( fraction >= 0 ){
                x1 += stepx;
                fraction -= dy;
            }
            y1 += stepy;
            fraction += dx;
            LCD_setPixelXY( x1, y1, mode );
        }
    }
};

uint8_t clampX(uint8_t x){
	if (x>=BUFFER_WIDTH) return BUFFER_WIDTH-1;
	return x;
};

uint8_t clampY(uint8_t y){
	if (y>=BUFFER_HEIGHT) return BUFFER_HEIGHT-1;
	return y;
};

void LCD_fillRect(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2,uint8_t mode){
	uint8_t t;
	x1=clampX(x1);
	x2=clampX(x2);
	y1=clampY(y1);
	y2=clampY(y2);
	if (x2<x1) { t=x1; x1=x2; x2=t; } 
	if (y2<y1) { t=y1; y1=y2; y2=t; } 
	while(y1<=y2){
		LCD_drawLine(x1,y1,x2,y1,mode);
		y1++;
	}	
};

void LCD_char(uint8_t* fontSet, uint8_t ch,uint8_t x,uint8_t y,uint8_t mode) {
	uint8_t i, j;
	uint8_t k  = pgm_read_byte(fontSet);
	uint8_t xx = pgm_read_byte(fontSet+1);
	uint8_t yy = pgm_read_byte(fontSet+2);
	uint8_t px=xx;
	uint8_t py=yy;
	if(ch < k){
		for(i=0; i<px; i++){
			for(j=0; j<py; j++){
				if(pgm_read_byte(fontSet+(xx*ch+3)+(i)) & _BV((j))){ 
					LCD_setPixelXY(x,y+j,mode);
				}
				else{
					LCD_setPixelXY(x,y+j,0);	
				}						
			}
			x++;
		}
	}
	else {
		x+=1;
	}
};

void LCD_drawString(uint8_t fontSet, uint8_t* buf, uint8_t is_pstr,uint8_t x,uint8_t y,uint8_t mode) {
	uint8_t i=0, offset=32, ch;
	uint8_t xx = pgm_read_byte(fontSet+1);

	// check if string is in flash/sram and read appropriately
	if(is_pstr) ch = pgm_read_byte(buf+i);
	else ch = buf[i];
	do {
		if(ch > 31 && ch < 123) {
			LCD_char(fontSet, ch-offset,x,y,mode);
		}
		else {
			LCD_char(fontSet, 255,x,y,mode);	// empty (not recognized)
		}
		x+=xx+1; // char spacing
		i++;
		// check if string is in flash/sram and read appropriately
		if(is_pstr) ch = pgm_read_byte(buf+i);
		else  ch = buf[i];
	} while(ch != '\0');
};

void write_hex(uint8_t b) {
	uint8_t a;
	a = b >> 4;
	if (a <= 9 )
	  send_char(a + 0x30);
	else
	  send_char(a + 0x37);
	a = b & 0xF;
	if (a <= 9 ) 
	  send_char(a + 0x30);
	else 
	  send_char(a + 0x37);
};

void dump_framebuffer(void){
	int count=0;
	char temp,fbc="Framebuffer contents:";
	write_str(fbc);
	write_enter();
	while (count<BUFFER_SIZE){
		temp=frameBuffer[count];
		write_hex(temp);
		count++;
	}
	write_enter();
};

void LCD_text_mode(void){
	sei();
	//Configurar interrupt para piscar o cursor
	TCCR0=00000101; //ultimos 3 prescaler
	TIMSK=00000001; //activar timer em overflow
	LCD_clearFrameBuffer();
	LCD_cleartextBuffer();
	
	text_page=0;
	posicao_cursor=0;
	char uart_buffer;

	while(1){
		uart_buffer=read_char();
		send_char(uart_buffer);
		//Backspace
		if (uart_buffer==8){
			if (posicao_cursor>=0){
				LCD_text_mode_shiftar(1,0,posicao_cursor-1);
			}
			if(posicao_cursor>0){
				posicao_cursor--;
			}
		}
		//Enter
		if (uart_buffer==13){
			send_char(uart_buffer);
			LCD_text_mode_shiftar(posicao_cursor%15,1,((posicao_cursor/15)+1)*15);
			LCD_text_mode_shiftar(15-(posicao_cursor%15),1,posicao_cursor);
			posicao_cursor=((posicao_cursor/15)+1)*15;
			uart_buffer=8;
		}
		//Escape (cursor + saida)
		if (uart_buffer==27){
			uart_buffer=read_char();
			if (uart_buffer==0x7E){ //Delete
				LCD_text_mode_shiftar(1,0,posicao_cursor);
			}
			if (uart_buffer==113) {
				cli();
				send_char(65);
				LCD_clearFrameBuffer();
				LCD_displayFrameBuffer();
				write_enter();
				break;
			}
			if (uart_buffer==101) {
				LCD_cleartextBuffer();
				posicao_cursor=0;
			}
			if (uart_buffer==91){
				uart_buffer=read_char();
				if (uart_buffer==65 && posicao_cursor >= 15     ) { posicao_cursor-=15; } //A para cima
				if (uart_buffer==66 && posicao_cursor <= 179-15 ) { posicao_cursor+=15; } //B para baixo
				if (uart_buffer==67 && posicao_cursor != 179    ) { posicao_cursor+=1;  } //C para a frente
				if (uart_buffer==68 && posicao_cursor != 0      ) { posicao_cursor-=1;  } //D para tras
			}
			uart_buffer=8;	
		}
		//Space
		if (uart_buffer==32){
			LCD_text_mode_shiftar(1,1,posicao_cursor);
		}
		// If text:
		if (uart_buffer!=8){
			textBuffer[posicao_cursor]=uart_buffer;
			if(posicao_cursor>=179){
				posicao_cursor=179;
			}			
			else{
				posicao_cursor++;
			}
		}
		text_page=(posicao_cursor)/60;
	}
};

void LCD_text_mode_shiftar (uint8_t shift_n_chars,uint8_t shift_direction, uint8_t posicao_cursor){
	uint8_t a_shiftar, count=0;
	//Para a frente (space)
	if (shift_direction==1){
		a_shiftar=179-posicao_cursor-shift_n_chars; // numero de caracteres a shiftar
		if(posicao_cursor+shift_n_chars<=179) {
			while (a_shiftar>=count){
				textBuffer[179-count]=textBuffer[179-shift_n_chars-count];
				count++;
			}
			count=0;
		
			while (shift_n_chars>count){
				textBuffer[posicao_cursor+count]=32;
				count++;
			}		
		}
	}	
	// Para tras (backspace)
	if (shift_direction==0){
		count=0;
		a_shiftar=179-posicao_cursor; // numero de caracteres a shiftar
		while (a_shiftar>=count){
			if(posicao_cursor<179){
				textBuffer[179-a_shiftar+count]=textBuffer[179-a_shiftar+shift_n_chars+count];
			}
			count++;
			textBuffer[180]=32;
		}	
	}
};

void preenche_ecra(uint8_t text_page){
	uint8_t* font     = (uint8_t*)fontSet1;
	uint8_t  fontX	  = pgm_read_byte(font+1);	// font width
	uint8_t  fontY	  = pgm_read_byte(font+2);	// font height
	uint8_t  x=0, y=0;
	uint8_t count =0;

	while (count < 60) {
		LCD_char(font, textBuffer[text_page*60+count]-32,x,y,LCD_SET);
		x+=fontX+1;		            // give 1 pixel space b/w letters
		if(textBuffer [text_page*60+count]-32!=8 && textBuffer [text_page*60+count]-32!=13){
			if((x+fontX) > ( BUFFER_WIDTH - 6)  /*&& (x+fontX)< 128*/){
				x = 0;
				y+= fontY + 1;	// give one pixel space b/w lines
			}
		}
	count++;	
	}
	LCD_displayFrameBuffer();
};

void LCD_print_image(void){
	int count=0;
	LCD_clearFrameBuffer();	
	count=0;
	while(count<BUFFER_SIZE){
		frameBuffer[count]=pgm_read_byte(&image1[count]);
		count++;
	}
	LCD_displayFrameBuffer();
};
		
void LCD_desenha_cursor() {
	uint8_t* font     = (uint8_t*)fontSet1;	
	uint8_t  fontX	  = pgm_read_byte(font+1);	// font width
	uint8_t  fontY	  = pgm_read_byte(font+2);	// font height
	uint8_t  x, y;
	linha=((posicao_cursor%60)/15);
	coluna=(posicao_cursor%15);
	x=(fontX+1)*coluna;
	y=(fontY+1)*linha;
	LCD_char(font, 123-32 ,x,y,LCD_SET);
};
	
void LCD_text_status(void){
	uint8_t* font     = (uint8_t*)fontSet1;	
	uint8_t  fontX	  = pgm_read_byte(font+1);	// font width
	uint8_t  fontY	  = pgm_read_byte(font+2);	// font height
	uint8_t  x, y;
	uint8_t linha_local,coluna_local;
	linha_local=((posicao_cursor%60)/15)+1;
	coluna_local=(posicao_cursor%15)+1;
	x=0;
	y=(fontY+1)*4+1;

	LCD_drawLine(0,y-1,96,y-1,1); // Linha horizontal
	LCD_drawLine(90,0,90,31,1);   // Linha vertical
	LCD_drawString(font, (uint8_t*)PSTR("Pag:   Pos:   x \0"),1,x,y,LCD_SET);
	LCD_char(font,linha+17,91,33,LCD_SET);
	if (coluna_local>9) {
		LCD_char(font,49-32,72,33,LCD_SET);
	}	
	LCD_char(font,(coluna_local%10-1)+17,78,33,LCD_SET);
	switch(text_page){
			case 0 :
				LCD_char(font,49-32,30,33,LCD_SET);
				LCD_fillRect(92, 0,96,10,1);
				LCD_fillRect(92,10,96,20,0);
				LCD_fillRect(92,20,96,30,0);
				break;
			case 1 :
				LCD_char(font,50-32,30,33,LCD_SET);
				LCD_fillRect(92, 0,96,10,0);
				LCD_fillRect(92,10,96,20,1);
				LCD_fillRect(92,20,96,30,0);
				break;
			case 2 :
				LCD_char(font,51-32,30,33,LCD_SET);
				LCD_fillRect(92, 0,96,10,0);
				LCD_fillRect(92,10,96,20,0);
				LCD_fillRect(92,20,96,30,1);
				break;
		}
};	
	
		
ISR(TIMER0_OVF_vect){
	if (status==1) {
		
		if (pisca==0 || pisca==30000) {
		    preenche_ecra(text_page);
			LCD_text_status();
			LCD_displayFrameBuffer();
			pisca=0;
		}
		
		pisca++;
		
		if (pisca==15000) {
			preenche_ecra(text_page);
			LCD_text_status();
			LCD_desenha_cursor();
			LCD_displayFrameBuffer();
		}
	}
};

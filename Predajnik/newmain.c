/* 
 * File:   newmain.c
 * Opis programa: Program koji omogucava da pritiskom na taster promenimo stanje diode
*  Kori??eni taster je RD0, a dioda RB0
 * Otklonjena je mogucnost 'odskakanja tastera'.
 *Odradjeno:
 * napred, nazad, levo, desno
 * kupola levo i desno
 * brzina gore i dole + indikatorske diode(ne rade)
 * uart prvo posalje "#" konkatenirano sa jos 2 karaktera za komandu
 * Created on 16. mart 2017., 11.07
 */

#include <stdio.h>
#include <stdlib.h>
#include<p30fxxxx.h>

#define TMR1_period 10000// perioda tajmera je 1ms

_FOSC(CSW_FSCM_OFF & XT_PLL4);//instruction takt je isti kao i kristal
_FWDT(WDT_OFF);

enum command{f,b,r,l,a,h, null};//

unsigned int stanje[13] = {0,0,0,0,0,0,0,0,0,0,0,0,0}; //stanje za softverski debounding, broji do 20 pa onda pusta
unsigned int taster[13] = {0,0,0,0,0,0,0,0,0,0,0,0,0}; //taster stavlja na 1 ako stanje dodje do 20
unsigned int speed = 1;
unsigned int i;
unsigned int lights = 0;
unsigned int RD0_stanje = 0; //RD0 sam dodao jer ima 14 tastera, svi su na RBx osim ovog sto je na RD
unsigned int RD0_taster = 0;
unsigned char tempRX;
unsigned int broj1,broj2;

//unsigned int lights = 0;
unsigned int fwd = 0; 
unsigned int bck = 0;
unsigned int left = 0;
unsigned int right = 0;
unsigned int aim = 0;
unsigned int horn = 0;

unsigned int send_counter = 0; //tajmer ida na 1ms, send counter se inkrementiera ide od 0 do 2
unsigned int refresh_enable = 1; //da li smem da omogucim refresh ili ne
//ako iz maina saljem, onemogucim ga da ne dodje do konflikta kod koriscenja uarta

enum command prevCmd;

void Init_T1(void)
{
	TMR1 = 0;
	PR1 = TMR1_period;
	
	T1CONbits.TCS = 0; // 0 = Internal clock (FOSC/4)
	//IPC1bits.T2IP = 3 // T2 interrupt pririty (0-7)
	//SRbits.IPL = 3; // CPU interrupt priority is 3(11)
	IFS0bits.T1IF = 0; // clear interrupt flag
	IEC0bits.T1IE = 1; // enable interrupt

	T1CONbits.TON = 1; // T2 on 
}

void __attribute__ ((__interrupt__)) _T1Interrupt(void) // svakih 1ms
{
    if(refresh_enable == 1){ //ako je omogucen refresh
    
        if(send_counter == 0){//ako je send counter na 0 posalji signal za svetla i inkrementiraj
            if(lights == 1) WriteStringToUART1("#D1");
            else WriteStringToUART1("#D0");
            send_counter++;
        }else if(send_counter == 1){//ako je 1 posalji signal za brzinu i ikrementiraj
            
            char buff[10];
            sprintf(buff, "#A%d", speed);
             WriteStringToUART1(buff);   
             send_counter++;
        }else if(send_counter == 2){//ako je 2 posalji prethodnu komandu, za slucaj da nije uhvatio pustanje sirene ili signala kretanja posto moze da zaglavi
            checkPrevCmd();
            WriteStringToUART1("#XX");//ovo ne sluzi nicemu
            send_counter = 0; //vrati na 0 i refresh ciklus je gotov za 3ms
        }
    }
    IFS0bits.T1IF = 0; 
       
}

void initUART1(void)
{
    U1BRG=0x00207;//ovim odredjujemo baudrate

    U1MODEbits.ALTIO=1;//biramo koje pinove koristimo za komunikaciju osnovne ili alternativne

    //IEC0bits.U1RXIE=1;//omogucavamo rx1 interupt

    U1STA&=0xfffc;

    U1MODEbits.UARTEN=1;//ukljucujemo ovaj modul

    U1STAbits.UTXEN=1;//ukljucujemo predaju
}

/*********************************************************************
* Ime funkcije      : WriteUART1                                     *
* Opis              : Funkcija upisuje podatak u registar 			 *
* 					  za slanje podataka U1TXREG,   				 *
* Parametri         : unsigned int data-podatak koji treba poslati   *
* Povratna vrednost : Nema                                           *
*********************************************************************/

void WriteUART1(unsigned int data)
{
	  while(!U1STAbits.TRMT);

    if(U1MODEbits.PDSEL == 3)
        U1TXREG = data;
    else
        U1TXREG = data & 0xFF;
}


void WriteStringToUART1(unsigned char * str){
    unsigned int k = 0;
    while(*(str+k)!= 0x00){
        WriteUART1(*(str+k));
        k++;
    }
}

void speed_indicator(int speed){ //indikator brzine ima 5 dioda
    
    LATFbits.LATF4 = 0; //brzina 1
    LATFbits.LATF5 = 0;//brzina 2
    LATFbits.LATF6 = 0;//brzina 3
    LATDbits.LATD8 = 0;//brzina 4
    LATDbits.LATD2 = 0;//brzina 5
    switch(speed){ //ako je brzina 0 sve ih pogasi
        case 0:
            LATFbits.LATF4 = 0;
            LATFbits.LATF5 = 0;
            LATFbits.LATF6 = 0;
            LATDbits.LATD8 = 0;
            LATDbits.LATD2 = 0;
            break;
        case 1:
            LATFbits.LATF4 = 1;
            LATFbits.LATF5 = 0;
            LATFbits.LATF6 = 0;
            LATDbits.LATD8 = 0;
            LATDbits.LATD2 = 0;
            break;
        case 2:
            LATFbits.LATF4 = 1;
            LATFbits.LATF5 = 1;
            LATFbits.LATF6 = 0;
            LATDbits.LATD8 = 0;
            LATDbits.LATD2 = 0;
            break;
        case 3:
            LATFbits.LATF4 = 1;
            LATFbits.LATF5 = 1;
            LATFbits.LATF6 = 1;
            LATDbits.LATD8 = 0;
            LATDbits.LATD2 = 0;
            break;
        case 4:
            LATFbits.LATF4 = 1;
            LATFbits.LATF5 = 1;
            LATFbits.LATF6 = 1;
            LATDbits.LATD8 = 1;
            LATDbits.LATD2 = 0;
            break;
        case 5:
            LATFbits.LATF4 = 1;
            LATFbits.LATF5 = 1;
            LATFbits.LATF6 = 1;
            LATDbits.LATD8 = 1;
            LATDbits.LATD2 = 1;
    };
}

void check_speed(){
    if(PORTBbits.RB4 == 1){
            if(stanje[4] < 21) stanje[4]++; //broji do 20
            
            if(stanje[4] == 20) taster[4] = 1; //ako je dosao do 20 taster je pritisnut
            
        }else{
            if(taster[4] == 1){//ako je taster pusten a promenljiva taster je na 1 znaci da je button pritisnut pa pusten
                    //falling edge
                if(speed < 5) speed++;
                speed_indicator(speed);
                char buff[10];
                sprintf(buff, "#A%d", speed);
                refresh_enable = 0;
                for(i=0;i<1;i++) WriteStringToUART1(buff);
                refresh_enable = 1;
                }
            stanje[4] = 0;
            taster[4] = 0;
            }
        
        if(PORTBbits.RB5 == 1){
            if(stanje[5] < 21) stanje[5]++;
            
            if(stanje[5] == 20) taster[5] = 1;
        }else{
            if(taster[5] == 1){
                    //falling edge
                if(speed > 0) speed--;
                speed_indicator(speed);
                char buff[10];
                sprintf(buff, "#A%d", speed);
                refresh_enable = 0;
                for(i=0;i<1;i++) WriteStringToUART1(buff);
                refresh_enable = 1;
                }
            stanje[5] = 0;
            taster[5] = 0;
            }
}

void check_movement(){
     if(PORTBbits.RB0 == 0){
            if(stanje[0] < 21) stanje[0]++;
            
            if(stanje[0] == 20){//ako je taster pritisnut tj drzis ga salje F1
                taster[0] = 1;
                fwd = 1;
                prevCmd = null;
                char buff[11] = "#F1";
                refresh_enable = 0;
                for(i=0;i<1;i++) WriteStringToUART1("#F1");//dok drzis taster salje jedan F1 u jednom ciklusu
                refresh_enable = 1;
                stanje[0] = 0;
                LATFbits.LATF1 = 1;
            }
        }else{
            if(taster[0] == 1){
                //char buff[10] = "#F0";
                //ako je taster 0 onda je pusten taster i salje F0
                fwd = 0;
                refresh_enable = 0;
                prevCmd = f;
                for(i=0;i<10;i++) WriteStringToUART1("#F0");//ako je taster pusten posalje 10 puta za redom da stane, da budemo sigurni da je primio
                refresh_enable = 1;
                }
            stanje[0] = 0;
            taster[0] = 0;
            LATFbits.LATF1 = 0;
            }
     
     
      if(PORTBbits.RB1 == 0){
            if(stanje[1] < 21) stanje[1]++;
            
            if(stanje[1] == 20){
                taster[1] = 1;
                bck = 1;
                prevCmd = null;
                //char buff[10] = "#B1";
                refresh_enable = 0;
                for(i=0;i<1;i++) WriteStringToUART1("#B1");
                refresh_enable = 1;
                stanje[1] = 0;
                LATFbits.LATF0 = 1;
            }
        }else{
            if(taster[1] == 1){
                bck = 0;
                prevCmd = b;
                //char buff[10] = "#B0";
                refresh_enable = 0;
                for(i=0;i<10;i++) WriteStringToUART1("#B0");
                refresh_enable = 1;
                }
            stanje[1] = 0;
            taster[1] = 0;
            LATFbits.LATF0 = 0;
            }
     
     
     if(PORTBbits.RB2 == 0){
            if(stanje[2] < 21) stanje[2]++;
            
            if(stanje[2] == 20){
                taster[2] = 1;
                left = 1;
                prevCmd = null;
                //char buff[10] = "#L1";
                refresh_enable = 0;
                for(i=0;i<1;i++) WriteStringToUART1("#L1");
                refresh_enable = 1;
                stanje[2] = 0;
            }
        }else{
            if(taster[2] == 1){
                left = 0;
                prevCmd = l;
                //char buff[10] = "#L0";
                refresh_enable = 0;
                for(i=0;i<10;i++) WriteStringToUART1("#L0");
                refresh_enable = 1;
                }
            stanje[2] = 0;
            taster[2] = 0;
            }
     
     
      if(PORTBbits.RB3 == 0){
            if(stanje[3] < 21) stanje[3]++;
            
            if(stanje[3] == 20){
                taster[3] = 1;
                right = 1;
                prevCmd = null;
                char buff[10] = "#R1";
                refresh_enable = 0;
                for(i=0;i<1;i++) WriteStringToUART1("#R1");
                refresh_enable = 1;
                stanje[3] = 0;
            }
        }else{
            if(taster[3] == 1){
                right = 0;
                prevCmd = r;
                char buff[10] = "#R0";
                refresh_enable = 0;
                for(i=0;i<10;i++) WriteStringToUART1("#R0");
                refresh_enable = 1;
                }
            stanje[3] = 0;
            taster[3] = 0;
            }
}

void check_turrent(){
    
     if(PORTBbits.RB11 == 1){
            if(stanje[11] < 21) stanje[11]++;
                
            if(stanje[11] == 20){
                taster[11] = 1;
            }
        }else{
            if(taster[11] == 1){
                    //falling edge
                
                //char buff[10];
                //sprintf(buff, "#S%d", speed);
                refresh_enable = 0;
                for(i=0;i<10;i++) WriteStringToUART1("#TL");
                refresh_enable = 1;
                }
            stanje[11] = 0;
            taster[11] = 0;
            }
     
     
     if(PORTBbits.RB12 == 1){
            if(stanje[12] < 21) stanje[12]++;
            
            if(stanje[12] == 20) taster[12] = 1;
        }else{
            if(taster[12] == 1){
                    //falling edge
                
                //char buff[10];
                //sprintf(buff, "#S%d", speed);
                refresh_enable = 0;
                for(i=0;i<10;i++) WriteStringToUART1("#TR");
                refresh_enable = 1;
                }
            stanje[12] = 0;
            taster[12] = 0;
            }
}

void check_lights(){
    
         if(PORTBbits.RB9 == 1){
            if(stanje[9] < 21) stanje[9]++;
            
            if(stanje[9] == 20) taster[9] = 1;
        }else{
            if(taster[9] == 1){
                    //falling edge
                
                //char buff[10];
                //sprintf(buff, "#S%d", speed);
                refresh_enable = 0;
                for(i=0;i<10;i++) WriteStringToUART1("#BL"); //levi zmigavac
                refresh_enable = 1;
                }
            stanje[9] = 0;
            taster[9] = 0;
            }
         
         
         if(PORTBbits.RB10 == 1){
            if(stanje[10] < 21) stanje[10]++;
            
            if(stanje[10] == 20) taster[10] = 1;
        }else{
            if(taster[10] == 1){
                    //falling edge
                
                //char buff[10];
                //sprintf(buff, "#S%d", speed);
                refresh_enable = 0;
                for(i=0;i<10;i++) WriteStringToUART1("#BR"); //desni zmigavac zmigavac
                refresh_enable = 1;
                }
            stanje[10] = 0;
            taster[10] = 0;
            }
         
         
         if(PORTBbits.RB6 == 1){
            if(stanje[6] < 21) stanje[6]++;
            
            if(stanje[6] == 20) taster[6] = 1;
        }else{
            if(taster[6] == 1){
                    //falling edge
                
                //char buff[10];
                //sprintf(buff, "#S%d", speed);
                if(lights == 0){
                    lights = 1;
                    refresh_enable = 0;
                    for(i=0;i<10;i++) WriteStringToUART1("#D1"); //levi zmigavac
                    refresh_enable = 1;
                }else{
                    lights = 0;
                    refresh_enable = 0;
                    for(i=0;i<10;i++) WriteStringToUART1("#D0"); //levi zmigavac
                    refresh_enable = 1;
                }
                
                }
            stanje[6] = 0;
            taster[6] = 0;
            }
}

void check_siren(){
    if(PORTBbits.RB7 == 0){
            if(stanje[7] < 21) stanje[7]++;
            
            if(stanje[7] == 20){
                taster[7] = 1;
                horn = 1;
                prevCmd = null;
                char buff[11] = "#S1";
                refresh_enable = 0;
                for(i=0;i<1;i++) WriteStringToUART1("#S1");
                refresh_enable = 1;
                stanje[7] = 0;
            }
        }else{
            if(taster[7] == 1){
                //char buff[10] = "#F0";
                horn = 0;
                prevCmd = h;
                refresh_enable = 0;
                for(i=0;i<10;i++) WriteStringToUART1("#S0");
                refresh_enable = 1;
                }
            stanje[7] = 0;
            taster[7] = 0;
            }
}


void check_gun(){
    
    if(PORTBbits.RB8 == 0){
            if(stanje[8] < 21) stanje[8]++;
            
            if(stanje[8] == 20){
                taster[8] = 1;
                aim = 1;
                prevCmd = null;
                char buff[11] = "#N1";
                refresh_enable = 0;
                for(i=0;i<1;i++) WriteStringToUART1("#N1");
                refresh_enable = 1;
                stanje[8] = 0;
            }
        }else{
            if(taster[8] == 1){
                //char buff[10] = "#F0";
                aim = 0;
                prevCmd = a;
                refresh_enable = 0;
                for(i=0;i<10;i++) WriteStringToUART1("#N0");
                refresh_enable = 1;
                }
            stanje[8] = 0;
            taster[8] = 0;
            }
    
    
    if(PORTDbits.RD0 == 1){
            if(RD0_stanje < 21) RD0_stanje++;
            
            if(RD0_stanje == 20) RD0_taster = 1;
        }else{
            if(RD0_taster == 1){
                    //falling edge
                
                //char buff[10];
                //sprintf(buff, "#S%d", speed);
                refresh_enable = 0;
                for(i=0;i<10;i++) WriteStringToUART1("#FIRE"); //desni zmigavac zmigavac
                refresh_enable = 1;
                }
            RD0_stanje = 0;
            RD0_taster = 0;
            }
    
}

void initPins(){
     ADPCFGbits.PCFG0=1;//kazemo da nije analogni vec digitalni pin
    ADPCFGbits.PCFG1=1;//kazemo da nije analogni vec digitalni pin
    ADPCFGbits.PCFG2=1;//kazemo da nije analogni vec digitalni pin
    ADPCFGbits.PCFG3=1;//kazemo da nije analogni vec digitalni pin
    ADPCFGbits.PCFG4=1;//kazemo da nije analogni vec digitalni pin
    ADPCFGbits.PCFG5=1;//kazemo da nije analogni vec digitalni pin
    ADPCFGbits.PCFG6=1;//kazemo da nije analogni vec digitalni pin
    ADPCFGbits.PCFG7=1;//kazemo da nije analogni vec digitalni pin
    ADPCFGbits.PCFG8=1;//kazemo da nije analogni vec digitalni pin
    ADPCFGbits.PCFG9=1;//kazemo da nije analogni vec digitalni pin
    ADPCFGbits.PCFG10=1;//kazemo da nije analogni vec digitalni pin
    ADPCFGbits.PCFG11=1;//kazemo da nije analogni vec digitalni pin
    ADPCFGbits.PCFG12=1;//kazemo da nije analogni vec digitalni pin
    
    TRISBbits.TRISB0=1;//konfigurisemo kao ulaz
	TRISBbits.TRISB1=1;//konfigurisemo kao ulaz
    TRISBbits.TRISB2=1;//konfigurisemo kao ulaz
    TRISBbits.TRISB3=1;//konfigurisemo kao ulaz
    TRISBbits.TRISB4=1;//konfigurisemo kao ulaz
    TRISBbits.TRISB5=1;//konfigurisemo kao ulaz
    TRISBbits.TRISB6=1;//konfigurisemo kao ulaz
    TRISBbits.TRISB7=1;//konfigurisemo kao ulaz
    TRISBbits.TRISB8=1;//konfigurisemo kao ulaz
    TRISBbits.TRISB9=1;//konfigurisemo kao ulaz
    TRISBbits.TRISB10=1;//konfigurisemo kao ulaz
    TRISBbits.TRISB11=1;//konfigurisemo kao ulaz
    TRISBbits.TRISB12=1;//konfigurisemo kao ulaz
    
	TRISDbits.TRISD0=1;//konfigurisemo kao izlaz
    
    TRISFbits.TRISF0 = 0; //5 dioda su za brzie
    TRISFbits.TRISF1 = 0; //2 diode su da sijaju kada idemo napred i nazad
    TRISFbits.TRISF4 = 0;
    TRISFbits.TRISF5 = 0;
    TRISFbits.TRISF6 = 0;
    TRISDbits.TRISD8 = 0;
    TRISDbits.TRISD2 = 0;
}

void checkPrevCmd(){
    
    
    if(prevCmd == f) WriteStringToUART1("#F0");
    else if(prevCmd == b) WriteStringToUART1("#B0");
    else if(prevCmd == l) WriteStringToUART1("#L0");
    else if(prevCmd == r) WriteStringToUART1("#R0");
    else if(prevCmd == h) WriteStringToUART1("#S0");
    else if(prevCmd == a) WriteStringToUART1("#N0");
}

void check_buttons(){
        check_speed();
        check_movement();
        check_turrent();
        check_lights();
        check_siren();
        check_gun();
}

int main(int argc, char** argv) {
    
    initUART1();
    Init_T1();
    initPins();
    speed_indicator(speed);
    while(1){
        
        //WriteUART1(55);
        check_buttons();

        //checkPrevCmd();
        //for(i=0;i<3000;i++);
        
        
            
            
        }
    
    return 0;
}
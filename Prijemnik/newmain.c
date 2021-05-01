/* 
Naziv programa: UART serijska RS232 komunikacija
Primena programa:  Program omogucava komunikaciju dsPIC30f4013 sa PC-em preko
                serijskog porta.Brzina komunikacije je 9600 
			    a, frekvencija oscilatora je 10MHz sa PLL-om 4 
Opis programa:  vraca nazad poslati podatak sa mikrokontrolera 

 *
 * Created on 27. septembar 2017., 13.59
 * Komandu salje tako sto prvo posalje # a nakon toga 2 karaktera koji su idektifikatori komande
 * Spisak validnih komadi:
 * #F1 --> idi napred
 * #F0 --> nemoj ici napred
 * #B1 --> idi nazad
 * #B0 --> nemoj ici nazad
 * #L1 --> idi levo
 * #L0 --> nemoj ici levo
 * #R1 --> idi desno
 * #R0 --> nemoj ici desno
 * #S1 --> stisni sirenu
 * #S0 --> pusti sirenu
 * #Ax --> podesi brzinu, gde x ide od 0 do 5
 * #BL --> levi zmigavac
 * #BR --> desni zmigavac
 * #D0 --> upali svetla
 * #D1 --> ugasi svetla
 * #TR --> kupola desno
 * #TL --> kupola levo
 * #N0 --> upali nisan
 * #N1 --> ugasi nisan
 * #FIRE --> pucaj, mada hvara samo FI, nisam mogao da se patim sa prepoznavanjem celog niza
 * 
 */

#include <stdio.h>
#include <stdlib.h>
#include<p30fxxxx.h>

#define TMR1_period 10000 // 1/Fosc = 0.1us !!!, 0.1us * 10000 = 1ms  */

#define Blinker_L LATBbits.LATB0 
#define Blinker_R LATBbits.LATB1 
#define Lights  LATBbits.LATB2 
#define M1_1 LATBbits.LATB3 
#define M1_2 LATBbits.LATB4 
#define M2_1 LATBbits.LATB5 
#define M2_2 LATBbits.LATB6 
#define Aim LATBbits.LATB7 
#define PWM_mot LATBbits.LATB9 
#define PWM_turr LATBbits.LATB10 
#define PWM_shot LATBbits.LATB11 
#define horn LATDbits.LATD2
#define servo_min_fire 30 //krajnji levi polozaj servo motora
#define servo_max_fire 60 //krajnji desni polozaj servo motora, idem samo do odredjenih uglova zbog koraka kupole, ne moze od kablova dalje
#define servo_period 780 //sa preskalerom od 256 ova vrednost podesava frekvenciju pwm-a da bude 50Hz
#define fire_servo OC1RS //output compare na pinu RD0
#define turret_servo OC2RS //output compare na pinu RD1
#define motor_speed OC4RS //output compare na pinu RD3

#define blinker_counter_cmp 500 //posto je tajmer T1 podesen na 1ms, ova vrednost brojaca za zmigavce ce omogiciti da blinkaju frekvencijom od 1Hz

_FOSC(CSW_FSCM_OFF & XT_PLL4);//instruction takt je isti kao i kristal
_FWDT(WDT_OFF);

unsigned char tempRX; //trenutni karakter u UART bufferu
char command[2] = {0,0};
char prev_tempRX;  //prosli karakter u uart bufferu, gleda da dobije # pa neko validno slovo
unsigned int hashtag_count = 0;
unsigned int i, j;

unsigned int blinker_left = 0; //signal levog zmigavca
unsigned int blinker_right = 0; //signal desnog zmigavca
unsigned int lights = 0; //signal svetala
unsigned int fwd = 0; //signal za kretanje napred
unsigned int bck = 0; //signal za kretanje nazad
unsigned int left = 0; //signal za skretanje levo
unsigned int right = 0;//signal za skretanje desno
unsigned int speed = 1;//brzina, ide od 0 do 5, 0 je parkirna
unsigned int aim = 0;//signal za nisan
unsigned int shoot = 0;//signal za pucanje
unsigned int turr = 42; // 42 je srednji polozaj izmedju servo min fire i servo max fire
unsigned int siren = 0;//signal za sirenu
unsigned int validCommand = 0; //proverava da li je komanda validna
unsigned int turr_enable = 1;//turr_enable, treba nam posto salje vise #TR za redom, da bih bio siguran da je primio, pa da ne okrene kupolu 10 puta za redom
unsigned int turr_counter = 0;//brojac koristim u tajmeru 2 da turr_enable vrati na 0
unsigned int fire_counter = 20;//brojac koristim u tajmeru 2 da servo za pucanje vrati u poceetni polozaj
//unsigned int shoot_counter = 0;

unsigned int blinker_counter = 0;//brojac za zmigavac u tajmeru 1
unsigned int blinkerFSM = 0;//Finite State Machine za zmigavce
/*
 *Kada zmigavac nije pritisnit FSM je u stanju 0 iliti IDLE
 *Kada je zmigavac pritisnit FSM je u start stanje iliti 1
 * Kada se pritisne taster za levo ili desno ako je FSM u stanju 1 prelazi u stanje 2
 * Kada se pritisne taster za  napred ili nazad, ako je FSM u stanje 2 prelazi u stanje 0 i zmigavac se gasi
 */

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

void Init_T2(void)
{
	 PR2=servo_period;//odredjuje frekvenciju po formuli
        OC1RS=20;//postavimo pwm
        OC1R=1000;//inicijalni pwm pri paljenju samo
        OC1CONbits.OCM = 0b110; //output ccompare modul za pwm, 0b111 nije radilo iz nekog razloga
        T2CONbits.TCKPS=0b11; //preskaler 1:256
        //IFS0bits.T2IF = 0; // clear interrupt flag
        //IEC0bits.T2IE = 1; // enable interrupt
        T2CONbits.TON=1;//ukljucujemo timer koji koristi
        OC1RS = servo_max_fire; //OC1RS ide od 0 do vrednosti koja se nalazi u PR2, pa se faktor ispune od 0 do 100 skalira od 0 do PR2
        
        OC2RS=20;//postavimo pwm
        OC2R=1000;//inicijalni pwm pri paljenju samo
        OC2CONbits.OCM = 0b110; //output ccompare mudule for pwm
        OC2RS = 43;
        
        OC4RS=20;//postavimo pwm
        OC4R=1000;//inicijalni pwm pri paljenju samo
        OC4CONbits.OCM = 0b110; //output ccompare mudule for pwm
        OC4RS = 0;
        
        IFS0bits.T2IF = 0; // clear interrupt flag
        IEC0bits.T2IE = 1; // enable interrupt
        
}

void setSpeed(){
    if(speed == 0){
        motor_speed = 0;
        turr = 43;//kada je u parkirnoj brzini, automatski ispravi kupolu, da moze pored njega neko drugi da parkira
        turret_servo = 43;
    }
    else motor_speed = 234+109*speed-8;//brzina motor, pocinje od 30% i penje se za 14% za svaku brzinu
}

int off_counter = 0;

void __attribute__ ((__interrupt__)) _T1Interrupt(void) // svakih 1ms
{
    TMR1 = 0;
    if(blinker_left == 1){//ako je pritisnut taster zmigavca, blinka frekvencijom 1Hz dok ga FSM ne iskljuci
        blinker_counter++;
        if(blinker_counter >= blinker_counter_cmp){
            blinker_counter = 0;
            Blinker_L=~Blinker_L;
            
        }
    }else if(blinker_right == 1){
        blinker_counter++;
        if(blinker_counter >= blinker_counter_cmp){
            blinker_counter = 0;
            Blinker_R=~Blinker_R;
            
        }
    }
    
    if(blinker_left == 0 && blinker_right == 0){ //samo ako su oba 0, iskljucuje zmigavce posto po onom algoritmu iznad moze da ostane zaglavljen na 1
        Blinker_L = 0;
        Blinker_R = 0;
    }
    
    if(siren == 1){
        horn = ~horn;//pali i gasi taster sirene na svakih 1ms, tako da imamo frekvenciju sirene od 500Hz
    }else horn = 0;
    
    
     
    IFS0bits.T1IF = 0; 
    
    ////turnOffAll();
       
}

void __attribute__ ((__interrupt__)) _T2Interrupt(void) // svakih 1ms
{//tajmer 2 radi na frekvenciji od 50Hz
    TMR2 = 0;
    turr_counter++;
    if(turr_counter == 25){//25 iteracija kroz 50Hz tajmer je pola sekunde
        turr_enable = 1;//na svakih pola sekunde radi enable kule
        turr_counter = 0;
    }
    
    if(fire_counter < 50) fire_counter++;//50 iteracija je sekund
    else fire_servo = servo_max_fire; //vrati servo za pucanje u pocetni polozaj
    
    IFS0bits.T2IF = 0; 
       
}
void initUART1(void)
{
    U1BRG=0x0207;//ovim odredjujemo baudrate 0x0207 je baudrate od 1200

    U1MODEbits.ALTIO=1;//biramo koje pinove koristimo za komunikaciju osnovne ili alternativne

    IEC0bits.U1RXIE=1;//omogucavamo rx1 interupt

    U1STA&=0xfffc;

    U1MODEbits.UARTEN=1;//ukljucujemo ovaj modul

    U1STAbits.UTXEN=1;//ukljucujemo predaju
}

void __attribute__((__interrupt__)) _U1RXInterrupt(void) 
{
    prev_tempRX = tempRX;
    IFS0bits.U1RXIF = 0;
    tempRX=U1RXREG;

} 

void __attribute__((__interrupt__)) _U2RXInterrupt(void) 
{
    prev_tempRX = tempRX;
    IFS1bits.U2RXIF = 0;
    tempRX=U2RXREG;

}  //ni ovo ne treba
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

/*
 * 
 */

int command1IsValid(char c){ // proverava da li je slovo koje dolazi nakon # neko od onig iz tabele komandi
    switch(c){
        case 'F':
            return 1;
            break;
        case 'B':
            return 1;
            break;
        case 'L':
            return 1;
            break;
        case 'R':
            return 1;
            break;
        case 'S':
            return 1;
            break;
        case 'A':
            return 1;
            break;
        case 'T':
            return 1;
            break;
        case 'D':
            return 1;
            break;
        case 'N':
            return 1;
            break;
        default:
            return 0;
    }
}

void turnOffAll(){
    fwd = 0;
    bck = 0;
    left = 0;
    right = 0;
    //unsigned int speed = 1;
    aim = 0;
    turr_enable = 1;
    //iskljuci sve da ne bi zaglavljivalo, nisam siguran da li ovo igde koristim
   // siren = 0;

}

void check_direction(){ //kontrola H mosta za smer motora

    
    if(fwd == 1){ //1010 je za smer napred
        M1_1 = 1;
        M1_2 = 0;
        M2_1 = 1;
        M2_2 = 0;
        if(blinkerFSM == 2){ //Ako je FSM za zmigavac u stanju 2 vrati ga u IDLE i ugasi zmigavce
            blinkerFSM = 0;
            blinker_left = 0;
            blinker_right = 0;
        }
        return 0;
    }
    if(bck == 1){// 0101 je smer za nazad
        M1_1 = 0;
        M1_2 = 1;
        M2_1 = 0;
        M2_2 = 1;
        if(blinkerFSM == 2){//Ako je FSM za zmigavac u stanju 2 vrati ga u IDLE i ugasi zmigavce
            blinkerFSM = 0;
            blinker_left = 0;
            blinker_right = 0;
        }
        return 0;
    }
    
    if(left == 1){//Smer za levo je 1001
        M1_1 = 1;
        M1_2 = 0;
        M2_1 = 0;
        M2_2 = 1;
        if(blinkerFSM == 1) blinkerFSM = 2; //ako je FSM u stanju 1 prelazi u stanje 2
        return 0;
    }if(right == 1){ //smer za desno je 0110
        M1_1 = 0;
        M1_2 = 1;
        M2_1 = 1;
        M2_2 = 0;
        if(blinkerFSM == 1) blinkerFSM = 2; //ako je FSM u stanju 1 prelazi u stanje 2
        return 0;
    }
    
    M1_1 = 0;//motori su ugaseni za 0000
    M1_2 = 0;
    M2_1 = 0;
    M2_2 = 0;
}
bool command_enable = 0;
void RfModuleRead(){
        if(hashtag_count == 0){ //ovo ne sluzi nicemu
        if(prev_tempRX == '#' && tempRX != '#'){ //ako je prethodni karakter u baferu # a sledeci nije, ima sanse da je primljena validna komanda
            //if(command1IsValid(tempRX) == 1) command[0] = tempRX; //proverava da li je koanda validna i ako jeste smesta je u command[0]
            command_enable = 1;
        }else if(command1IsValid(prev_tempRX) == 1 && command_enable){//ako je prethodna komanda nije #, moguce je da je prethodna validna a da je trenutna drugi karakter komande
            //WriteUART1(prev_tempRX);
            //WriteUART1(tempRX);
            command_enable = 0;
            validCommand = 1; 
            switch(prev_tempRX){
        case 'F': //ako je prethodna komanda F udji u F i proveravaj da li je trenutna 0 ili 1 ako je 0 gasi ako je 1 pali
            if(tempRX == '1'){ // 
               // //WriteStringToUART1("Napred\n");
                turnOffAll();
                fwd = 1;
            }
            else if(tempRX == '0'){
               // //WriteStringToUART1("Nemoj napred\n");
                turnOffAll();
                fwd = 0;
            }
            else if(tempRX == 'I'){
              //  //WriteStringToUART1("PUCAJ!\n");
                //WriteStringToUART1("F\n");
                //turnOffAll();
                shoot = 1;
                fire_servo = servo_min_fire; //kara je primljena komanda pucanja postavi servo u krajnji polozaj
                fire_counter = 0;
                
                //for(i=0;i<15000;i++);
                    //for(j=0;j<300;j++);
                    
                //fire_servo = servo_max_fire;
                
            }
            break;
        case 'B':
            if(tempRX == '1'){
               // //WriteStringToUART1("Nazad\n");
                turnOffAll();
                bck = 1;
            }
            else if(tempRX == '0'){
              //  //WriteStringToUART1("Nemoj nazad\n");
                turnOffAll();
                bck = 0;
            }
            else if(tempRX == 'L'){
               // //WriteStringToUART1("Levi zmigavci\n");
                blinker_left = 1;
                if(blinkerFSM == 0) blinkerFSM = 1;
                
            }
            else if(tempRX == 'R'){
              //  //WriteStringToUART1("Desni zmigavci\n");
                blinker_right = 1;
                if(blinkerFSM == 0) blinkerFSM = 1;
            }
            break;
        case 'L':
            if(tempRX == '1'){
              //  //WriteStringToUART1("LEvo\n");
                turnOffAll();
                left = 1;
            }
            else if(tempRX == '0'){
              //  //WriteStringToUART1("Nemoj levo\n");
                turnOffAll();
                left = 0;
            }
            break;
        case 'R':
            if(tempRX == '1'){
               // //WriteStringToUART1("Desno\n");
                turnOffAll();
                right = 1;
            }
            else if(tempRX == '0'){
              //  //WriteStringToUART1("Nemoj desno\n");
                turnOffAll();
                right = 0;
            }
            break;
        case 'S':
            if(tempRX == '1'){
              //  //WriteStringToUART1("Sirena on\n");
                
                //if(siren != 1) //turnOffAll();
                siren = 1;
                turnOffAll();
            }
            else if(tempRX == '0'){
              //  //WriteStringToUART1("Sirena off\n");
                //turnOffAll();
                siren = 0;
            }
            break;
        case 'N':
            if(tempRX == '1'){
              //  //WriteStringToUART1("Nisani\n");
                turnOffAll();
                Aim = 1;
            }
            else if(tempRX == '0'){
              //  //WriteStringToUART1("Nemoj nisaniti\n");
                turnOffAll();
                Aim = 0;
            }
            break;
        case 'T':
            if(tempRX == 'L'){ //komanda za kupolu levo
                //WriteStringToUART1("L\n");////WriteStringToUART1("Kupola Levo\n");
                if(turr_enable == 1){
                turr_enable = 0;
                if(turr > servo_min_fire+2) turr-=2; // ako je polozaj veci od minimalnog, smanjuje
                else turr = servo_min_fire; // ako ne, zakucava ga na minimaln
                turret_servo = turr;
                //for(i=0;i<10000;i++);
                }
            }
            else if(tempRX == 'R'){
                if(turr_enable == 1){
                turr_enable = 0;
                //WriteStringToUART1("R\n"); ////WriteStringToUART1("Kupola desno\n");
                if(turr < servo_max_fire-2) turr+=2;
                
                else turr = servo_max_fire;
                turret_servo = turr;
                //for(i=0;i<10000;i++);
                }
            }
            break;
        case 'A':
            switch(tempRX){
                case '0':
                  //  //WriteStringToUART1("Brzina 0\n");
                    speed = 0;
                    //WriteStringToUART1("0\n");
                    break;
                case '1':
                  //  //WriteStringToUART1("Brzina 1\n");
                    speed = 1;
                    //WriteStringToUART1("1\n");
                    break;
                case '2':
                  //  //WriteStringToUART1("Brzina 2\n");
                    speed = 2;
                    //WriteStringToUART1("2\n");
                    break;
                case '3':
                  //  //WriteStringToUART1("Brzina 3\n");
                    //WriteStringToUART1("3\n");
                    speed = 3;
                    break;
                case '4':
                 //   //WriteStringToUART1("Brzina 4\n");
                    //WriteStringToUART1("4\n");
                    speed = 4;
                    break;
                case '5':
                  //  //WriteStringToUART1("Brzina 5\n");
                    //WriteStringToUART1("5\n");
                    speed = 5;
                    break;
            }
            break;
        case 'D':
             if(tempRX == '1'){
                 ////WriteStringToUART1("Upali svetla\n");
                 ////turnOffAll();
                 Lights = 1;
             }
             else if(tempRX == '0'){
                 ////WriteStringToUART1("Ugasi svetla\n");
                //   //turnOffAll();
                 Lights = 0;
             }
            break;
            
        case 'X':
            if(tempRX == 'X') turnOffAll();
            break;
        default:
            hashtag_count = 0;
            validCommand = 0;
            break;
            
    }
        }else hashtag_count = 0;
        

        }
}

void initPins(){ //inicijilizacija pinova
    ADPCFGbits.PCFG0=1;//kazemo da nije analogni vec digitalni pin
    ADPCFGbits.PCFG1=1;//kazemo da nije analogni vec digitalni pin
    ADPCFGbits.PCFG2=1;//kazemo da nije analogni vec digitalni pin
    ADPCFGbits.PCFG3=1;//kazemo da nije analogni vec digitalni pin
    ADPCFGbits.PCFG4=1;//kazemo da nije analogni vec digitalni pin
    ADPCFGbits.PCFG5=1;//kazemo da nije analogni vec digitalni pin
    ADPCFGbits.PCFG6=1;//kazemo da nije analogni vec digitalni pin
    ADPCFGbits.PCFG7=1;//kazemo da nije analogni vec digitalni pin
    ADPCFGbits.PCFG9=1;//kazemo da nije analogni vec digitalni pin
    ADPCFGbits.PCFG10=1;//kazemo da nije analogni vec digitalni pin
    ADPCFGbits.PCFG11=1;//kazemo da nije analogni vec digitalni pin
    
    TRISBbits.TRISB0=0;//konfigurisemo kao izlaz
	TRISBbits.TRISB1=0;//konfigurisemo kao izlaz
    TRISBbits.TRISB2=0;//konfigurisemo kao izlaz
    TRISBbits.TRISB3=0;//konfigurisemo kao izlaz
    TRISBbits.TRISB4=0;//konfigurisemo kao izlaz
    TRISBbits.TRISB5=0;//konfigurisemo kao izlaz
    TRISBbits.TRISB6=0;//konfigurisemo kao izlaz
    TRISBbits.TRISB7=0;//konfigurisemo kao izlaz
    TRISBbits.TRISB9=0;//konfigurisemo kao izlaz
    TRISBbits.TRISB10=0;//konfigurisemo kao izlaz
    TRISBbits.TRISB11=0;//konfigurisemo kao izlaz
    TRISDbits.TRISD2=0;//konfigurisemo kao izlaz
    TRISDbits.TRISD0=0;
    
    LATBbits.LATB0 = 0; //blinker left
    LATBbits.LATB1 = 0; //blinker right
    LATBbits.LATB2 = 0; //lights
    LATBbits.LATB3 = 0; //M1_1
    LATBbits.LATB4 = 0; //M1_2
    LATBbits.LATB5 = 0; //M2_1
    LATBbits.LATB6 = 0; //M2_2
    LATBbits.LATB7 = 0; //Aim
    LATBbits.LATB9 = 0; //PWM_mot
    LATBbits.LATB10 = 0; //PWM turr
    LATBbits.LATB11 = 0;//PWM shot
    LATDbits.LATD2 = 0;//Siren
    LATDbits.LATD0 = 0;
}

int main(int argc, char** argv) {
    
    
    initUART1();
    //initUART2();
    initPins();
    Init_T1();
    Init_T2();
    
    
	while(1)
	{
        //WriteUART1(tempRX);
        
        //fwd = 1;
        RfModuleRead();
        setSpeed();
        if(validCommand == 1){
            check_direction();//ako je komanda validna ulazi, ako ne onda ne ulazi, da mi ne upada stalno u check_direction() ako bas ne mora
            validCommand = 0;
        }
        //M1_1 = 1;
    }//od whilea

    return (EXIT_SUCCESS);
}


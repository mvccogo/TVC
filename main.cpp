// Projeto de CDM - Sistema TVC
// Trechos do giroscópio estão comentados

#include <Servo.h> //Biblioteca para o servo
#include <Wire.h>  
#include <LiquidCrystal.h>
#include <math.h>
// Biblioteca para o giroscópio:
// #include <MPU6050_tockn.h>

// Inicializa o giroscópio
//MPU6050 gyro(Wire);


// Portas
const int porta_Motor1 = 6;
const int porta_Motor2 = 9;
const int ledV = 10;
const int ledG = 8;
////////////// Variáveis
// Servos
Servo motor1;
Servo motor2;

// LCD
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

//Angulos provenientes do giroscopio em graus
float gyroX;
float gyroY;

//ultimos angulos obtidos
float gyroX_last;
float gyroY_last;


unsigned long ultimoTempo = 0;								// Controle do tempo
unsigned long ultimaPerturbacao = 0;

float deltat = 0;											// Tempo entre duas iterações
float vel_angular[3] = {0,0,0};								// Em rads/s
float pi = 3.1415;

float Fx = 0;												// Força perpendicular no eixo X
float Fy = 0;												// Força perpendicular no eixo Y

float accel_x = 0;											// Aceleração eixo X
float accel_y = 0;											// Aceleração eixo Y

float theta1 = 0;											// Rotação do motor 1
float theta2 =0;											// Rotação do motor 2

float t = 0;												// Tempo de reação do sistema TVC

float ultimoLcd = 0;


//Dados do foguete
const float L = 1;
const float m = 8;
const float a_b = 0.4;
const float Fth = 200;
//////////////////////////////////////////////////////////////

void setup()
{
  	pinMode(ledV, OUTPUT);
  	pinMode(ledG, OUTPUT);
    Serial.begin(9600);										//inicia o monitor serial
    lcd.begin(16, 2);										//inicia o lcd
    Wire.begin();
    //gyro.begin();
    //gyro.calcGyroOffsets(true);

    motor1.attach(porta_Motor1);							//seta o motor1 na porta 6
    motor2.attach(porta_Motor2);							//seta o motor2 na porta 9
  	motor1.write(90);
  	motor2.write(90);
  
	digitalWrite(ledV, LOW);
    digitalWrite(ledG, HIGH);
    lcd.setCursor(0,0);										
    lcd.print("Projeto CDM");		
    lcd.setCursor(0,1);
    lcd.print("Sistema TVC");
	delay(1000);
  	lcd.clear();
  	lcd.setCursor(0,0);
    lcd.print("Iniciando ");
  	lcd.setCursor(0,1);
    lcd.print("Simulacao em : ");
    for (int i = 3; i >= 0; i--){
      	lcd.setCursor(15,1);
      	lcd.print(i);
      	delay(1000);
    }
    // Condições iniciais para as variáveis simuladas
    // Foguete na vertical
    gyroX = 0;
    gyroY = 0;
  	lcd.clear();
}

void loop()
{
    deltat = (float)(millis() - ultimoTempo)/1000.0f;
  	ultimoTempo = millis();
    Simular(deltat);										// Simula a captura de dados do giroscópio com base na vel. angular
    AtualizarVelAngular();
	
  	Serial.print(gyroX);
  	Serial.print(",");
  	Serial.println(gyroY);
  	
    lcd.setCursor(0,0);									
  	printLcd("X=",gyroX);
    
  	lcd.setCursor(8,0);
  	printLcd("Y=",gyroY);

    lcd.setCursor(0,1);
    printLcd("Wx=",vel_angular[0]);
  
    lcd.setCursor(8,1);
    printLcd("Wy=",vel_angular[1]);

   	digitalWrite(ledV, HIGH);
    digitalWrite(ledG, HIGH);
  
    if(abs(gyroX) > 1 || abs(gyroY) >1){
        digitalWrite(ledG, LOW);
        digitalWrite(ledV, HIGH);
    }else{
        digitalWrite(ledV, LOW);
        digitalWrite(ledG, HIGH);
    }
    // Se estivéssemos utilizando um giroscópio MPU6050:
    // gyro.update();
    // gyroX = gyro.getAngleX();
    // gyroY = gyro.getAngleY();

    if(millis() - ultimaPerturbacao > 700) {				//verifica se já houve tempo o suficiente para uma proxima perturbação
        Perturbar();										//gera uma perturbação aleatoria
        ultimaPerturbacao = millis();						//grava o momento da ultima perturbação
    }          

  
    if((abs(gyroX) > 0.1) || (abs(gyroY) > 0.1)){ 			// Estamos estabilizados?
      	//Não estamos estabilizados. Precisamos aplicar uma força
       
        t = 0.1f;
        accel_x = (-2.0f*((gyroX*pi/180.0f) + vel_angular[0]*t))/(t*t);		// Equação 11
        accel_y = (-2.0f*((gyroY*pi/180.0f) + vel_angular[1]*t))/(t*t);		//	Equação 11

        Fx = m*(L/2)*accel_x;												// Equação 10
        Fy = m*(L/2)*accel_y;												// Equação 10
        if(Fx/Fth < 1){
          theta1 = (a_b)*asin(Fx/Fth)*180.0f/pi;							// Equação 7
        }
      	if(Fy/Fth < 1){
          theta2 = (a_b)*asin(Fy/Fth)*180.0f/pi;							// Equação 8
        }

      
        motor1.write(90 +(int)theta1);										// Somado de 90 graus para possibilitar deflexões negativas
        motor2.write(90 +(int)theta2);

    
    }

    gyroX_last = gyroX;
    gyroY_last = gyroY;

  
}

void AtualizarVelAngular(){
    // VelAngular = rads/s 
    if(deltat != 0){
        vel_angular[0] += (accel_x * deltat)  ;
        vel_angular[1] += (accel_y * deltat)  ;
        
    }
}


void Perturbar(){
    vel_angular[0] = ((rand() % 400)-200)*pi/180.0f;//gera uma velocidade angular 0 aleatoria entre -150 e 150 graus/s
    vel_angular[1] = ((rand() % 400)-200)*pi/180.0f;//gera uma velocidade angular 1 aleatoria entre 0 e 5 graus/s
}

// Precisamos simular a entrada do giroscópio:
void Simular(float deltat){
    gyroX = gyroX_last + vel_angular[0]*deltat*180.0f/pi + (accel_x * deltat * deltat)/2.0f;
    gyroY = gyroY_last + vel_angular[1]*deltat*180.0f/pi + (accel_y * deltat * deltat)/2.0f;
}

void printLcd(String texto, float valor){
  	if(valor >= 0){
      	lcd.print(texto+" ");//escreve "X=" no lcd
    	lcd.print(valor);//escreve o angulo gyroX no lcd
  	}else{
      	lcd.print(texto);//escreve "X=" no lcd
    	lcd.print(valor);//escreve o angulo gyroX no lcd
  	}
}




// 74XX4051 ADDRESS PINS :
#define S0 4
#define  S1 0
#define  S2 2
#define S3 15

// 74XX4051 ANALOG PIN :
#define  Z 13
int analogVal[16];
void setup()
{
  // CONFIGURE ADDRESS PINS
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
    pinMode(S3, OUTPUT);


  Serial.begin(115200);
}

void loop () 
{
  //000,001,010
  // LOOP THROUGH ALL THE ADDRESSES
  for (int count=0;count<16;count++)
  {
     // SET THE ADDRESS
     digitalWrite(S0, bitRead(count, 0) );
     digitalWrite(S1, bitRead(count, 1) );
     digitalWrite(S2, bitRead(count, 2) );
          digitalWrite(S3, bitRead(count, 3) );


     
     // READ THE ANALOG FOR THAT ADDRESS
     int reading = analogRead(Z);
     //you can add the output of anothe mux here!
     int channel = count;
     // SERIAL OUTPUT
     // print something like : A0-### value
     Serial.print("CHANNEL_");
     Serial.print(count);
     Serial.print("=");
     Serial.print(reading);
     Serial.print("  ");
     //delay(100);
   }
   Serial.println();Serial.println();
   delay(2000); // to observe the values
  
}
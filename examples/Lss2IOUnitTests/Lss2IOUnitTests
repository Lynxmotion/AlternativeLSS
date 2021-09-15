// Edited by Eric Nantel
// Changed the sensibility of the pins tests

// RGB pins
int red_light_pin= 3;
int green_light_pin = 6;
int blue_light_pin = 5;

// 2RC Mode
char mystr[11] = "Results:";
char mystr2[11];
char mystr3[2] = "\n";
char mystr4[2];

//Arduino Mode
char mystr11[11] = "Results:";
char mystr22[11];
char mystr33[2] = "\n";
char mystr44[2];

// Serial buffer size
#define CMDBUFFER_SIZE 32

// Digital Analog Error Token 
bool pinD_A_Error = false;

void setup() {
  Serial.begin(115200);
  
  //Set RGB pins as outputs
  pinMode(red_light_pin, OUTPUT);
  pinMode(green_light_pin, OUTPUT);
  pinMode(blue_light_pin, OUTPUT);
  
  //Set digital pins as outputs
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(14, OUTPUT);
  pinMode(7, OUTPUT);
  digitalWrite(7,HIGH);
  digitalWrite(14,HIGH);

  //Reset the RGB
  RGB_color(255, 255, 255);
}

void loop() {
// Step 1 : Test RGB LEDS
  Serial.println("///////////////////////////////////////////");
  Serial.println("/////////////// Testing RGB ///////////////");
  Serial.println("///////////////////////////////////////////");
  Serial.println(" ");

  Serial.println("Directions Testing RGB:");
  Serial.println("  1.  Look at the RGB LED and verify that each color is displayed");
  Serial.println("  2.  Press ENTER to proceed");
  Serial.println(" ");

  while (!Enter_Event())
    {Blink_Wait();}

  Serial.println("RGB should be blinking RED");
  Serial.println("Press ENTER to Confirm");
  Serial.println("");
  while (!Enter_Event())
    {Blink_Red();}
  
  Serial.println("RGB should be blinking GREEN");
  Serial.println("Press ENTER to Confirm");
  Serial.println("");
  while (!Enter_Event())
    {Blink_Green();}
  
  Serial.println("RGB should be blinking BLUE");
  Serial.println("Press ENTER to Confirm");
  Serial.println("");
  while (!Enter_Event())
    {Blink_Blue();}

  Serial.println("Press ENTER to proceed with the next Test");
  Serial.println("");
  while (!Enter_Event())
    {Blink_Wait();}
  
// Step 2 : Test Analog/Digital pins:
  Serial.println("///////////////////////////////////////////");
  Serial.println("/////// Testing Digital/Analog Pins ///////");
  Serial.println("///////////////////////////////////////////");
  Serial.println(" ");

  Serial.println("Directions Testing Digital / Analog Pins:");
  Serial.println("  1.  Connect the following");
  Serial.println("  2.  D11 to 17(A3)");
  Serial.println("  3.  D10 to 18(A4)");
  Serial.println("  4.  D9  to 19(A5)");
  Serial.println("  5.  Press ENTER to proceed");
  Serial.println(" ");
  
  while (!Enter_Event())
    {Blink_Wait();}
    
  Serial.println("Results:");
  Test_pins(11,17);
  Test_pins(10,18);
  Test_pins(9,19);
//  Serial.println("Successfully tested all Digital/Analog pins");
  if (pinD_A_Error == true){
    RGB_color(245, 255, 255);
  }
  else{
    RGB_color(255, 245, 255);
  }

  // Waiting for the Tester to see the results
  Serial.println(" ");
  Serial.println("Press ENTER to proceed with the next Test");
  Serial.println("");
  while (!Enter_Event())
    {}
  RGB_color(255, 255, 255); // Reset color back to OFF
    
// Step 3 : Test Tristate Buffers and Switch:
  Serial.println("///////////////////////////////////////////");
  Serial.println("/// Testing Tristate Buffers and Switch ///");
  Serial.println("///////////////////////////////////////////");
  Serial.println(" ");
  Serial.println("Directions Testing 2RC Buffer:");
  Serial.println("  1.  Connect LSS port TX in RX");
  Serial.println("  2.  Place switch in 2RC Mode");
  Serial.println("  3.  Make sure that the line ending is set to : Carriage Return");  
  Serial.println("  4.  Press ENTER to proceed");
  Serial.println(" ");
    while (!Enter_Event())
  {Blink_Wait();}

  Test_Tristate_2RC();
  
  // Waiting for the Tester to see the results
  Serial.println(" ");
  Serial.println("Press ENTER to proceed with the next Test");
  Serial.println(" ");
  while (!Enter_Event())
    {}
  
  // Reset the RGB
  RGB_color(255, 255, 255); // Reset color back to OFF

  // Waiting for the Tester to see the results
  Serial.println("Directions Testing CH340E Buffer:");
  Serial.println("  1.  Place switch in Arduino Mode"); 
  Serial.println("  2.  Press ENTER to proceed");
  Serial.println(" ");
  while (!Enter_Event())
    {Blink_Wait();}
  
  // Reset the RGB
  RGB_color(255, 255, 255); // Reset color back to OFF

  // Testing the CH340E Tristate Buffer
  Test_Tristate_CH340E();

  Serial.println(" ");
  Serial.println("*** ALL TESTS DONE / CORRECT ANYTHING THAT DIDN'T WORKED ***");

  // End of tests
  while (1) {}
}

void Test_Tristate_2RC(){
  digitalWrite(14,HIGH);  // Enable TX Arduino to TX LSS (enable)
  digitalWrite(7,LOW);    // Disable TX USB to RX Arduino (disable)
  delay(50);
  Serial.write(mystr,11);
  Serial.readBytes(mystr2,11);
  if (mystr[0] == mystr2[0] && mystr[1] == mystr2[1] && mystr[2] == mystr2[2] && mystr[3] == mystr2[3] && mystr[4] == mystr2[4] && mystr[5] == mystr2[5] && mystr[6] == mystr2[6] && mystr[7] == mystr2[7] && mystr[8] == mystr2[8] && mystr[9] == mystr2[9] && mystr[10] == mystr2[10]) {
    digitalWrite(14,LOW); // Enable 2RC Triste
    delay(10);
    Serial.flush();
    Serial.write(mystr3,2);
    Serial.readBytes(mystr4,2);
    if (mystr3 != mystr4) {Serial.println("   OK:  Tristate Test Succeeded");}
    RGB_color(255, 245, 255);
  }
  else 
  {
    Serial.println("");
    Serial.println("  ERROR:  Verify if LSS Tx and Rx are properly connected and Switch in 2RC Mode");
    RGB_color(245, 255, 255);
  }    
  digitalWrite(7,HIGH);
  digitalWrite(14,LOW);
}    

void Test_Tristate_CH340E(){
  digitalWrite(7,LOW);
  delay(10);
  Serial.write(mystr11,11);
  Serial.readBytes(mystr22,11);
  if (mystr11[0] == mystr22[0] && mystr11[1] == mystr22[1] && mystr11[2] == mystr22[2] && mystr11[3] == mystr22[3] && mystr11[4] == mystr22[4] && mystr11[5] == mystr22[5] && mystr11[6] == mystr22[6] && mystr11[7] == mystr22[7] && mystr11[8] == mystr22[8] && mystr11[9] == mystr22[9] && mystr11[10] == mystr22[10]) {
    Serial.println("");
    Serial.println("  OK:  Tristate Test Succeeded");
    RGB_color(255, 245, 255);
  }
  else  {
    Serial.println("");
    Serial.println("  ERROR:  Verify if LSS Tx and Rx are properly connected and Switch in Arduino Mode");
    RGB_color(245, 255, 255);
  }    
} 

void RGB_color(int red_light, int green_light, int blue_light){
  analogWrite(red_light_pin, red_light);
  analogWrite(green_light_pin, green_light);
  analogWrite(blue_light_pin, blue_light);
}

void Test_pins(int digital_pin_number, int analog_pin_number){
  digitalWrite(digital_pin_number,LOW);
  delay(10);
  if (analogRead(analog_pin_number) < 20)
  {
    digitalWrite(digital_pin_number,HIGH);
    delay(10);
    if (analogRead(analog_pin_number) > 1004)
    {
      digitalWrite(digital_pin_number,LOW);
      delay(10);
      if (analogRead(analog_pin_number) < 20)
      { 
      Serial.print("  OK: tested pin D");
      Serial.print(digital_pin_number);
      Serial.print(" and ");
      Serial.println(analog_pin_number);
      }
       else
      {
      pinD_A_Error = true;
      Serial.print("  ERROR:  Please check if you have properly connected D");
      Serial.print(digital_pin_number);
      Serial.print(" to ");
      Serial.println(analog_pin_number);
      }
    }
    else
    {
      pinD_A_Error = true;
      Serial.print("  ERROR:  Please check if you have properly connected D");
      Serial.print(digital_pin_number);
      Serial.print(" to ");
      Serial.println(analog_pin_number);
    }
  }
  else
  {
    pinD_A_Error = true;
    Serial.print("  ERROR:  Please check if you have properly connected D");
    Serial.print(digital_pin_number);
    Serial.print(" to ");
    Serial.println(analog_pin_number);
  }
}

bool Enter_Event(){ 
  char c;
  while(Serial.available()){
    c = Serial.read();
    if (c == '\r'){
      return true; 
    }
    else{
      return false;
    } 
  }
}
 
void Blink_Wait(){
  RGB_color(245, 255, 255);
  delay(100);
  RGB_color(255, 245, 255);
  delay(100);
  RGB_color(255, 255, 245);
  delay(100);
  RGB_color(255, 255, 255); 
} 

void Blink_Red(){
  RGB_color(245, 255, 255);
  delay(100);
  RGB_color(100, 255, 255);
  delay(100); 
} 

void Blink_Green(){
  RGB_color(255, 245, 255);
  delay(100);
  RGB_color(255, 220, 255);
  delay(100); 
} 

void Blink_Blue() {
  RGB_color(255, 255, 245);
  delay(100);
  RGB_color(255, 255, 200);
  delay(100); 
} 
 

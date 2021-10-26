/*******************************
   SARSAT decodeur
   Oled Display 128x64
   F4GMU - F1GHO
   09/2021

  V 9.0 fonction GPS et BlueTooth vérifiés OK
  V10   remis en forme transfert des valeurs reçues dans octets et bits
       changer mode de detection synchro
  V11  balayage ecran toutes les 8 secondes...evite le reset watchdog!!     
********************************/

String Version = "11";
String nom_bt = "RX406_F1GHO";

////////////////////////////////////////////////
//voir ligne 70-71 pour choix ecran
#include <SSD1306.h>    // pour ecran 0.96 pouce
#include <SH1106.h>     // pour ecran 1.3 pouce



#include <SPI.h>
#include <math.h>
#include <Wire.h>
#include <BluetoothSerial.h>
#include <axp20x.h> // pour gérer les alimentations des différents éléments (prendre lib f1gho modifiée)
#include <TinyGPS++.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "Variables.h"
#include <SSD1306.h>
#include "Code_Baudot.h"

TinyGPSPlus gps;
AXP20X_Class axp;
HardwareSerial GPS_NEO(1);  // GPS NEO6m sur UART 1
TinyGPSCustom alti_GPS(gps, "GPGGA", 9);// defini la trame de recherche et la position de la donnée
TinyGPSCustom lat_GPS (gps, "GPGGA", 2);
TinyGPSCustom lng_GPS (gps, "GPGGA", 4);

const unsigned char UBLOX_INIT[] PROGMEM = {
  0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A //(5Hz)
};
const unsigned char UBLOX_SAVE[] PROGMEM = {
  0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0x31, 0xBF
};

//++++++++++ BROCHAGE DU CIRCUIT  ++++++++++//
// définitions raccordement du module TTGO OLED V1
#define GPS_RX   12
#define GPS_TX   34
#define Data_rx  4

#define BP_poussoir 15          // poussoir encodeur
const byte pinEncodeurA =  2 ;  // borne 1 encodeur
const byte pinEncodeurB = 13 ;  // borne 2 encodeur

#define TEMPS_ANTI_REBOND_ROT 120  // ajuster ce temps (ms)en fonction de la qualité du rotatif
#define ESP32_DISABLE_BROWNOUT_DETECTOR 0
#define WDT_TIMEOUT 3

// définitions  des objets et raccordement
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
BluetoothSerial SerialBT;

// suivant afficheur commenter la ligne non utile
SSD1306  display(0x3c, 21, 22); // adresse de l'afficheur,  pin pour SDA  et SCL   // pour ecran 0.96 pouce
//SH1106  display(0x3c, 21, 22); // adresse de l'afficheur,  pin pour SDA  et SCL  // pour ecran 1.3 pouce

///////////////// Forward declaration  /////////////////////////
void IRAM_ATTR Isr_BP_V();
void IRAM_ATTR encodeurTickA();

/////////////////////////////////////
void setup()
{
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);  // brownout detector mis hors service

  pinMode(Data_rx, INPUT);          // entree sur interruption
  pinMode(BP_poussoir, INPUT_PULLUP);
  pinMode(pinEncodeurA, INPUT_PULLUP);
  pinMode(pinEncodeurB, INPUT_PULLUP);

  SPI.begin();
  Serial.begin(115200);
  Wire.begin(21, 22); /// connection du CI AXP192 et ecran OLED
  if (!axp.begin(Wire, AXP192_SLAVE_ADDRESS)) Serial.println("AXP192 Begin OK"); else Serial.println("AXP192 Begin Defaut");

  // mets toutes les alims  AXP192 sur on
  axp.setPowerOutPut(AXP192_LDO2, AXP202_ON); // Lora on
  axp.setPowerOutPut(AXP192_LDO3, AXP202_ON); // Gps on
  axp.setPowerOutPut(AXP192_DCDC1, AXP202_ON); // OLED on
  axp.setPowerOutPut(AXP192_DCDC3, AXP202_ON); // 3.3v pour  ESP32
  axp.setDCDC1Voltage(3300); //regle le bus 3.3V  à 3.3V.
  axp.setPowerOutPut(AXP192_DCDC2, AXP202_ON);
  axp.setPowerOutPut(AXP192_EXTEN, AXP202_ON);

  /// registre 0x36  press key parametres
  axp.setStartupTime(10); // 10=start en 512mS  11= 1Sec. 01= 256mS 00 = 128mS
  axp.setlongPressTime(0x10); // temps de presslong 0x10 = 2 Sec. 0x11 = 2.5 Sec.
  axp.setTimeOutShutdown(true); // si appui > long que presstimelong   enable ou disable
  axp.setShutdownTime(0x0);  // 0x0 = stop 4sec. 0x01= 6s  0x10 8s 0x11=10s
  axp.enableChargeing(true);
  axp.setChargingTargetVoltage(AXP202_TARGET_VOL_4_15V); // val possibles: 4_1V, 4_15V, 4_36V
  axp.adc1Enable(AXP202_BATT_CUR_ADC1, 1);
  axp.enableIRQ(AXP202_VBUS_REMOVED_IRQ | AXP202_VBUS_CONNECT_IRQ | AXP202_BATT_REMOVED_IRQ | AXP202_BATT_CONNECT_IRQ, 1);
  axp.clearIRQ();

  baChStatus = "Decharge";
  if (axp.isChargeing()) baChStatus = "Charge";
  Serial.println(baChStatus);

  if (axp.isBatteryConnect()) {
    snprintf(volbuffer, sizeof(volbuffer), "%.2fV/%.1fmA", axp.getBattVoltage() / 1000.0, axp.isChargeing() ? axp.getBattChargeCurrent() : axp.getBattDischargeCurrent());
    Serial.println(volbuffer);
  }
  //////  Bluetooth/////////////////////////////////////
  SerialBT.begin(nom_bt); //nom Bluetooth
  Serial.println("Bluetooth démarré! peut être appairé...");

  display.init();  // init de l'ecran OLED
  delay(100);
  display.clear();  // efface tout
  display.flipScreenVertically();  //permet de choisir le sens de l'afficheur
  display.setContrast(255);
  display.setFont(ArialMT_Plain_10);// taille de police 10, 16, 24...

  //////////////  PARAMETRAGE DU GPS  /////////////////////////////////
  GPS_NEO.println("$PUBX,40,VTG,1,0,0,0,0,0*5F"); // desactive la trame gps
  GPS_NEO.println("$PUBX,40,GSA,1,0,0,0,0,0*4F");
  GPS_NEO.println("$PUBX,40,GSV,1,0,0,0,0,0*58");
  GPS_NEO.println("$PUBX,40,GLL,1,0,0,0,0,0*5D");
  ////////////////  mets en cycle choisi dans le fichier
  for (unsigned int i = 0; i < sizeof(UBLOX_INIT); i++) {  // pour régler le cycle envoi trame 1Hz, 5Hz ou 10 Hz
    GPS_NEO.write(pgm_read_byte(UBLOX_INIT + i) );         // choisir dans le UBLOX_INIT
  }
  ///////////////  sauve la configuration en eeprom du GPS
  for (unsigned int i = 0; i < sizeof(UBLOX_SAVE); i++) {  // pour sauvegarder les réglages
    GPS_NEO.write(pgm_read_byte(UBLOX_SAVE + i) );
  }
  // GPS_NEO.end();

  display.clear();
  display.setFont(ArialMT_Plain_16);// taille de police 10, 16, 24...
  display.drawString(0, 10, "Rx 406MHz V" + Version);
  display.drawString(0, 27, "BALISE SARSAT");
  display.drawString(13, 45,  "F1GHO  2021");
  display.display();
  display.setFont(ArialMT_Plain_10);// taille de police 10, 16, 24...
  delay(2000);

  attachInterrupt(pinEncodeurA, encodeurTickA, FALLING); // entree encodeur rotatif
  attachInterrupt(Data_rx, analyse_signal, CHANGE);  // interruption sur Rise et Fall
  //  fab_trame();
}

////////////////////////////////////////////////////
void IRAM_ATTR encodeurTickA()
{
  detachInterrupt(pinEncodeurA);
  //  detachInterrupt(Dio1);

  unsigned long interval = millis();
  if ((interval - TimeDernierChangement) > TEMPS_ANTI_REBOND_ROT) {
    if (digitalRead(pinEncodeurB) == HIGH)
    { //  sens rotation positif
      sens_encodeur = 1;
    }
    else { //  sens rotation negatif
      sens_encodeur = -1;
    }
    TimeDernierChangement = interval;
  }

  attachInterrupt(pinEncodeurA, encodeurTickA, FALLING); // entree encodeur rotatif
}

/////////////////////////////////////////////
void loop()
{
  if (!decod_fini && count_oct >= Nb_oct) {
    detachInterrupt(Data_rx);  // interruption sur Rise et Fall
    octet[0] = 0xFF; octet[1] = 0xFE; octet[2] = trame_synchro & 0x000000FF;

    if (octet[2] == 0xD0 ||  octet[2] == 0x2F) {
      for ( byte i = 0; i < count_oct ; i++) {  // affichage de la trame en Hexa
        if (octet[i] < 16) Serial.print('0'); Serial.print (octet[i], HEX);
      } Serial.println();

      decodage_type_code();
      decod_fini = 1;
      Num_ecran = 0;
      impression();
      voltmetre();
      affiche_ecran(0);
      envoi_bluetooth();
      acquisition_GPS();
      if (gps_valid == true) {
        calcul_dist_azimut(Plat2, Plon2, latit_GPS, longi_GPS);
        gps_valid = false;
        Serial.println(def_dist);
        Serial.print(azim_bal); Serial.println("°");
        SerialBT.println("Distance Balise: " + def_dist + "Azimut : " + azim_bal); SerialBT.println("°");
        SerialBT.println("=====================");
        SerialBT.println();
      }
      trame_synchro = 0;
      count = 0;
      start_flag = 0;
      data_demod = 0;
      der_bit = 0;
      etat = 1;
      decod_fini = 1; // repart pour trame suivante
      delay(250); // retard  sur remise interuption pour ne pas prendre des zeros qui suivent la trame
      attachInterrupt(Data_rx, analyse_signal, CHANGE);  // interruption sur Rise et Fall

    }          // repart pour trame suivante
  }

  if ( Serial.available() > 0) Serial_trame();
/*  
  if (inter_timer) {
//    timerWrite(timer, 0); //reset le timer
//    timerAlarmEnable(timer); //revalide la fonction timer
    inter_timer = false;
    voltmetre();
    acquisition_GPS(0);

    if (gps_valid == true) {
      calcul_dist_azimut(Plat2, Plon2, latit_GPS, longi_GPS);
      gps_valid = false;
    }
    affiche_ecran(Num_ecran);
    SerialBT.print("Distance Balise: " + def_dist + "   Azimut : " + azim_bal); SerialBT.println("°");
    SerialBT.println("================================");
    SerialBT.println();
  }
*/

  if (sens_encodeur == 1) {
    Num_ecran++;
    if (Num_ecran > 4) Num_ecran = 0;
    affiche_ecran(Num_ecran);
    sens_encodeur = 0;
  }
  else   if (sens_encodeur == -1) {
    Num_ecran--;
    if (Num_ecran < 0) Num_ecran = 4;
    affiche_ecran(Num_ecran);
    sens_encodeur = 0;
  }
  if ( Num_ecran == 4 && digitalRead(BP_poussoir) == LOW) {  // choix OUI repartir en reception trame nouvelle
    for ( byte i = 0; i < count_oct ; i++) {
      octet[i] = 0x00;
    }
    decod_fini = 0;
    count_oct = 0;
    Num_ecran = 0;
    affiche_attente();
    attachInterrupt(Data_rx, analyse_signal, CHANGE);  // interruption sur Rise et Fall

  }
  temps_ecoule = millis() - millisecondes;
  if (temps_ecoule > 8000){
    millisecondes = millis();
    voltmetre();
    acquisition_GPS();
    if (gps_valid == true) {
      calcul_dist_azimut(Plat2, Plon2, latit_GPS, longi_GPS);
      gps_valid = false;
    }
    SerialBT.print("Distance Balise: " + def_dist + "   Azimut : " + azim_bal); SerialBT.println("°");
    SerialBT.println("================================");
    SerialBT.println();

    Num_ecran++;
    if (Num_ecran > 4) Num_ecran = 0;
    affiche_ecran(Num_ecran);
  }
}

///////////////////////////////////////////////////
void affiche_attente() {
  display.clear();
  display.setFont(ArialMT_Plain_10);// taille de police 10, 16, 24...
  display.drawString(20, 2, "PATIENTEZ...");
  display.drawString(10, 15, "ATTENTE RECEPTION");
  display.drawString(20, 28, "D'UNE TRAME...");
  display.display();
  Num_ecran = 5;
  Serial.println( "PATIENTEZ... ATTENTE RECEPTION D'UNE TRAME...");
}

////////////////////////////////////////////////////////////////////////////////
///  Extraction des bits emis à 400bits/s soit 2,5mS par bit en mode NRZ 
void analyse_signal() {
  duree_palier = micros() - microseconds;
  microseconds = micros();

  if (duree_palier > 1150 && duree_palier < 1350 && der_bit == 0) {   // si T = 1,25ms ET der_bit == 0 --> der_bit = 1
    der_bit = 1;  return;
  }
  else if (duree_palier > 1150 && duree_palier < 1350 && der_bit == 1) {    // si T = 1,25ms ET der_bit == 1 --> der_bit = etat
    der_bit = 0;
  }
  else if (duree_palier > 2400 && duree_palier < 2600) {    // si T = 2,50ms --> bit = !etat
    etat = !etat;
    der_bit = 0;
  }
  else if (duree_palier > 1350 && duree_palier < 2400)  return;

  if (!start_flag)  trame_synchro = trame_synchro << 1 | etat; // ajout du bit en fin
  else if (start_flag)  data_demod = data_demod << 1 | etat; // ajout du bit en fin
  Test();
}

//////////////////////////////////////////////////////
//  TEST SYNCHRO, ET TRANSFERTS DANS OCTETS ET BITS //
//////////////////////////////////////////////////////
void Test() {
  if (!start_flag) {
    trame_synchro &= 0x00FFFFFF;
    if (trame_synchro == 0xFFFED0 || trame_synchro == 0xFFFE2F) {
      start_flag = 1;
      count_bits = 25;
      count = 0;
      count_oct = 3;
      data_demod = 0;
    }
  }
  else if (start_flag) {
    bits[count_bits] = etat;
    if (bits[25] == 1) Nb_oct = 18; else Nb_oct = 14;
    count_bits ++;
    count ++;
    if (count == 8) {                   //si nombre de bits = 8
      octet[count_oct] = data_demod;    //data dans octet numero xx
      count = 0;
      count_oct ++;
      data_demod = 0;
      if (count_oct >= Nb_oct ) {
        detachInterrupt(Data_rx);  // interruption sur Rise et Fall
        decod_fini = 0;
      }
    }
  }
}

//////////////////////////////
void decodage_type_code() {
//  int a = 25;
//  for ( byte i = 3; i < count_oct ; i++) {
//    for ( int b = 7; b > -1; b--) {
//      bits[a] = bitRead(octet[i], b);
//      //     Serial.print (bits[a]);
//      a++;
//    } //Serial.println("");
//  }
//  //Serial.println();

  if (octet[2] == 0xD0)  type_alerte = "AutoTest 406";  else if (octet[2] == 0x2F) type_alerte = "!! DETRESSE 406";

  longtrame = bits[25]; // bit 25 taille de la trame   1->longue trame  0->trame courte
  flag_protocol = bits[26]; // bit 26 flag_protocol de type de encodage de localisation

  pays = lecture_bits(27, 36);
  switch (pays) // en fonction de la valeur
  {
    case 226:
    case 227:
    case 228: str_pays = "FRANCE" ; break;
    case 205: str_pays = "BELGIQUE" ; break;
    case 211: str_pays = "ALLEMMAGNE" ; break;
    case 247: str_pays = "ITALIE" ; break;
    case 253: str_pays = "LUXEMBOURG" ; break;
    case 254: str_pays = "MONACO" ; break;
    case 269: str_pays = "SUISSE" ; break;
    case 224:
    case 225: str_pays = "ESPAGNE" ; break;
    case 232:
    case 233:
    case 234:
    case 235: str_pays = "ANGLETERRE" ; break;
    case 257: str_pays = "NORVEGE" ; break;
    default:  str_pays = String(pays, DEC);
      /*
        226-227-228 FRANCE
        329 GUADELOUPE
        347 MARTINIQUE
        361 ST PIERRE ET MIQUELON
        501 TERRE ADELIE
        540 NOUVELLE CALEDONIE
        546 POLYNESIE FRANCAISE
        578 WALLIS ET FUTUNA
        607 ST PAUL ET AMSTERDAM
        618 CROZET
        635 KERGUELEN
        660 REUNION
        745 GUYANE
      */

  }

  if (longtrame == 1 && flag_protocol == 0)
  {
    protocol_code = lecture_bits(37, 40); //(octet[4] & 0x0F); //  bits 37-40
    switch (protocol_code) // en fonction de la valeur du flag
    {
      ///// Standard location
      case 2: Type = "Maritime EPIRB";   break;  //EPIRB - MMSI/Location Protocol
      case 3: Type = "Aviation ELT-24bit"; break; //ELT - 24-bit Address/Location Protocol
      case 4: Type = "Serial-ELT";            break;  //ELT - serial 0100
      case 5: Type = "Serial ELT-aircraft";   break;  //ELT - aircraft operator designator 0101
      case 6: Type = "Serial EPIRB";          break;  //EPIRB-serial 0110
      case 7: Type = "Serial PLB";            break;  //PLB-serial 0111
      case 12: Type = "Ship Security";         break;
      case 14: Type = "Test_loc";              break; //Standard Test Location Protocol
      /////  National location
      case 8: Type = "ELT";       break;   //ELT - serial 0100
      case 10: Type = "EPIRB";    break;   //EPIRB-serial
      case 11: Type = "PLB";      break;   //PLB
      case 15: Type = "Test_loc"; break;   //National Test Location Protocol
    }

    if (protocol_code == 8 || protocol_code == 10 || protocol_code == 11 || protocol_code == 15)
      Decod_national_loc_long();
    else Decod_standard_loc_long();
  }

  else  if (flag_protocol == 1) // avec longtrame = 1 ou 0
  {
    protocol_code = lecture_bits(37, 39); //(octet[4] & 0x0E) >> 1);   // bit 37-39
    switch (protocol_code) // en fonction de la valeur du flag
    {
      case 1: Type = "ELT-Aviation";   break;        //ELT - Aviation User Protocol (aircraft registration markings)
      case 2: Type = "Maritime EPIRB"; break;        //EPIRB - Maritime User Protocol: (MMSI, 6 digits)ou radio call sign, 6 characters
      case 4: Type = "National";       break;        //National User Protocol
      case 6: Type = "Radio call sign EPIRB"; break; //EPIRB - Radio Call Sign User Protocol
      case 7: Type = "Test";           break;        //Test User Protocol
      case 3: Type = "Serial";
        Serial_user_protocol = lecture_bits(40, 42); // en fonction de la valeur des bits 40-42
        switch (Serial_user_protocol) // en fonction de la valeur du flag
        {
          case 0: serial_user_prot  = "ELTs with serial identification number"; break;
          case 1: serial_user_prot  = "ELTs with aircraft operator designator & serial number"; break;
          case 2: serial_user_prot  = "float free EPIRBs with serial identification number"; break;
          case 3: serial_user_prot  = "ELTs with aircraft 24-bit address"; break;
          case 4: serial_user_prot  = "non float free EPIRBs with serial identification number"; break;
          case 6: serial_user_prot  = "PLBs with serial identification number"; break;
        }

        //        bit_43 = bits[43] ; break;
        //bit 43 = 0: serial identification number is assigned nationally
        //bit 43 = 1: identification data include the C/S type approval certificate number

        if (longtrame == 1 ) Decod_User_loc_long(); else  Decod_User_loc_court();
    }
  }
}

/////////////////////////// DECODAGE EN FONCTION protocol_ CODE ET LONG. TRAME  ///////////////////////////////////////
//--------------------------------------------------------------------------------------------------------------------
////////////////////////////////////////////////////////////////////
void Decod_national_loc_long() //Nat loc protocole trame longue
{
  def_prot_TFT = "National Protocol";
  def_protocol = "National Location Protocol long";

  matric = lecture_bits(41, 58);
  Specific = "Serial Nb= " + String(matric, DEC);

  latflag = bits[59];
  latdeg = lecture_bits(60, 66);  // par pas de 1 degre  0--90
  latmin = lecture_bits(67, 71);  // par pas de 2 min  0--58
  latmin = (latmin * 2);
  latoffset = bits[113];   // Latitude NAT LOCATION
  latofmin = lecture_bits(114, 115); // par pas de 1 min 0--3
  latofsec = lecture_bits(116, 119); // par pas de 4 sec  0--56
  latofsec = (latofsec * 4);

  if (latoffset == 1) {
    latofsec = (latofsec * 100 / 36);
    latmin = (((latmin + latofmin) * 1000 / 6) + latofsec);
  }
  else if (latoffset == 0) {
    latmin = (latmin - latofmin);
    if (latmin < 0) {
      latmin = (60 - abs(latmin));
      latdeg = (latdeg - 1);
    }
    latmin = ((latmin * 1000 / 6) - (latofsec * 100 / 36));
  }

  lonflag = bits[72];
  londeg = lecture_bits(73, 80);
  lonmin = lecture_bits(81, 85);
  lonmin = (lonmin * 2);
  lonoffset = bits[120];       //Longitude NAT LOCATION
  lonofmin = lecture_bits(121, 122);
  lonofsec = lecture_bits(123, 126);
  lonofsec = (lonofsec * 4);

  if (lonoffset == 1) {
    lonmin = (((lonmin + lonofmin) * 1000 / 6) + (lonofsec * 100 / 36));
  }
  else if (lonoffset == 0) {
    lonmin = (lonmin - lonofmin);
    if (lonmin < 0) {
      lonmin = (60 - abs(lonmin));
      londeg = (londeg - 1);
    }
    lonmin = (lonmin * 1000 / 6) - (lonofsec * 100 / 36);
  }
  if (latdeg == 127 || londeg == 255) {
    display.print("GPS non synchronise");
    Serial.println("GPS non synchronise");
  }

  if (bits[111] == 0)   Device_source_pos = "Externe Navigation Device"; else Device_source_pos = "Interne Navigation Device";
  if (bits[112] == 0) radio_loc_device = "NO or autre"; else radio_loc_device = "121.5MHz";

  matric = lecture_bits(127, 132);
  Specific  = String(matric, DEC);
  Specific  = "N°= " + Specific;
}
//////////////////////////////////////////////////
void Decod_standard_loc_long()  //Std loc protocol trame longue
{
  def_protocol = "Standard Location Protocol Long";
  def_prot_TFT = "Standard Loc";

  identification_balise_St();

  if (bits[111] == 0) Device_source_pos = "Externe"; else Device_source_pos = "Interne";

  if (bits[112] == 0) radio_loc_device = "NO or autre"; else radio_loc_device = "121.5MHz";

  //Latitude STD LOCATION
  latflag = bits[65];    // bit 65    lat sign  0 = N,  1 = S
  latdeg = lecture_bits(66, 72);
  latmin = lecture_bits(73, 74);
  latmin = (latmin * 15);
  latoffset = bits[113];
  latofmin = lecture_bits(114, 118);
  latofsec = lecture_bits(119, 122);
  latofsec = (latofsec * 4);

  if (latoffset == 1) {
    latofsec = (latofsec * 100 / 36);
    latmin = (((latmin + latofmin) * 1000 / 6) + latofsec);
  }
  else if (latoffset == 0) {
    latmin = (latmin - latofmin);
    if (latmin < 0) {
      latmin = (60 - abs(latmin));
      latdeg = (latdeg - 1);
    }
    latmin = ((latmin * 1000 / 6) - (latofsec * 100 / 36));
  }

  lonflag = bits[75];
  londeg = lecture_bits(76, 83);
  lonmin = lecture_bits(84, 85);
  lonmin = (lonmin * 15);
  lonoffset = bits[123];
  lonofmin = lecture_bits(124, 128);
  lonofsec = lecture_bits(129, 132);
  lonofsec = (lonofsec * 4);

  if (lonoffset == 1) {
    lonmin = (((lonmin + lonofmin) * 1000 / 6) + (lonofsec * 100 / 36));
  }
  else if (lonoffset == 0) {
    lonmin = (lonmin - lonofmin);
    if (lonmin < 0) {
      lonmin = (60 - abs(lonmin));
      londeg = (londeg - 1);
    }
    lonmin = (lonmin * 1000 / 6) - (lonofsec * 100 / 36);
  }

  if (latdeg == 127 || londeg == 255) {
    Serial.println( "GPS non synchro");
    display.drawString(0, 20, "GPS non synchro");
    display.display();
  }
}
/////////////////////////////////////////////////////////////////////
void Decod_User_loc_long()    // User loc protocol trame longue
{
  def_prot_TFT = "User loc.";
  def_protocol = "User location Protocol";

  Radio_location();

  if (bits[107] == 0)   Device_source_pos = "Externe"; else Device_source_pos = "Interne";

  latflag = bits[108];
  latdeg = lecture_bits(109, 115);
  latmin = lecture_bits(116, 119);
  latmin = (latmin * 4);
  if (latflag == 0) ns = "N "; else ns = "S ";

  lonflag = bits[120];
  londeg = lecture_bits(121, 128);
  lonmin = lecture_bits(129, 132);
  lonmin = (lonmin * 4);

  if (latdeg == 127 || londeg == 255) {
    display.drawString(0, 20, "GPS non synchro");
    display.display();
    Serial.println("GPS non synchronise");
  }
  //  Fin_decodage();
  // display_trame(18);
}
//////////////////////////////////////////////////////////////////////
void Decod_User_loc_court() // User protocol trame courte
{
  def_protocol = "User Protocol 22HEX. No location";
  def_prot_TFT = "User Protocol 22HEX";
  def2_prot_TFT = "No location";

  Radio_location();

  def_prot_TFT = "User Protocol";
  def2_prot_TFT = "22-HEX. No location";
}


/****************************************
  SORTIE DES INFORMATIONS
 ****************************************/
void impression() {

  if (latflag == 0) ns = "N "; else ns = "S ";
  deg_latit = String(latdeg);
  min_dec_latit = String( latmin, DEC);
  deg_latit = (deg_latit + "." + min_dec_latit);
  Plat2 = deg_latit.toFloat(), 6;

  if (lonflag == 0) ew = "E "; else ew = "W ";
  deg_longi = String(londeg);
  min_dec_longi = String( lonmin, DEC);
  deg_longi = (deg_longi + "." + min_dec_longi);
  Plon2 = deg_longi.toFloat(), 6;

  for ( byte i = 3; i < count_oct ; i++) {  // affichage de la trame en Hexa
    if (octet[i] < 16) Serial.print('0');
    Serial.print (octet[i], HEX);
  } Serial.println("");

  Serial.println("-- RESULTATS DECODAGE --");
  Serial.println(type_alerte);
  Serial.println("Protocole code: " + String(protocol_code) + "= " + Type);
  Serial.println(appellation);
  Serial.println("Specific Nb= " + Specific);
  Serial.print("Pays : ");
  Serial.println(str_pays);
  Serial.println(def_protocol);
  if (serial_user_prot != "") {
    Serial.print("Serial_user_prot:");
    Serial.println(serial_user_prot);
  }
  //  if (flag_protocol == 1) {
  Serial.print("Balise Auxillaire: ");
  Serial.println(radio_loc_device);
  //  }
  Serial.print("Navigation Device "); Serial.println(Device_source_pos);
  Serial.print(ns); Serial.print(Plat2, 4); Serial.println("°");
  Serial.print(ew);
  if (Plon2 < 10) Serial.print("00"); else if (Plon2 < 100) Serial.print("0");
  Serial.print(Plon2, 4); Serial.println("°");
  Serial.println();
}

/////////////////////////////////////
void affiche_ecran(byte Num) {
  if (Num == 0) {
    display.clear();
    display.setFont(ArialMT_Plain_10);// taille de police 10, 16, 24...
    display.drawString(0, 0, type_alerte);
    display.drawString(0, 10, "Protocol:" + String(protocol_code) + '=' + Type);
    display.drawString(0, 20, appellation );
    display.drawString(10, 30,  "N°= " + Specific);
    display.drawString(0, 40, "Pays : " + str_pays);
    display.drawString(0, 50, "Balise Auxillaire:" + radio_loc_device);
    display.display();
  }
  else if (Num == 1) {
    display.clear();
    display.setFont(ArialMT_Plain_10);// taille de police 10, 16, 24...
    display.drawString(0, 0, def_protocol);
    if (def2_prot_TFT != "") display.drawString(80, 0, def2_prot_TFT);
    display.drawString(0, 10, "Navigation Device " + Device_source_pos);
    display.setFont(ArialMT_Plain_16);// taille de police 10, 16, 24...
    display.drawString(0, 21, ns + ' ' + String(Plat2, 5) + "°");
    if (Plon2 < 10) esp = " 00"; else if (Plon2 < 100) esp = " 0";
    display.drawString(0, 37, ew + esp + String(Plon2, 5) + "°");
    display.setFont(ArialMT_Plain_10);// taille de police 10, 16, 24...
    display.drawString(0, 53, "Dist: " + def_dist);
    display.drawString(68, 53, "Azimut: " + azim_bal + "°");

    display.display();
  }
  else if (Num == 2) {
    display.clear();
    display.setFont(ArialMT_Plain_10);// taille de police 10, 16, 24...
    display.drawString(0, 0, def_protocol);
    if (def2_prot_TFT != "") display.drawString(80, 0, def2_prot_TFT);
    display.setFont(ArialMT_Plain_16);// taille de police 10, 16, 24...
    display.drawString(0, 18, "Dist: " + def_dist);
    display.drawString(0, 34, "Azimut: " + azim_bal + "°");
    display.setFont(ArialMT_Plain_10);// taille de police 10, 16, 24...
    display.drawString(0, 53,  String(latit_GPS, 5));
    display.drawString(50, 53, String(longi_GPS, 5));
    display.drawString(98, 53, String(altit_GPS));
    display.display();
  }
  else if (Num == 3) {
    String zero, hexa, hexa2;
    for ( byte i = 3; i < Nb_oct ; i++) {
      if (octet[i] < 16)  zero = '0'; else zero = "";
      if (i < 11) hexa = hexa + ' ' + zero + String(octet[i], HEX);
      if (i >= 11) hexa2 = hexa2 + ' ' +  zero + String(octet[i], HEX);
    }
    hexa.toUpperCase(); hexa2.toUpperCase();
    display.clear();
    display.setFont(ArialMT_Plain_10);// taille de police 10, 16, 24...
    display.drawString(0, 2, hexa);
    display.drawString(0, 12, hexa2);
    display.drawString(0, 30, "Etat Batterie:");
    display.drawString(20, 42, "Tension= " + String(V_bat, 2) + "V");
    display.drawString(20, 53, baChStatus + "= " + String(I_bat, 1) + "mA");
    display.display();
  }
  else if (Num >= 4) {
    display.clear();
    display.setFont(ArialMT_Plain_10);// taille de police 10, 16, 24...
    display.drawString(10, 1, "POUR RELANCER LA ");
    display.drawString(8, 12, "RECEPTION APPUYEZ");
    display.drawString(10, 25, "SUR LE SUR BOUTON");
    display.drawString(8, 38, "POUSSOIR ENCODEUR");
    display.drawString(20, 51, "SINON TOURNEZ");
    display.display();
    //    if (Num == 4) {
    //      display.drawString(27, 51, "^");
    //      display.drawString(28, 56, "|");
    //    }
    //    display.display();
    //    if (Num == 5) {
    //      display.drawString(88, 51, "^");
    //      display.drawString(89, 56, "|");
    //
    //    }
  }
  envoi_bluetooth();

}
//////////////////////////////////////////////////////////
void envoi_bluetooth() {
  for ( byte i = 3; i < count_oct ; i++) {  // affichage de la trame en Hexa
    if (octet[i] < 16) SerialBT.print('0');
    SerialBT.print (octet[i], HEX);
  } SerialBT.println("");

  SerialBT.println("-- RESULTATS DECODAGE --");
  SerialBT.println(type_alerte);
  SerialBT.println("Protocole code: " + String(protocol_code) + "= " + Type);
  SerialBT.println(appellation);
  SerialBT.println("Specific Nb= " + Specific);
  SerialBT.print("Pays : ");
  SerialBT.println(str_pays);
  SerialBT.println(def_protocol);
  if (serial_user_prot != "") {
    SerialBT.print("Serial_user_prot:");
    SerialBT.println(serial_user_prot);
  }
  //  if (flag_protocol == 1) {
  SerialBT.print("Balise Auxillaire: ");
  SerialBT.println(radio_loc_device);
  //  }
  SerialBT.print("Navigation Device "); SerialBT.println(Device_source_pos);
  SerialBT.print(ns); SerialBT.print(Plat2, 4); SerialBT.println("°");
  SerialBT.print(ew);
  if (Plon2 < 10) SerialBT.print("00"); else if (Plon2 < 100) SerialBT.print("0");
  SerialBT.print(Plon2, 4); SerialBT.println("°");
  SerialBT.println();

  SerialBT.println("Position Station Mobile:");
  SerialBT.print( String(latit_GPS, 6) + "°   ");
  SerialBT.print( String(longi_GPS, 6) + "°   Alti= ");
  SerialBT.println( String(altit_GPS) + 'm');
  SerialBT.println();

}

//////////////////////////////////////////////////////////////////
uint32_t lecture_bits(byte a, byte b) {
  byte s = (b + 1) - a;
  byte impair = s % 2; // si nombre de bits à lire est impair = 1
  uint32_t val_2bits = 0x00000000, valtot_bits = 0x00000000;

  for (byte i = a; i < b; i++) {
    val_2bits =  (bits[i] << 1) | bits[i + 1];
    valtot_bits = (valtot_bits << 2) | val_2bits;
    i = i + 1; // pour aller 2 bits plus en avant au prochain tour de la boucle
  }
  if (impair) valtot_bits = (valtot_bits << 1) | bits[b];
  return valtot_bits;
}

///////////////////////////////
void identification_balise_St() {  // code MMSI ou 24 bits addresse ou....
  if (protocol_code == 2 || protocol_code == 12) {
    matric = lecture_bits(41, 60);
    appellation  = "MMSI=" + String(matric, HEX) + " MID=" + matric;
    appellation.toUpperCase();
    byt2_bits = lecture_bits(61, 64);
    Specific = String(byt2_bits, DEC);
    if  (protocol_code == 12) byt2_bits = 0;
  }
  else if (protocol_code == 3) { // 24 bits Aircraft
    matric = lecture_bits(41, 64);
    Specific = String(matric, DEC);
    appellation  = String(matric, HEX);
    appellation.toUpperCase();
    appellation  = "Aircraft ID= " + appellation;
  }
  else if (protocol_code > 4 && protocol_code < 8) {
    if (protocol_code != 5) {
      matric = lecture_bits(41, 50);
      Specific = "Certif_balise N°= " + String(matric, DEC);
      matric = lecture_bits(51, 64);
      appellation  = String(matric, HEX);
      appellation.toUpperCase();
      appellation  = "Serial N°= " + appellation;
    }
    else if (protocol_code == 5) {
      matric = lecture_bits(41, 45); // 1er caractere
      matric = matric + 32;
      appellation = Code_Baudot[matric];

      matric = lecture_bits(46, 50); // 2e caractere
      matric = matric + 32;
      appellation = appellation + Code_Baudot[matric];

      matric = lecture_bits(51, 55); // 3e caractere
      matric = matric + 32;
      appellation = appellation + Code_Baudot[matric];

      appellation = "Op Design= " + appellation;

      matric = lecture_bits(56, 64);
      Specific  = String(matric, HEX);
      Specific.toUpperCase();
      Specific  = "N°= " + Specific;
    }
  }
}

///////////////////////////////////////////////////////////////
void fab_trame() {
  octet[0] = 0xFF;  octet[1] = 0xFE; // trame de synchro pour tous modeles
  //////  CHOISIR LA VALEUR octet(2) DESIREE  /////////////
  // octet[2] = 0xD0; // la balise est en mode autotest ///
  octet[2] = 0x2F; // la balise est en mode detresse ///
//    90127B92922BC02B4968F58450220B
//    8E23ADEC7730A0616807B7082709A3
  ///////////////////////////////////////////////////////////
  String trame_demo = "8E23ADEC7730A0616807B7082709A3"; // trame de donnees
  octet[3] = 0x8E; octet[4] = 0x23; octet[5] = 0xAD; octet[6] = 0xEC; octet[7] = 0x77; octet[8] = 0x30; octet[9] = 0xA0;
  octet[10] = 0x61; octet[11] = 0x68; octet[12] = 0x07; octet[13] = 0xB7; octet[14] = 0x08; octet[15] = 0x27; octet[16] = 0x09; octet[17] = 0xA3;
  count_oct = 18;

  //////////////////////////////////////////////
  //    String trame_demo = "8E92F1206334A0330CDFF70DE52163"; // F12063 Pays : 233 Standard Location Protocol N 52.4423° W 001.5855°
  //    octet[3] = 0x8E; octet[4] = 0x92; octet[5] = 0xF1; octet[6] = 0x20; octet[7] = 0x63; octet[8] = 0x34; octet[9] = 0xA0;
  //    octet[10] = 0x33; octet[11] = 0x0C; octet[12] = 0xDF; octet[13] = 0xF7; octet[14] = 0x0D; octet[15] = 0xE5; octet[16] = 0x21; octet[17] = 0x63;
  //    count_oct = 17;
  decodage_type_code();

}

///////////////////////////////////////////////////////////////
void Serial_trame() {
  char input[3];
  int charsRead;
  count_oct = 3;

  octet[0] = 0xFF;  octet[1] = 0xFE; // trame de synchro pour tous modeles
  //////  CHOISIR LA VALEUR octet(2) DESIREE  /////////////
  //  octet[2] = 0xD0; // la balise est en mode autotest ///
  octet[2] = 0x2F; // la balise est en mode detresse ///

  while (Serial.available() > 0) {
    charsRead = Serial.readBytesUntil('\n', input, 2);  // prends 2 caracteres
    input[charsRead] = '\0';                            //convertit en string et ajoute "0"
    unsigned long val_string = strtoul( input, NULL, 16); //convertit en valeur hexa base 16
    octet[count_oct] = val_string;
    count_oct++;
    delay(2);
  }
  if ( count_oct < 14 && count_oct > 3) {
    Serial.println("Trop court " + String(count_oct) + String(" octets"));
    count_oct = 3;
    Serial.flush();
  }
  decod_fini = 0;
  //count_oct = 18;
}


void Radio_location() {
  byte rld =  lecture_bits(84, 85); // bits 84-85 systeme de Radio Localisation auxillaire
  switch (rld) // en fonction de la valeur du flag
  {
    case 0:  radio_loc_device = "Aucune";  break;
    case 1:  radio_loc_device = "121.5Mhz";  break;
    case 2:  radio_loc_device = "SART";  break;
    case 3:  radio_loc_device = "Autre RLD";  break;
  }
}

////////////////////////////////
void acquisition_GPS() {
  GPS_NEO.begin(9600, SERIAL_8N1, GPS_TX, GPS_RX);   //34-TX 12-RX
  delay(200);
  unsigned long start = millis();
  do
  {
    while (GPS_NEO.available()) gps.encode(GPS_NEO.read());
  }
  while (millis() - start < 600); // reste env. 2 sec en réception
  GPS_NEO.end();

  if (millis() - start > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println("Pas de reception GPS verifier le cablage.");
  }
  else
  {
    if (gps.location.isValid())
    {
      gps_valid = true;
      latit_GPS = gps.location.lat();
      longi_GPS = gps.location.lng();
      altit = gps.altitude.meters();
      altit_GPS = int(altit);
    }
    else   gps_valid = false;
  }
}

/////////////////////////////////////////
void voltmetre()
{
  if (axp.isChargeing()) {
    baChStatus = "Charge";
    I_bat = axp.getBattChargeCurrent();
  }
  else {
    baChStatus = "Decharge";
    I_bat = axp.getBattDischargeCurrent();
  }
  V_bat = axp.getBattVoltage();
  V_bat /=  1000 ;
}
//////////////////////////////////////////////////////////////////

//+++++  CALCUL DE LA DISTANCE, cap et azimut en mode GPS ++++++++++
void calcul_dist_azimut( double Lat1, double Lon1, double Lat2, double Lon2) {
  if (ew == "W ") Lon1 *= -1;

  dist_calc = gps.distanceBetween(Lat1, Lon1, Lat2, Lon2 );  // exprimée en metres
  dist_bal = dist_calc;
  double bearing = gps.courseTo(Lat2, Lon2, Lat1, Lon1); // en degres en inversant les données pour avoir azimut et non course
  // bearing = angle de visée et dist_calc = distance
  azimut = int(bearing);
  azim_bal = String(azimut);

  if (dist_bal / 1000 >= 10) {
    dist_bal /= 1000;
    def_dist = String(dist_bal);
    byte longdist = def_dist.length() - 3 ;
    def_dist =  def_dist.substring(0, longdist); // pour enlever les decimales
    def_dist = def_dist + " Km";
  }
  else  if (dist_bal / 1000 < 1) {
    def_dist = String(dist_bal) + " m";
  }
}

/////////////////////////////////////////////

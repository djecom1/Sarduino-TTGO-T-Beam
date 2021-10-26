/********************************
  Definitions des variables
*********************************/
#ifndef Variables_h
#define Variables_h

bool interupt_BT;
uint16_t pays;
long latdeg,  londeg, latofmin, latofsec, lonofmin, lonofsec;
int32_t lonmin, latmin;
bool longtrame, flag_protocol, latflag, lonflag, latoffset, lonoffset;
bool Start_decod; 
byte start_flag;
byte count, count_oct, data_demod, protocol_code;
bool bloq_serialin , decod_fini;
float  V_bat, I_bat;   // pour lecture tension batterie
String Type, deg_latit, min_dec_latit, deg_longi, min_dec_longi, ns, ew ;
String immat, azim_bal, synchro_gps;
float Plat2, Plon2, Plat1, Plon1;
volatile double  dist_calc;
volatile int azimut;
volatile float dist_bal; 
String type_alerte, str_pays, def_protocol, def_prot_TFT,def2_prot_TFT;
String radio_loc_device, balise_num, serial_user_prot;
byte Serial_user_protocol, count_bits , bit_43;
uint32_t matric ;
byte octet[18]; //0...17 = 18octets  1er FF = octet[0]
byte bits[145];
byte Nb_oct = 14; // par defaut trame courte
String Device_source_pos;
String appellation, Specific, esp;
byte byt2_bits;
String  baChStatus,def_dist;
static char volbuffer[128];
volatile  unsigned long TimeDernierChangement = 0;
int sens_encodeur,Num_ecran, altit_GPS;
bool gps_valid, inter_timer;
volatile double longi_GPS, latit_GPS, altit;
uint32_t trame_synchro;
bool  der_bit = 0;
bool etat = 1;    // debut de trame à 1
uint32_t  microseconds;
uint16_t duree_palier;
uint32_t millisecondes, temps_ecoule ;

#endif

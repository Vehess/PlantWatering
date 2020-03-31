/**
 * Exemple de code pour la lecture d'un capteur DHT11 ou DHT22.
 * Code original issu du lien suivant, et modifi� pour mes besoins
 * https://www.carnetdumaker.net/articles/utiliser-un-capteur-de-temperature-et-dhumidite-dht11-dht22-avec-une-carte-arduino-genuino/
 * Merci � l'auteur original!
 */

#include "humidite.h"

/** Broche "DATA" du capteur */
const byte BROCHE_CAPTEUR = 5;

/* Code d'erreur de la fonction readDHT11() et readDHT22() */
const byte DHT_SUCCESS = 0;        // Pas d'erreur
const byte DHT_TIMEOUT_ERROR = 1;  // Temps d'attente d�pass�
const byte DHT_CHECKSUM_ERROR = 2; // Donn�es re�ues erron�es

/* Code d'erreur de la fonction readDHT11() et readDHT22() */
typedef enum{
	DHT_SUCCESS        = 0,  // Pas d'erreur
	DHT_TIMEOUT_ERROR  = 1,  // Temps d'attente d�pass�
	DHT_CHECKSUM_ERROR = 2   // Donn�es re�ues erron�es
}DHT_t;

/** Fonction setup() */
void setup() {

  /* Initialisation du port s�rie */
  Serial.begin(115200);
  Serial.println(F("Demo DHT11 et DHT22"));

  /* Place la broche du capteur en entr�e avec pull-up */
  pinMode(BROCHE_CAPTEUR, INPUT_PULLUP);
}

void humidite_engine()
{
	float temperature, humidity;

	switch (readDHT11(BROCHE_CAPTEUR, &temperature, &humidity))
	{
		case DHT_SUCCESS:
			/* Affichage de la temp�rature et du taux d'humidit� */
			//Penser a r�cuperer l'information ici.
			Serial.print(F("Humidite (%): "));
			Serial.println(humidity, 2);
			Serial.print(F("Temperature (^C): "));
			Serial.println(temperature, 2);
			break;

	  case DHT_TIMEOUT_ERROR:
			Serial.println(F("Pas de reponse !"));
			break;

	  case DHT_CHECKSUM_ERROR:
			Serial.println(F("Pb de communication !"));
			break;
	  }

	}
}

/** Fonction loop() */
void loop() {
  float temperature, humidity;

  /* Lecture de la temp�rature et de l'humidit�, avec gestion des erreurs */
  // N.B. Remplacer readDHT11 par readDHT22 en fonction du capteur utilis� !
  switch (readDHT11(BROCHE_CAPTEUR, &temperature, &humidity))
  {
  case DHT_SUCCESS:

    /* Affichage de la temp�rature et du taux d'humidit� */
    Serial.print(F("Humidite (%): "));
    Serial.println(humidity, 2);
    Serial.print(F("Temperature (^C): "));
    Serial.println(temperature, 2);
    break;

  case DHT_TIMEOUT_ERROR:
    Serial.println(F("Pas de reponse !"));
    break;

  case DHT_CHECKSUM_ERROR:
    Serial.println(F("Pb de communication !"));
    break;
  }

  /* Pas plus d'une mesure par seconde */
  // N.B. Avec le DHT22 il est possible de r�aliser deux mesures par seconde
  delay(1000);
}

/**
 * Lit la temp�rature et le taux d'humidit� mesur� par un capteur DHT11.
 *
 * @param pin Broche sur laquelle est c�bl�e le capteur.
 * @param temperature Pointeur vers la variable stockant la temp�rature.
 * @param humidity Pointeur vers la variable stockant le taux d'humidit�.
 * @return DHT_SUCCESS si aucune erreur, DHT_TIMEOUT_ERROR en cas de timeout, ou DHT_CHECKSUM_ERROR en cas d'erreur de checksum.
 */
DHT_t readDHT11(byte pin, float* temperature, float* humidity)
{

  /* Lit le capteur */
  byte data[5];
  byte ret = readDHTxx(pin, data, 18, 1000);

  /* D�tecte et retourne les erreurs de communication */
  if (ret != DHT_SUCCESS)
    return ret;

  /* Calcul la vraie valeur de la temp�rature et de l'humidit� */
  *humidity = data[0];
  *temperature = data[2];

  /* Ok */
  return DHT_SUCCESS;
}

/**
 * Lit la temp�rature et le taux d'humidit� mesur� par un capteur DHT22.
 *
 * @param pin Broche sur laquelle est c�bl�e le capteur.
 * @param temperature Pointeur vers la variable stockant la temp�rature.
 * @param humidity Pointeur vers la variable stockant le taux d'humidit�.
 * @return DHT_SUCCESS si aucune erreur, DHT_TIMEOUT_ERROR en cas de timeout, ou DHT_CHECKSUM_ERROR en cas d'erreur de checksum.
 */
DHT_t readDHT22(byte pin, float* temperature, float* humidity)
{
  /* Lit le capteur */
  byte data[5];
  byte ret = readDHTxx(pin, data, 1, 1000);

  /* D�tecte et retourne les erreurs de communication */
  if (ret != DHT_SUCCESS)
  {
	  return ret;
  }

  /* Calcul la vraie valeur de la temp�rature et de l'humidit� */
  float fh = data[0];
  fh *= 256;
  fh += data[1];
  fh *= 0.1;
  *humidity = fh;

  float ft = data[2] & 0x7f;
  ft *= 256;
  ft += data[3];
  ft *= 0.1;
  if (data[2] & 0x80)
  {
    ft *= -1;
  }
  *temperature = ft;

  /* Ok */
  return DHT_SUCCESS;
}

/**
 * Fonction bas niveau permettant de lire la temp�rature et le taux d'humidit� (en valeurs brutes) mesur� par un capteur DHTxx.
 */
DHT_t readDHTxx(byte pin, byte* data, unsigned long start_time, unsigned long timeout)
{
  data[0] = data[1] = data[2] = data[3] = data[4] = 0;
  // start_time est en millisecondes
  // timeout est en microsecondes

  /* Conversion du num�ro de broche Arduino en ports / masque binaire "bas niveau" */
  uint8_t bit = digitalPinToBitMask(pin);
  uint8_t port = digitalPinToPort(pin);
  volatile uint8_t *ddr = portModeRegister(port);   // Registre MODE (INPUT / OUTPUT)
  volatile uint8_t *out = portOutputRegister(port); // Registre OUT (�criture)
  volatile uint8_t *in = portInputRegister(port);   // Registre IN (lecture)

  /* Conversion du temps de timeout en nombre de cycles processeur */
  unsigned long max_cycles = microsecondsToClockCycles(timeout);

  /* Evite les probl�mes de pull-up */
  *out |= bit;  // PULLUP
  *ddr &= ~bit; // INPUT
  delay(100);   // Laisse le temps � la r�sistance de pullup de mettre la ligne de donn�es � HIGH

  /* R�veil du capteur */
  *ddr |= bit;  // OUTPUT
  *out &= ~bit; // LOW
  delay(start_time); // Temps d'attente � LOW causant le r�veil du capteur
  // N.B. Il est impossible d'utilise delayMicroseconds() ici car un d�lai
  // de plus de 16 millisecondes ne donne pas un timing assez pr�cis.

  /* Portion de code critique - pas d'interruptions possibles */
  noInterrupts();

  /* Passage en �coute */
  *out |= bit;  // PULLUP
  delayMicroseconds(40);
  *ddr &= ~bit; // INPUT

  /* Attente de la r�ponse du capteur */
  timeout = 0;
  while(!(*in & bit))
  { /* Attente d'un �tat LOW */
    if (++timeout == max_cycles)
    {
        interrupts();
        return DHT_TIMEOUT_ERROR;
    }
  }

  timeout = 0;
  while(*in & bit)
  { /* Attente d'un �tat HIGH */
    if (++timeout == max_cycles)
    {
        interrupts();
        return DHT_TIMEOUT_ERROR;
    }
  }

  /* Lecture des donn�es du capteur (40 bits) */
  for (byte i = 0; i < 40; ++i)
  {
    /* Attente d'un �tat LOW */
    unsigned long cycles_low = 0;
    while(!(*in & bit))
    {
      if (++cycles_low == max_cycles)
      {
        interrupts();
        return DHT_TIMEOUT_ERROR;
      }
    }

    /* Attente d'un �tat HIGH */
    unsigned long cycles_high = 0;
    while(*in & bit)
    {
      if (++cycles_high == max_cycles)
      {
        interrupts();
        return DHT_TIMEOUT_ERROR;
      }
    }

    /* Si le temps haut est sup�rieur au temps bas c'est un "1", sinon c'est un "0" */
    data[i / 8] <<= 1;
    if (cycles_high > cycles_low)
    {
      data[i / 8] |= 1;
    }
  }

  /* Fin de la portion de code critique */
  interrupts();

  /*
   * Format des donn�es :
   * [1, 0] = humidit� en %
   * [3, 2] = temp�rature en degr�s Celsius
   * [4] = checksum (humidit� + temp�rature)
   */

  /* V�rifie la checksum */
  byte checksum = (data[0] + data[1] + data[2] + data[3]) & 0xff;
  if (data[4] != checksum)
  {
	    return DHT_CHECKSUM_ERROR; /* Erreur de checksum */
  }
  else
  {
        return DHT_SUCCESS; /* Pas d'erreur */
  }
}

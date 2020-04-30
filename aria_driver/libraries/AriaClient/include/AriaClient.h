/**************************************************************************************************
 *
 *	@author    Damian Muszynski <d.muszynski@accrea.com>
 *	@brief     API for ARIA ARM
 *  @file	   AriaClient.h
 *	@copyright 2020 ACCREA ENGINEERING
 *
**************************************************************************************************/

#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

// Definicja struktur z danymi
typedef struct __attribute__((__packed__))
{
	uint8_t ControlMode_DEM;
	float JointsPosition_DEM[7];
	float JointsVelocity_DEM[7];
	float JointsTorque_DEM[7];
	float CartesianEEPosition_DEM[3];
	float CartesianEEQuaternion_DEM[4];
	float GripperPosition_DEM;
} ARM_DATA_MISO;

typedef struct __attribute__((__packed__))
{
	uint8_t ULA_Status;
	uint16_t ULA_FaultCode;
	uint8_t JointMode_ACT[7];
	uint16_t JointFaultCode_ACT[7];
	float JointPos_ACT[7];
	float JointVel_ACT[7];
	float JointTrq_ACT[7];
	float EEPos_ACT[3];
	float EEQuat_ACT[4];
	float GripperPos_ACT;
} ARM_DATA_MOSI;

/*
 * Funkcja inicjalizujaca gniazdo polaczenia oraz potrzebne zmienne
 * Jesli adres ip nie zostal skonfigurowany wczesniej, wykorzystane sa
 * wartosci domyslne 192.168.9.9:7777
 * adres moze zostac zmieniony takze po wykonaniu tej funkcji
 */
extern void AriaClient_Init();

/*
 * Funkcja probujaca nawiazac polaczenie z serwerem
 * Wartosci zwracane:
 *   0 - otrzymano poprawna odpowiedz od serwera
 *  -1 - brak odpowiedzi (serwer nie odnaleziony)
 *  -2 - otrzymano bledne dane
 */
extern int AriaClient_Connect();

/*
 * Funkcja inicjujaca usluge ciaglej wymiany danych z serwerem
 * Czestotliwosc wymiany danych to 100Hz
 * W przypadku utraty polaczenia usluga przez 2 sekundy probuje
 * odnowic polaczenie, a jesli sie nie uda konczy prace
 *
 * Wartosci zwracane
 *   0 prawidlowe uruchomienie uslugi
 *  -1 usluga juz zostala uruchomiona
 *  -2 blad tworzenia uslugi
 */
extern int AriaClient_StartCommunication();

/*
 * Funkcja zatrzymujaca usluge ciaglej wymiany danych z serwerem
 */
extern void AriaClient_StopCommunication();

/*
 * Funkcja zwraca informacje czy polaczenie jest aktywne (usluga wymiany danych aktywna)
 * Gdy ta funkcja zwraca 0 to dane w buforze nie sa aktualne
 */
extern uint8_t AriaClient_isConnected();

/*
 * Funkcja umozliwiajaca zmiane adresu serwera do ktorego probujemy sie polaczyc
 * - ipAddr jest to string w formacie "XXX.XXX.XXX.XXX" np. "192.168.1.51"
 * - port jest to liczba bez znaku np. 7777
 */
extern void AriaClient_SetAddress(const char* ipAddr, uint16_t port);



/******** Funkcje sluzace do odczytu danych z bufora komunikacji ***********/


/*
 * Funkcje zarzadzajace praca ramienia
 */

// Funkcja zwracajaca aktualny tryb pracy ramienia
extern uint8_t AriaClient_GetArmStatus();

/* Funkcja zwracajaca aktualny kod bledu ramienia
 * nalezy go interpretowac jako numer w formacie hexadecymalnym */
extern uint16_t AriaClient_GetArmFaultCode();

/* Funkcja powodujaca zmiane trybu pracy ramienia
 * przed zmiana trybu aktualne wartosci sa przepisywane jako zadane
 * w celu ochrony przed niechcianymi przemieszczeniami */
extern void AriaClient_SetArmControlMode(uint8_t controlMode);


/*
 * Funkcje w przestrzeni kartezjanskiej - dla koncowki roboczej
 */

/* Funkcja zwracajaca aktualna pozycje koncowki roboczej
 * Argument musi zawierac wskaznik na tablice typu float o 3 elementach */
extern void AriaClient_GetArmEEPosition(float *EEPos);

/* Funkcja zwracajaca aktualna orientacje koncowki roboczej
 * Argument musi zawierac wskaznik na tablice typu float o 4 elementach */
extern void AriaClient_GetArmEEQuaternion(float *EEQuat);

// Funkcja zwracajaca aktualna pozycje elementu wykonawczego koncowki roboczej
extern float AriaClient_GetArmGripperPos();

/* Funkcja powodujaca zmiane wartosci zadanej pozycji koncowki roboczej
 * Argument musi zawierac wskaznik na tablice typu float o 3 elementach */
extern void AriaClient_SetArmEEPosition(float *EEPos);

/* Funkcja powodujaca zmiane wartosci zadanej orientacji koncowki roboczej
 * Argument musi zawierac wskaznik na tablice typu float o 4 elementach */
extern void AriaClient_SetArmEEQuaternion(float *EEQuat);

// Funkcja powodujaca zmiane wartosci zadanej elementu wykonawczego koncowki roboczej
extern void AriaClient_SetArmGripperPos(float position);


/*
 * Funkcje w przestrzeni jointowej - dane z modulow
 */

// Funkcja zwracajaca aktualny tryb pracy wybranego modulu
extern uint8_t AriaClient_GetJointMode(uint8_t jointNr);

// Funkcja zwracajaca aktualny kod bledu wybranego modulu
extern uint16_t AriaClient_GetJointFaultCode(uint8_t jointNr);

// Funkcja zwracajaca aktualna pozycje wybranego modulu
extern float AriaClient_GetJointPos(uint8_t jointNr);

// Funkcja zwracajaca aktualna predkosc wybranego modulu
extern float AriaClient_GetJointVel(uint8_t jointNr);

// Funkcja zwracajaca aktualny moment generowny przez wybrany modul
extern float AriaClient_GetJointTrq(uint8_t jointNr);

// Funkcja powodujaca zmiane wartosci zadanej pozycji wybranego modulu
extern void AriaClient_SetJointPos(uint8_t jointNr, float position);

// Funkcja powodujaca zmiane wartosci zadanej predkosci wybranego modulu
extern void AriaClient_SetJointVel(uint8_t jointNr, float velocity);

// Funkcja powodujaca zmiane wartosci zadanej generowanego momentu wybranego modulu
extern void AriaClient_SetJointTrq(uint8_t jointNr, float torque);


/*
 * Funkcja zwracajaca strukture danych typu ARM_DATA_MOSI zawierajaca
 * wszystkie dane otrzymane od serwera
 */
extern ARM_DATA_MOSI AriaClient_GetArmAllData();

/*
 * Funkcja umozliwiajaca wyslanie struktury danych typu ARM_DATA_MISO
 * zawierajacej wszystkie dane ktore mozna wyslac do serwera
 * UWAGA! Uzywac tylko wtedy kiedy jest sie pewnym wszystkich danych w strukturze
 */
extern void AriaClient_SetArmAllData(ARM_DATA_MISO userArmData);


#ifdef __cplusplus
}
#endif


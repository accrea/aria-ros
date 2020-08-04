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

/**** Definicja struktur z danymi ***/
// Po zadeklarowaniu struktur w kodzie nalezy je wypelnic zerami przed ich uzyciem!!!

/* Struktura danych wysylanych do mastera ramienia
 * Zadane parametry pracy ramienia */
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

/* Struktura danych odbieranych od mastera ramienia
 * Aktualne parametry pracy ramienia */
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
 * Po poprawnym uruchomieniu ciaglej komunikacji nastepuje przepisanie
 * aktualnych wartosci pozycji na wartosci zadane aby nie wystapily
 * niepozadane ruchy
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
 */
extern void AriaClient_SetArmControlMode(uint8_t controlMode);

/* Funkcja oczekujaca na osiagniecie trybu pracy ramienia.
 * Funkcja posiada timeout wynoszacy 2s.
 * Uwaga! Ta funkcja nie zastepuje funkcji SetArmControlMode
 * tzn nie zmienia wartosci zadanej.
 *
 * zwracane wartosci:
 *  0 gdy ramie osiagnelo wymagany stan
 * -1 gdy ramie nie osiagnelo wymaganego stanu */
extern int AriaClient_WaitForMode(uint8_t controlMode);


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

/* Funkcja zwracajaca aktualny tryb pracy wybranego modulu
 * jako argument podajemy numer modulu (1-7)
 */
extern uint8_t AriaClient_GetJointMode(uint8_t jointNr);

/* Funkcja zwracajaca aktualny kod bledu wybranego modulu
 * jako argument podajemy numer modulu (1-7)
 */
extern uint16_t AriaClient_GetJointFaultCode(uint8_t jointNr);

// Funkcja zwracajaca aktualna pozycje wybranego modulu
extern float AriaClient_GetJointPos(uint8_t jointNr);

/* Funkcja zwracajaca aktualna predkosc wybranego modulu
 * jako argument podajemy numer modulu (1-7)
 */
extern float AriaClient_GetJointVel(uint8_t jointNr);

/* Funkcja zwracajaca aktualny moment generowny przez wybrany modul
 * jako argument podajemy numer modulu (1-7)
 */
extern float AriaClient_GetJointTrq(uint8_t jointNr);

/* Funkcja powodujaca zmiane wartosci zadanej pozycji wybranego modulu
 * jako argumenty podajemy numer modulu (1-7) oraz wartosc jako float
 */
extern void AriaClient_SetJointPos(uint8_t jointNr, float position);

/* Funkcja umozliwiajaca zmiane wartosci zadanej pozycji dla kilku modulow
 * jako argumenty podajemy: liczbe modulow 1-7, wskaznik na tablice z nowymi wartosciami
 */
extern void AriaClient_SetJointsPos(uint8_t jointsCnt, float* position);

/* Funkcja powodujaca zmiane wartosci zadanej predkosci wybranego modulu
 * jako argumenty podajemy numer modulu (1-7) oraz wartosc jako float
 */
extern void AriaClient_SetJointVel(uint8_t jointNr, float velocity);

/* Funkcja umozliwiajaca zmiane wartosci zadanej predkosci dla kilku modulow
 * jako argumenty podajemy: liczbe modulow 1-7, wskaznik na tablice z nowymi wartosciami
 */
extern void AriaClient_SetJointsVel(uint8_t jointsCnt, float* velocity);

/* Funkcja powodujaca zmiane wartosci zadanej generowanego momentu wybranego modulu
 * jako argumenty podajemy numer modulu (1-7) oraz wartosc jako float
 */
extern void AriaClient_SetJointTrq(uint8_t jointNr, float torque);

/* Funkcja umozliwiajaca zmiane wartosci zadanej generowanego momentu dla kilku modulow
 * jako argumenty podajemy: liczbe modulow 1-7, wskaznik na tablice z nowymi wartosciami
 */
extern void AriaClient_SetJointsTrq(uint8_t jointsCnt, float* torque);


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


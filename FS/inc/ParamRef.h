/*****************************************************************************************************************
* Copyright by BSH Bosch und Siemens Hausgeraete GmbH - EDS
*
*-----------------------------------------------------------------------------------------------------------------
*
* Die Nutzung, die Vervielfaeltigung und die Weitergabe dieser Software oder ihrer Teile ist nur mit
* ausdruecklicher schriftlicher Zustimmung der BSH EDS gestattet. Die Software darf vom Verwender
* ausschliesslich fuer die Entwicklung von Elektroniken fuer BSH genutzt werden, eine Nutzung fuer
* sonstige Zwecke (z.B. Entwicklung von Elektroniken fuer Dritte) ist untersagt. Saemtliche Rechte,
* einschliesslich aller Urheber-, Patent- und Gebrauchsmusterrechte, an der Software bleiben BSH
* vorbehalten. Zuwiderhandlungen werden straf- und zivilrechtlich verfolgt.
*
* Die Software wurde nach dem Stand der Technik erstellt und geprueft. Aufgrund der Komplexitaet der
* Implementierung von Embedded Controller Software sind  vom Verwender fuer jede Implementierung eigene
* Tests durchzufuehren, um die Funktionssicherheit in seiner Entwicklung sicherzustellen. Die Software
* wurde fuer den Einsatz als Bibliothek entwickelt. Jede anderweitige Verwendung erfolgt in alleiniger
* Verantwortung des Nutzers. Der Nutzer ist berechtigt, die Software zur Anpassung an andere
* Microcontroller und andere Compiler in eigener Verantwortung zu veraendern  und anzupassen. 
* BSH leistet keinen Support fuer derartige Anpassungen, ist jedoch
* berechtigt, von Verwender vorgenommene Anpassungen fuer eigene Zwecke kostenfrei zu nutzen.
*
* Die Haftung der BSH fuer die Funktionsfaehigkeit und -sicherheit der Software, auch beim  Einsatz in
* einer konkreten Anwendung, und moegliche, durch sie verursachte Folgeschaeden ist, ausser im Falle von
* Vorsatz oder grober Fahrlaessigkeit, ausgeschlossen. In jedem Fall ist die Haftung der BSH jedoch auf
* den typischerweise auftretenden, vorhersehbaren Schaden begrenzt.
*
* Technische Aenderungen sind vorbehalten.
*
*-----------------------------------------------------------------------------------------------------------------
*
* Use, reproduction and dissemination of this software or parts of it, is not permitted without express
* written authority of BSH EDS. The user is allowed to use this software exclusively for the development
* of electronic boards for BSH. Usage for other purposes is strictly prohibited (e.g. development of
* electronic boards for third parties). All rights, including copyright, rights created by patent grant
* or registration, and rights by protection of utility patents, are reserved. Violations will be
* prosecuted by civil and criminal law.
*
* The software was created and approved by acknowledged rules of technology. Because of the complexity
* of embedded controller software the user of this software has to perform his own tests to ensure
* proper functionality in his environment. The software was developed for usage as a software library.
* The user is sole responsible for every other usage.  The user may modify the software for adaptations
* needed for other microcontrollers, as well as to another compilers Metrowerks at own risk. 
* BSH will not support any adaptations or changes. BSH is authorised to use all adaptations for 
* own purposes free of charge.
*
* BSH assumes no liability for the functionality or reliability of the software even in concrete
* applications. BSH assumes no liability for consequential damages, except in case of intention or gross
* negligence. In any case the liability is limited to the typical and predictable damage.
*
* Technical changes are reserved.
*
******************************************************************************************************************
 *   PROJECT          GV640 III Generation
 *   MODULE-PREFIX    REFPAR
 *   FILE             %PM%
 *   ARCHIVE          %PP%:%PI%
 *   PROCESSOR        STM32
 *   COMPILER         KEIL
 *****************************************************************************************************************
 *   AUTHOR           Ralf Hochhausen
 *   CREATED          20.05.2011
 * 
 *   LAST CHANGE      %PRT%
 *   LAST REVISION    %PR%
 *   STATUS	      %PS%
 *****************************************************************************************************************
 *   PURPOSE */
/**     @file ParamRef.h Reference value parameters
 *      
 */
/*****************************************************************************************************************
 *   CHANGES
 * %PL%
 * 
 ****MHE**********************************************************************************************************/

#ifndef __REFPAR_H
#define __REFPAR_H

/**
 * \brief	Definition of the motor reference speed in [rpm]
 * 			All per unit speed values are referenced to this value. The value
 * 			is used to convert speed values from real to per unit numbers and back.
 * */
#define REFPAR_REFERENCE_SPEED_RPM			(8192.0)

/**
 * \brief	Definition of the reference current in [mA]
 *
 *			All per unit current values are referenced to this value. The
 *			value is used to convert current values from real to per unit
 *			numbers and vice versa.
 *			The value is equal to the max. measurable current value under
 *			ideal conditions (optimal values of the current amplifier).
 * */
#define REFPAR_REFERENCE_CURRENT_MA_C			(1250.0)  	//old PH_PCG value, used in current.c Imax
#define REFPAR_REFERENCE_CURRENT_MA			    (3500.0)	
/**
 * \brief	Definition of the reference DC link voltage in [V]
 * */
#define REFPAR_REFERENCE_DCL_VOLTAGE_V		(PAR_DCL_VOLT_MAX_V)
#define REFPAR_REFERENCE_15_VOLTAGE_MV	    (PAR_15_VOLT_MAX_MV)
#define REFPAR_REFERENCE_VREFINT_VOLTAGE_MV (PAR_ADC_REF_VOLTAGE * 1000)
/**
 * \brief	Definition of the reference line voltage in [V]
 * */
#define REFPAR_REFERENCE_LINE_VOLTAGE_V		(PAR_DCL_VOLT_MAX_V)

/**
 * \brief	Definition of the reference resistance in [Ohm]
 *
 * 			Measured resistances are referenced to this value.
 * */
#define REFPAR_REFERENCE_RESISTANCE_OHM		(U_MAX/I_MAX)

#endif /* __REFPAR_H */

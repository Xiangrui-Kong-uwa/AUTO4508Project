#ifndef PHIDGET_HELPER_FUNCTIONS_H
#define PHIDGET_HELPER_FUNCTIONS_H

#include <stdio.h>
#include <stdlib.h>
#include <phidget22.h>

/* Determine if we are on Windows of Unix */
#ifdef WIN32
#include <windows.h>
#else
#include <unistd.h>
#define Sleep(x) usleep((x)*1000)
#endif

typedef struct {
	int isRemote;
	int serverDiscovery;
	char hostname[100];
	int port;
	char password[100];
} NetInfo;

typedef struct {
	int deviceSerialNumber;
	int hubPort;
	int channel;
	int isHubPortDevice;
	int isVINT;
	NetInfo netInfo;
} ChannelInfo;

/**
* Reads user input and sets val to 1 if input Y or y, sets val to 0 if input N or n
* If def is not -1, and input is empty, sets val to def.
*
* @param val pointer to integer that will get set depending on the user input
* @return 0 if the operation succeeds, 1 if it fails
*/
int ProcessYesNo_Input(int* val, int def);

/**
* Displays the error string associated to the PhidgetRetrunCode from a phidget function call
* along with a message to help track it down
*
* @param returnCode The PhidgetReturnCode to get the error string of
* @param message A message to indicate where the error code originated
*/
void DisplayError(PhidgetReturnCode returnCode, char* message);

/**
* Checks the Phidget Return code for an error and  displays the associated error string if an error is detected
* This function will exit the program after displaying its message if an error has been detected.
* This function is intended for rudimentary error handling and cleanup in the basic case of only using one Phidget in the program.
* More complex programs should use error handling customized to the application.
*
* @param returnCode The PhidgetReturnCode to check
* @param message A message to indicate where the error code originated
* @param chptr a pointer to the Phidget channel that will be deleted as part of cleanup if an error is detected
*/
void CheckError(PhidgetReturnCode returnCode, char* message, PhidgetHandle *ch);

/**
* Initializes the ChannelInfo structure
*
* @param channelInfoPtr a pointer to the channelInfo struct to fill
*/
void InitChannelInfo(ChannelInfo *channelInfoPtr);

/**
* Inputs the DeviceSerialNumber of the Phidget channel
*
* @param channelInfoPtr a pointer to the channelInfo struct to fill
*/
void InputSerialNumber(ChannelInfo *channelInfoPtr);

/**
* Inputs the isHubPortDevice property of the Phidget channel
*
* @param channelInfoPtr a pointer to the channelInfo struct to fill
*/
void InputIsHubPortDevice(ChannelInfo *channelInfoPtr);

/**
* Inputs the HubPort and isHubportDevice of the Phidget channel if it is a VINT device
*
* @param channelInfoPtr a pointer to the channelInfo struct to fill
* @param chptr a pointer to the Phidget channel to set
*/
void InputVINTProperties(ChannelInfo *channelInfoPtr, PhidgetHandle *chptr);

/**
* Inputs the Channel value of the Phidget channel
*
* @param channelInfoPtr a pointer to the channelInfo struct to fill
*/
void InputChannel(ChannelInfo *channelInfoPtr);

/**
* Inputs IsRemote for the Phidget Channel.
* If isRemote is true, ask the user for network info. Using either server discovery or manual server info
*
* @param channelInfoPtr a pointer to the channelInfo struct to fill
*/
void SetupNetwork(ChannelInfo *channelInfoPtr);

/**
* Asks for all addressing parameters for the Phidget Channel.
*
* @param channelInfoPtr a pointer to the channelInfo struct to fill
* @param chptr a pointer to the Phidget channel to set
*/
void AskForDeviceParameters(ChannelInfo *channelInfoPtr, PhidgetHandle *chptr);

/**
* Prints information about the channel such as serial number, channel, and hubPort, where applicable
*
* @return 0 if the operation succeeds, or the PhidgetReturnCode of the function that failed
*/
PhidgetReturnCode PrintChannelInformaiton(PhidgetHandle ph);

/**
* Checks is Open failed, and prints additional information about how Open may have failed if it has.
* This function will exit the program after displaying its message if an error has been detected.
*
* @param e the PhidgetReturnCode
* @param ph The Phidget channel to set
* @param chptr a pointer to the Phidget channel that will be deleted as part of cleanup if an error is detected
*/
void CheckOpenError(PhidgetReturnCode e, PhidgetHandle *ch);

/**
* Checks if EnableServerDiscovery has failed, and prints additional information about how EnableServerDiscovery may have failed if it has
* This function will exit the program after displaying its message if an error has been detected.
*
* @param e the PhidgetReturnCode
* @param chptr a pointer to the Phidget channel that will be deleted as part of cleanup if an error is detected
*/
void CheckEnableServerDiscoveryError(PhidgetReturnCode e, PhidgetHandle *ch);

/**
* This function will delete the channel pointed to by chptr, display an error message, and wait for user input before exiting
* Calling this function will exit the program with code 1
*
* @param chptr a pointer to the Phidget channel that will be deleted as part of cleanup
*/
void ExitWithErrors(PhidgetHandle *ch);

#endif

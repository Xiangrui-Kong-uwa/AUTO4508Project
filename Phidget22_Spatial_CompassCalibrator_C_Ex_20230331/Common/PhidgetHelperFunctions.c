#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <phidget22.h>
#include "PhidgetHelperFunctions.h"

int ProcessYesNo_Input(int* val, int def) {
	char strvar[100];

	if (fgets(strvar, 100, stdin) == NULL)
		return 1;

	if (strvar[0] == '\n') {
		if (def == -1)
			return 1;
		*val = def;
		return 0;
	}

	if (strvar[0] == 'N' || strvar[0] == 'n') {
		*val = 0;
		return 0;
	}

	if (strvar[0] == 'Y' || strvar[0] == 'y') {
		*val = 1;
		return 0;
	}

	return 1;
}

void DisplayLocatePhidgetsLink() {
	printf("\n  | In the following example, you will be asked to provide information that specifies which Phidget the program will use. "
		"\n  | If you are unsure of any of these parameters, be sure to check www.phidgets.com/docs/Finding_The_Addressing_Information "
		"\n  | Press ENTER once you have read this message.\n");
	getchar();

	printf("\n--------------------\n");
}

void DisplayError(PhidgetReturnCode returnCode, char* message) {
	PhidgetReturnCode prc; //Used to catch error codes from each Phidget function call
	const char* error;

	fprintf(stderr, "Runtime Error -> %s\n\t", message);

	prc = Phidget_getErrorDescription(returnCode, &error);
	if (prc != EPHIDGET_OK) {
		DisplayError(prc, "Getting ErrorDescription");
		return;
	}

	fprintf(stderr, "Desc: %s\n", error);

	if (returnCode == EPHIDGET_WRONGDEVICE) {
		fprintf(stderr, "\tThis error commonly occurs when the Phidget function you are calling does not match the class of the channel that called it.\n"
		                "\tFor example, you would get this error if you called a PhidgetVoltageInput_* function with a PhidgetDigitalOutput channel.");
	}
	else if (returnCode == EPHIDGET_NOTATTACHED) {
		fprintf(stderr, "\tThis error occurs when you call Phidget functions before a Phidget channel has been opened and attached.\n"
		                "\tTo prevent this error, ensure you are calling the function after the Phidget has been opened and the program has verified it is attached.\n");
	}
	else if (returnCode == EPHIDGET_NOTCONFIGURED) {
		fprintf(stderr, "\tThis error code commonly occurs when you call an Enable-type function before all Must-Set Parameters have been set for the channel.\n"
		                "\tCheck the API page for your device to see which parameters are labled \"Must be Set\" on the right-hand side of the list.");
	}
}

void ExitWithErrors(PhidgetHandle *chptr) {
	Phidget_delete(chptr);
	fprintf(stderr, "\nExiting with error(s)...\n");
	printf("Press ENTER to end program.\n");
	getchar();
	exit(1);
}

void CheckError(PhidgetReturnCode returnCode, char* message, PhidgetHandle *chptr) {
	
	if (returnCode != EPHIDGET_OK) {
		DisplayError(returnCode, message);
		ExitWithErrors(chptr);
	}
}

void InitChannelInfo(ChannelInfo *channelInfoPtr) {
	channelInfoPtr->deviceSerialNumber = -1;
	channelInfoPtr->hubPort = -1;
	channelInfoPtr->channel = -1;
	channelInfoPtr->isHubPortDevice = 0;
	channelInfoPtr->isVINT = 0;

	channelInfoPtr->netInfo.isRemote = 0;
	channelInfoPtr->netInfo.serverDiscovery = 0;
	channelInfoPtr->netInfo.hostname[0] = '\0';
	channelInfoPtr->netInfo.port = 0;
	channelInfoPtr->netInfo.password[0] = '\0';
}



void InputSerialNumber(ChannelInfo *channelInfoPtr) {
	int deviceSerialNumber = 0;
	char strvar[100];

	printf("\nFor all questions, enter the value, or press ENTER to select the [Default]\n");

	printf("\n--------------------------------------\n");
	printf("\n  | Some Phidgets have a unique serial number, printed on a white label on the device.\n"
	  "  | For Phidgets and other devices plugged into a VINT Port, use the serial number of the VINT Hub.\n"
	  "  | Specify the serial number to ensure you are only opening channels from that specific device.\n"
	  "  | Otherwise, use -1 to open a channel on any device.\n");
	while (1) {
		printf("\nWhat is the Serial Number? [-1] ");
		if (fgets(strvar, 100, stdin) == NULL)
			continue;
		if (strvar[0] == '\n') {
			deviceSerialNumber = PHIDGET_SERIALNUMBER_ANY;
			break;
		}
		deviceSerialNumber = atoi(strvar);
		if (deviceSerialNumber >= -1 && deviceSerialNumber != 0)
			break;
	}

	channelInfoPtr->deviceSerialNumber = deviceSerialNumber;
	
	return;
}

void InputIsHubPortDevice(ChannelInfo *channelInfoPtr) {
	int isHubPortDevice = 0;

	while (1) {
		printf("\nIs this a \"HubPortDevice\"? [y/n] ");
		if (!ProcessYesNo_Input(&isHubPortDevice, -1))
			break;
	}

	channelInfoPtr->isHubPortDevice = isHubPortDevice;
}

void InputVINTProperties(ChannelInfo *channelInfoPtr, PhidgetHandle *chptr) {
	int canBeHubPortDevice = 0;
	Phidget_ChannelClass pcc;
	PhidgetReturnCode prc; //Used to catch error codes from each Phidget function call
	char strvar[100];
	int hubPort = -1;
	int isVINT = 0;

	printf("\n--------------------------------------\n");

	while (1) {
		printf("\nDo you want to specify the hub port that your device is plugged into?\n"
			"Choose No if your device is not plugged into a VINT Hub. (y/n) ");
		if (!ProcessYesNo_Input(&isVINT, -1))
			break;
	}

	channelInfoPtr->isVINT = isVINT;

	//Don't ask about the HubPort and the HubPortDevice if it's not a VINT device
	if (!isVINT)
		return;

	printf("\n--------------------------------------\n");
	printf("\n  | VINT Hubs have numbered ports that can be uniquely addressed.\n"
		"  | The HubPort# is identified by the number above the port it is plugged into.\n"
		"  | Specify the hub port to ensure you are only opening channels from that specific port.\n"
		"  | Otherwise, use -1 to open a channel on any port.\n");
	while (1) {
		printf("\nWhat HubPort is the device plugged into? [-1] ");
		if (fgets(strvar, 100, stdin) == NULL)
			continue;
		if (strvar[0] == '\n') {
			hubPort = PHIDGET_HUBPORT_ANY;
			break;
		}
		hubPort = atoi(strvar);
		if (hubPort >= -1 && hubPort <= 5)
			break;
	}

	channelInfoPtr->hubPort = hubPort;

	prc = Phidget_getChannelClass(*chptr, &pcc);
	CheckError(prc, "Getting ChannelClass", chptr);

	switch (pcc) {
	case PHIDCHCLASS_VOLTAGEINPUT:
		printf("\n--------------------------------------\n");
		printf("\n  | A VoltageInput HubPortDevice uses the VINT Hub's internal channel to measure the voltage on the white wire.\n"
		  "  | If the device you are trying to interface returns an analog voltage between 0V-5V, open it as a HubPortDevice.\n");
		canBeHubPortDevice = 1;
		break;
	case PHIDCHCLASS_VOLTAGERATIOINPUT:
		printf("\n--------------------------------------\n");
		printf("\n  | A VoltageRatioInput HubPortDevice uses the VINT Hub's internal channel to measure the voltage ratio on the white wire.\n"
		  "  | If the device you are trying to interface returns an ratiometric voltage between 0V-5V, open it as a HubPortDevice.\n");
		canBeHubPortDevice = 1;
		break;
	case PHIDCHCLASS_DIGITALINPUT:
		printf("\n--------------------------------------\n");
		printf("\n  | A DigitalInput HubPortDevice uses the VINT Hub's internal channel to detect digital changes on the white wire.\n"
		  "  | If the device you are trying to interface returns a 5V or 3.3V digital signal, open it as a HubPortDevice.\n");
		canBeHubPortDevice = 1;
		break;
	case PHIDCHCLASS_DIGITALOUTPUT:
		printf("\n--------------------------------------\n");
		printf("\n  | A DigitalOutput HubPortDevice uses the VINT Hub's internal channel to output a 3.3V digital signal on the white wire.\n"
		  "  | If the device you are trying to interface accepts a 3.3V digital signal, open it as a HubPortDevice.\n");
		canBeHubPortDevice = 1;
		break;
	default:
		break;
	}

	if (canBeHubPortDevice)
		InputIsHubPortDevice(channelInfoPtr);

	return;
}

void InputChannel(ChannelInfo *channelInfoPtr) {
	char strvar[100];
	int Channel = 0;

	// Hub port devices only have a single channel, so don't ask for the channel
	if (channelInfoPtr->isHubPortDevice)
		return;

	printf("\n--------------------------------------\n");
	printf("\n  | Devices with multiple inputs or outputs of the same type will map them to channels.\n"
	  "  | The API tab for the device on www.phidgets.com shows the channel breakdown.\n"
	  "  | For example, a device with 4 DigitalInputs would use channels [0 - 3]\n"
	  "  | A device with 1 VoltageInput would use channel 0\n");
	while (1) {
		printf("\nWhat channel# of the device do you want to open? [0] ");
		if (fgets(strvar, 100, stdin) == NULL)
			continue;
		if (strvar[0] == '\n') {
			Channel = 0;
			break;
		}
		Channel = atoi(strvar);
		if (Channel >= 0)
			break;
	}

	channelInfoPtr->channel = Channel;

	return;
}

void SetupNetwork(ChannelInfo *channelInfoPtr) {
	char strvar[100];
	int discovery = 0;
	int isRemote = 0;
	char *pos;
	int port;

	printf("\n--------------------------------------\n");
	printf("\n  | Devices can either be opened directly, or over the network.\n"
	  "  | In order to open over the network, the target system must be running a Phidget Server.\n");
	while (1) {
		printf("\nIs this device being opened over the network? [y/N] ");
		if (!ProcessYesNo_Input(&isRemote, 0))
			break;
	}

	channelInfoPtr->netInfo.isRemote = isRemote;

	// if it's not remote, don't need to ask about the network
	if (!isRemote)
		return;

	printf("\n--------------------------------------\n");
	printf("\n  | Server discovery enables the dynamic discovery of Phidget servers that publish their identity to the network.\n"
		  "  | This allows you to open devices over the network without specifying the hostname and port of the server.\n");
	while (1) {
		printf("\nDo you want to enable server discovery? [Y/n] ");
		if (!ProcessYesNo_Input(&discovery, 1))
			break;
	}

	channelInfoPtr->netInfo.serverDiscovery = discovery;

	if (discovery)
		return;

	printf("\n--------------------------------------\n");
	printf("\nPlease provide the following information in order to open the device\n");

	while (1) {
		printf("\nWhat is the Hostname (or IP Address) of the server? [localhost] ");
		if (fgets(channelInfoPtr->netInfo.hostname, 100, stdin) == NULL)
			continue;
		if (channelInfoPtr->netInfo.hostname[0] == '\n') {
			snprintf(channelInfoPtr->netInfo.hostname, 100, "localhost");
			break;
		}
		// Remove trailing newline
		if ((pos=strchr(channelInfoPtr->netInfo.hostname, '\n')) != NULL) {
			*pos = '\0';
			break;
		}
	}

	printf("\n--------------------------------------\n");
	while (1) {
		printf("\nWhat port is the server on? [5661] ");
		if (fgets(strvar, 100, stdin) == NULL)
			continue;
		if (strvar[0] == '\n') {
			port = 5661;
			break;
		}
		port = atoi(strvar);
		if (port <= 65535 && port > 0)
			break;
	}

	channelInfoPtr->netInfo.port = port;

	printf("\n--------------------------------------\n");
	while (1) {
		printf("\nWhat is the password of the server? [] ");
		if (fgets(channelInfoPtr->netInfo.password, 100, stdin) == NULL)
			continue;
		// Remove trailing newline
		if ((pos=strchr(channelInfoPtr->netInfo.password, '\n')) != NULL) {
			*pos = '\0';
			break;
		}
	}
	printf("\n--------------------------------------\n");

	return;
}

void CheckOpenError(PhidgetReturnCode e, PhidgetHandle *chptr) {
	PhidgetReturnCode prc; //Used to catch error codes from each Phidget function call
	Phidget_ChannelClass channelClass;
	int isRemote;

	if (e == EPHIDGET_OK)
		return;

	DisplayError(e, "Opening Phidget Channel");
	if (e == EPHIDGET_TIMEOUT) {
		fprintf(stderr, "\nThis error commonly occurs if your device is not connected as specified, "
			"or if another program is using the device, such as the Phidget Control Panel.\n\n"
			"If your Phidget has a plug or terminal block for external power, ensure it is plugged in and powered.\n");
		
		prc = Phidget_getChannelClass(*chptr, &channelClass);
		CheckError(prc, "Getting ChannelClass", chptr);

		if (channelClass != PHIDCHCLASS_VOLTAGEINPUT
			&& channelClass != PHIDCHCLASS_VOLTAGERATIOINPUT
			&& channelClass != PHIDCHCLASS_DIGITALINPUT
			&& channelClass != PHIDCHCLASS_DIGITALOUTPUT) {
			fprintf(stderr, "\nIf you are trying to connect to an analog sensor, you will need to use the "
				"corresponding VoltageInput or VoltageRatioInput API with the appropriate SensorType.\n");
		}

		prc = Phidget_getIsRemote(*chptr, &isRemote);
		CheckError(prc, "Getting IsRemote", chptr);

		if (isRemote)
			fprintf(stderr, "\nEnsure the Phidget Network Server is enabled on the machine the Phidget is plugged into.\n");
	}

	ExitWithErrors(chptr);
}

void CheckEnableServerDiscoveryError(PhidgetReturnCode e, PhidgetHandle *chptr) {

	if (e == EPHIDGET_OK)
		return;

	DisplayError(e, "Enable Server Discovery");
	if (e == EPHIDGET_UNSUPPORTED) {
		fprintf(stderr, "\nThis error commonly occurs if your computer does not have the required mDNS support. "
			"We recommend using Bonjour Print Services on Windows and Mac, or Avahi on Linux.\n");
	}

	ExitWithErrors(chptr);
}

void AskForDeviceParameters(ChannelInfo *channelInfoPtr, PhidgetHandle *chptr) {
	DisplayLocatePhidgetsLink();
	InitChannelInfo(channelInfoPtr);
	InputSerialNumber(channelInfoPtr);
	InputVINTProperties(channelInfoPtr, chptr);
	InputChannel(channelInfoPtr);
	SetupNetwork(channelInfoPtr);
}
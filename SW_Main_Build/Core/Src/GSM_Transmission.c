/*
 * GSM_Transmission.c
 *
 *  Created on: May 12, 2023
 *      Author: K. David
 */
#include "GSM_Transmission.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
#include "main.h"
#include "Rte.h"
#include "Std_Types.h"



extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;
static uint8_t UART_Init_Done;
static uint8_t GSM_isConnected;
static char UART_Rx_Buffer[200] = {0};
static char UART_GPS_Buffer[750] = {0};
static uint8_t Command_Character;
static uint8_t last_Command_Character;
static uint8_t Command_Buffer[3] = {0};
static uint8_t cyclic_read_counter;
static uint8_t cyclic_write_counter;
static char buffer[150];
char CTRLZ[] = {26};
static uint8_t Write_Done = 0u;
static uint16_t write_counter = 0u;
static char datapacket[100];
static float latitude,longitude,speed_in_knots,latitudeFinal,longitudeFinal;
static uint8_t speed_kmh;
static uint8_t tx_data[750];
static uint8_t GPS_Payload[100];
static uint8_t Msgindex;
static char* ptr;

char AT_Init_Commands[AT_INIT_COMMANDS_NUM][AT_INIT_COMMANDS_MAX_LENGTH] =
{
							"AT\r\n",

							"AT+CPIN?\r\n",

							"AT+CREG?\r\n",

							"AT+CGATT?\r\n",

							"AT+CIPSHUT\r\n",

							"AT+CIPSTATUS\r\n",

							"AT+CIPMUX=0\r\n",

							"AT+CIICR\r\n",

							"AT+CIFSR\r\n",

		/* check without this command */					"AT+CIPSPRT=0\r\n",

							"AT+SAPBR=3,1,\"Contype\",\"GPRS\"\r\n",

							"AT+SAPBR=3,1,\"APN\",\"your_apn_here\"\r\n",

							"AT+SAPBR=1,1\r\n",

							"AT+HTTPINIT\r\n",

							"AT+HTTPPARA=\"CID\",1\r\n"
};

char AT_Cyclic_Commands_Read[AT_CYCLIC_COMMANDS_READ_NUM][AT_CYCLIC_COMMANDS_MAX_LENGTH] =
{

							"GET https://api.thingspeak.com/channels/2058851/fields/2/last.json?api_key=C82RYZFJ4KUUQ3Y1\r\n",

							"AT+HTTPPARA=\"URL\",\"http://api.thingspeak.com/channels/2058851/fields/2/last?api_key=C82RYZFJ4KUUQ3Y1\"\r\n",

							"AT+HTTPACTION=0\r\n",

							"AT+HTTPREAD=0,10\r\n"
};

char AT_Cyclic_Commands_Write[AT_CYCLIC_COMMANDS_WRITE_NUM][AT_CYCLIC_COMMANDS_MAX_LENGTH] =
{
							"AT+CIPSTART=\"TCP\",\"api.thingspeak.com\",\"80\"\r\n",

							"AT+CIPSEND\r\n",

};

uint8_t Find_Command_Character(char* buffer, uint16_t buffer_size) {
  uint16_t i, j;
  char number[4] = {0}; // Maximum 3 digits for number plus null terminator
  for(i = 0; i < buffer_size; i++) {
    if(buffer[i] == '#') {
      // Found '#' character, check for number
      j = (i+1) % buffer_size;
      if(buffer[j] >= '0' && buffer[j] <= '9') {
        // Found number, extract digits and return as integer
        int k = j;
        while(k != i && buffer[k] >= '0' && buffer[k] <= '9') {
          k = (k+1) % buffer_size;
        }
        int len = (k - j + buffer_size) % buffer_size;
        strncpy(number, buffer + j, len);
        return atoi(number);
      }
    }
  }
  // Pattern not found
  return 10;
}

void Calculate_Command_Buffer(uint8_t decimal, uint8_t* binary_array, uint8_t array_size) {

    for (uint8_t i = 0; i < array_size; i++)
    {
        binary_array[i] = 0;
    }
    uint8_t i = array_size - 1;

    while (decimal > 0 && decimal <= 7 && i >= 0)
    {
        binary_array[i] = (decimal & ONE);
        decimal >>= 1;
        i--;
    }
}

float extractValueFromString(const char* str, const char* prefix, int valueLength) {
    const int prefixLength = strlen(prefix);

    const char* start = strstr(str, prefix);
    if (start == NULL) {
        // Prefix not found in the string
        return 0.0;
    }

    start += prefixLength;  // Move the pointer to the beginning of the value
    char valueStr[valueLength + 1];  // Add 1 for null-termination

    strncpy(valueStr, start, valueLength);
    valueStr[valueLength] = '\0';  // Null-terminate the value string

    return atof(valueStr);
}

float convertToDecimalDegrees(float coordinate) {
    int degrees = (int)(coordinate / 100);         // Extract the degrees component
    float minutes = coordinate - (degrees * 100); // Extract the decimal minutes component

    float decimalDegrees = degrees + (minutes / 60); // Convert to decimal degrees

    return decimalDegrees;
}

void convertToNormalCoordinates(float coord1, float coord2, float* latitude, float* longitude) {
    *latitude = convertToDecimalDegrees(coord1);
    *longitude = convertToDecimalDegrees(coord2);
}

int checkSubstring(const char* buffer) {
    const char* substring = ",A,";
    size_t bufferLength = strlen(buffer);

    // Check if the buffer length is less than 25
    if (bufferLength < 25) {
        // Check if the substring is present within the buffer
        if (strstr(buffer, substring) != NULL) {
            return 1;  // Substring found
        }
    } else {
        // Create a temporary buffer to hold the first 25 characters
        char tempBuffer[26];
        strncpy(tempBuffer, buffer, 25);
        tempBuffer[25] = '\0';  // Null-terminate the temporary buffer

        // Check if the substring is present within the temporary buffer
        if (strstr(tempBuffer, substring) != NULL) {
            return 1;  // Substring found
        }
    }

    return 0;  // Substring not found
}

void GPS_Read_Data(void)
{


		uint32_t counter_u32 = 0;
		Msgindex=0;
		strcpy(tx_data,(char*)UART_GPS_Buffer);
		ptr=strstr(tx_data,"GPRMC");
		if(*ptr=='G')
		{
			while(1)
			{
				GPS_Payload[Msgindex]=*ptr;
				Msgindex++;
				*ptr=*(ptr+Msgindex);
				if(*ptr=='\n')
				{
					GPS_Payload[Msgindex]='\0';
					break;
				}
				else
				{
					counter_u32++;
				}
				if(counter_u32 == 50000)
					break;
			}

		}
	}


void Send_Init_AT_Commands()
{
	if(FALSE == UART_Init_Done)
	{

		for(uint8_t i = 0 ; i < AT_INIT_COMMANDS_NUM ; i++)
			{
				HAL_UART_Transmit(&huart6, (uint8_t*)AT_Init_Commands[i],strlen(AT_Init_Commands[i]), UART_TIMEOUT);

				if(GSM_CONNECTED_INDEX_PAST == i && FALSE == GSM_isConnected)
				{

					if( NULL != strstr(UART_Rx_Buffer,"OK") )
					{
						GSM_isConnected = TRUE;
					}
					else
					{
						i = GSM_CONNECTED_INDEX_MINUS_ONE;
					}
				}

				HAL_Delay(3000);
			}

		if(TRUE == GSM_isConnected)
		{
			UART_Init_Done = TRUE;
		}
	}
}

void Send_Cyclic_Read_AT_Commands()
{
	if(TRUE == UART_Init_Done)
	{
		static uint8_t AT_Read_Commands_counter = 0u;
		Write_Done = FALSE;

		 if(CYCLIC_TIME_READ == cyclic_read_counter)
		{
			cyclic_read_counter = 0u;
			HAL_UART_Transmit(&huart6,(uint8_t*)AT_Cyclic_Commands_Read[AT_Read_Commands_counter],strlen(AT_Cyclic_Commands_Read[AT_Read_Commands_counter]), UART_TIMEOUT);
			AT_Read_Commands_counter++;
			if(AT_CYCLIC_COMMANDS_READ_NUM == AT_Read_Commands_counter)
			{
				AT_Read_Commands_counter = 0u;
			}
		}


	}
	cyclic_read_counter++;

}

void Send_Cyclic_Write_AT_Commands()
{
	static uint8_t AT_Write_Commands_counter = 0u;
	uint8_t lightStatus;
	uint8_t doorStatus;
	uint8_t fanStatus;
	uint8_t temperatureValue;
	uint8_t engineStatus;
	uint8_t driverPresenceStatus;

	/* Read data to send through to the server */
	Rte_Read_Lights_Status(&lightStatus);
	Rte_Read_Door_Status(&doorStatus);
	Rte_Read_Fan_Status(&fanStatus);
	Rte_Read_Temperature_Value(&temperatureValue);
	Rte_Read_Engine_Status(&engineStatus);
	Rte_Read_DriverPresence_value(&driverPresenceStatus);
	sprintf(datapacket,"%f|%f|%d|%d|%d|%d|%d|%d",latitudeFinal,longitudeFinal,lightStatus,doorStatus,fanStatus,temperatureValue,engineStatus,driverPresenceStatus);

	if(TRUE == UART_Init_Done)
	{
	if(CYCLIC_TIME_WRITE == cyclic_write_counter)
			{
				cyclic_write_counter = 0u;

				if( (0u == AT_Write_Commands_counter) || (1u == AT_Write_Commands_counter) )
				{
					HAL_UART_Transmit(&huart6,(uint8_t*)AT_Cyclic_Commands_Write[AT_Write_Commands_counter],strlen(AT_Cyclic_Commands_Write[AT_Write_Commands_counter]), UART_TIMEOUT);
					AT_Write_Commands_counter++;
					return;
				}

				if( 2u == AT_Write_Commands_counter )
				{
					sprintf(buffer,"GET https://api.thingspeak.com/update?api_key=D1GRDRI7RBQNR1K3&field1=%s\r\n",datapacket);
					HAL_UART_Transmit(&huart6,(uint8_t*)buffer,strlen(buffer), UART_TIMEOUT);
					AT_Write_Commands_counter++;
					return;
				}

				if( 3u == AT_Write_Commands_counter )
				{
					char CTRLZ[] = { 26 };
					HAL_UART_Transmit(&huart6, (uint8_t*) CTRLZ, 1, UART_TIMEOUT);
					AT_Write_Commands_counter = 0u;
					Write_Done = TRUE;
					write_counter = 0u;
					return;

				}
			}
	}
	cyclic_write_counter++;
}


void GSM_Start_Init()
{

	static uint8_t first_run = FALSE;
	HAL_UART_Receive_DMA(&huart6, (uint8_t*)UART_Rx_Buffer, UART_RECEIVE_MAX_LENGTH);
	if(first_run == FALSE)
	{
		first_run = TRUE;
		Command_Character = FALSE;
		cyclic_read_counter = 0u;
		cyclic_write_counter = 0u;
		Send_Init_AT_Commands();
	}
}

void GSM_Start_TaskCyclicEvent()
{

	/* TODO: Store and manipulate response from GPS Module */
	uint8_t driverPresent;
	Rte_Read_DriverPresence_value(&driverPresent);

	/* Receive values from GPS and process them */
	HAL_UART_Receive_DMA(&huart3, (uint8_t*)UART_GPS_Buffer, 750u);
	GPS_Read_Data();
	if(TRUE == checkSubstring(GPS_Payload))
	{
		latitude = extractValueFromString(GPS_Payload,"A,", 10u);
		longitude = extractValueFromString(GPS_Payload,"N,", 10u);
		speed_in_knots = extractValueFromString(GPS_Payload,"E,", 5u);
		speed_kmh = speed_in_knots * 1.852;
		convertToNormalCoordinates(latitude, longitude, &latitudeFinal, &longitudeFinal);
		if(speed_kmh <= 5u)
		{
			speed_kmh = 0u;
		}
	}
	else
	{
		latitudeFinal = 46.757283072640064;
		longitudeFinal = 23.596367095983563;
		uint8_t speed_kmh = speed_in_knots * 1.852;
	}

	Rte_Write_CarSignals_VehicleSpeed(speed_kmh);

	if( (600u <= write_counter)  )
	{

		Send_Cyclic_Write_AT_Commands();
	}
	else if( FALSE == driverPresent)
	{
		Send_Cyclic_Read_AT_Commands();
		Command_Character = Find_Command_Character(UART_Rx_Buffer,sizeof(UART_Rx_Buffer));

		if(FIND_PATTERN_ERROR_CODE != Command_Character)
		{
			last_Command_Character = Command_Character;
			Calculate_Command_Buffer(last_Command_Character, Command_Buffer, sizeof(Command_Buffer));
		}

		Rte_Write_CommandBuffer_Lights_status(Command_Buffer[0]);
		Rte_Write_CommandBuffer_Door_status(Command_Buffer[1]);
		Rte_Write_CommandBuffer_Fan_status(Command_Buffer[2]);
	}

	write_counter++;

}







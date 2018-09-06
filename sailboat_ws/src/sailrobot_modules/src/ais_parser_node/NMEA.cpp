#include "ais_parser_node/NMEA.h"
#include <iostream>

NMEA::NMEA(){

	//Constructeur de la classe NMEA 	
	// Il permet de définir la taille du tableau de DATA PAYLOAD
	// Le constructeur ne sert a rien en l'état
	
	int nbFragment;  //Field2
        int noFragment;  //Field3
        int seqId; //sequential message ID for multi-sentence message
        char radioChannel; //Field5
	char dataPayload[83];
	std::fill(dataPayload,dataPayload+83,0);

	std::cout << "creation trame" << std::endl;

};
NMEA NMEA::surcharge(){
	// ajoute un payload a un autre si le message est trop grand 
	NMEA test;

	return test;
};
void NMEA::represent(){

	printf(" NB FRAGMENT: %d \n",nbFragment);
	printf(" NO FRAGMENT: %d \n",noFragment);
	printf(" SEQ ID: %d \n", seqId);
	printf(" RADIO CHANNEL: %c \n", radioChannel);
	printf(" DATA PAYLOAD: %s \n", dataPayload);

}


#include "ais_parser_node/Parser.h"


char* Parser::champ_suivant(char* p){
	
	while(p 
	      && *p != 0
	      && *p !=','
	      && *p !='*'){
		
		p++; //tant que on tombe pas sur un délimiteur on incrémente	
			
	}
	if(p!=0){ return p+1;}
	return NULL;

}

unsigned int  Parser::nmea_uint( char *p )
{

    // code frome bcl/aisparser on github

    unsigned int i = 0;

    while (   p 
           && (*p != 0) 
           && (*p != ',') 
           && (*p != '*') )
    {
        i *= 10;
        i += *p - '0';
        p++;
    }

    return i;    
}

int Parser::end_trame(char* p,int i){

	if(*(p-1) == ','){ return i-1;}
	
	return i;
}

char* Parser::create_vector(char* vector, char *p){

	int i=0;
	long len =0;
	int res;
	int sizeField = 0;
	int y=4; //nombre de ',' 
	while(p != NULL && y)
	{
		y=end_trame(p,y);
		//on va construire le vecteur a mettre dans l'objet NMEA
		p = champ_suivant(p);
	if(i<3){
			res = nmea_uint(p);
			memcpy(vector+i*sizeof(int),&res,sizeof(int));
			len+=sizeof(int);
		}
		else
		{
		sizeField = champ_suivant(p)-p-1;
		memcpy(vector+len,p,sizeField);
		len +=sizeField; //perte de temps 
	}
		i++; //elimine les 3 ints
	}
	p = champ_suivant(p); //decale de dataPayload
	return champ_suivant(p); //decale du FillBit on a le checksum direct
	}

void Parser::create_msg(char *buffer){	
	// init of pointer to the newly received buffer 	
	unsigned char checksum;
	unsigned char* pchecksum = &checksum;
	char* pCheck;
	NMEA trameStruct;
	char* pTrame = (char*) &trameStruct;

	
	//memcpy(copymsg,buffer->data.c_str(),buffer->data.length());
	//pCheck = create_vector(pTrame,copymsg);

	pCheck = create_vector(pTrame,buffer);
	std::cout << "failed at checksum ?" << std::endl;
	if( check_nmea_checksum(pCheck,buffer)){
		//data_msg = trameStruct.dataPayload;
		//msg.data.push_back(std::string(trameStruct.dataPayload));
		msg.data += std::string(trameStruct.dataPayload)+"\n";
		std::cout << "ENVOI MSG: \n"<< msg.data << std::endl;
		//memcpy(data_msg,(char*) trameStruct.dataPayload,83*sizeof(char));
	}
	else{
		//erreur de checksum on remet le msg a 0 
		//memset((void*) data_msg.data,'\0',83);
		msg.data = "\0";

	}
}
void Parser::chat_back(const std_msgs::String::ConstPtr& buffer){
	//separates the strings based on \n character

	char* copymsg = new char[buffer->data.length()];
	memcpy(copymsg,buffer->data.c_str(),buffer->data.length());
	std::istringstream trames(copymsg);
	std::string line;
	//char line[100];

	while (std::getline(trames, line)) {
			//read every strings recieved 
		
		        create_msg((char*) line.c_str());
			std::cout <<"trame:  "<< line << std::endl;
			
			    }
	std::cout << "fin du while" << std::endl;
	delete copymsg;
	

}

int main(int argc,char** argv){ //rename into main to test
	
		Parser parser = Parser();
		//string trame;
		

		ros::init(argc,argv,"ais_parser_node");
		ros::NodeHandle n;
	
		ros::Subscriber trameNMEA = n.subscribe("NMEA",1000,&Parser::chat_back, &parser);
		ros::Publisher Data = n.advertise<std_msgs::String>("Data_Payload", 1000);

		ros::Rate loop_rate(20);
		while(ros::ok()){
			
		//	msg = trameStruct.dataPayload;
			std::cout<<parser.msg.data << std::endl;
			Data.publish(parser.msg);
			parser.msg.data.clear();

			ros::spinOnce();
			loop_rate.sleep();

		}

		return 0;
}


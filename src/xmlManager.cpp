#include "xmlManager.h"
#include "iostream"
#include <sstream>

bool XmlManager::XmlInit(){
    if(mXmlFileName.compare("")==0){
        std::cout<<"No Xml File Name"<<std::endl;
        return false;
    }

    TiXmlDocument xmlDoc;
    if(!xmlDoc.LoadFile(mXmlFileName.c_str())){
        const char* errorState = xmlDoc.ErrorDesc();
        std::cout<<"The File Does Not Exist With "<< errorState <<std::endl;
        return false;
    }

    // <motions> element
    TiXmlElement* root = xmlDoc.FirstChildElement();
    int count = 0;

    // <motion> element
    TiXmlElement* child = root->FirstChildElement();

    while(child){
        std::string roll_s = child->Attribute("roll");
        std::string pitch_s = child->Attribute("pitch");
        std::string updown_s = child->Attribute("updown");
        std::string elapsed_s = child->Attribute("elapsed");
        
        double roll, pitch, updown, elapsed;
        std::istringstream(roll_s) >> roll;
        std::istringstream(pitch_s) >> pitch;
        std::istringstream(updown_s) >> updown;
        std::istringstream(elapsed_s) >> elapsed;

        Motion motion(pitch,roll,updown,elapsed);
        motions.push_back(motion);

        child = child->NextSiblingElement();
    }

    std::cout << "XML Init Done..." << std::endl;
}

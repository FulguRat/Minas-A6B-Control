#include "tinystr.h"
#include "tinyxml.h"
#include <string>
#include <vector>
#include "motion.h"

class XmlManager{
public:
    XmlManager(std::string filename): mXmlFileName(filename) {};

    bool XmlInit();

    std::vector<Motion>& GetMotions() { return motions; }

private:
    std::string mXmlFileName = "";
    std::vector<Motion> motions;
};
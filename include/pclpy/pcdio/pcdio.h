#ifndef _PCLPY_PCDIO_H
#define _PCLPY_PCDIO_H

#include <string>

class PcdIOWrapper {

public:
	PcdIOWrapper();
	std::string loadPCDFile( std::string fileName );
	std::string savePCDFile( std::string fileName, std::string cloud );
};

#endif //_PCLPY_PCDIO_H

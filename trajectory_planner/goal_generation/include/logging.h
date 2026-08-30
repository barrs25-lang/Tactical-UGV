// Simulation environment for the DARPA Tactical Mapping Project
// Author: Julius Allen Marshall
// Date Created: December 6th, 2021
// Last Modified: March 6th, 2022
// Contact: mjulius@vt.edu

// File Decscription ####################################################################################
// This header file defines the LOGGER object, used for writing data and messages
// to log files.
// End File Decscription ################################################################################


#ifndef LOGGER_H
#define LOGGER_H

// Standard header files
#include <read_files.h>
#include <iomanip>
#include <chrono>
#include <time.h>

// This class is used to log data 
// for debugging or analysis purposes
class LOGGER {

// Publically available members
public:

	// Constructor
	LOGGER();

	// Destructor
	~LOGGER();

	// Prototype functions //
	int writeToLog(const string& message);
	int writeToLog(const string& message, bool end_line);
	int openLogFile(const string& file_name);
	void close();

// Private members
private:

	// boolean indicated failure or success of message writing
	bool message_of_status = 0;

	// Output file stream
	ofstream message_of;

};

#endif
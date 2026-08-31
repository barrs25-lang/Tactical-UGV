// Simulation environment for the DARPA Tactical Mapping Project
// Author: Julius Allen Marshall
// Date Created: December 6th, 2021
// Last Modified: December 6th, 2021
// Contact: mjulius@vt.edu

// File Decscription ####################################################################################
// This source file handles log files (opening, reading, writing, closing)
// End File Decscription ################################################################################

// Custom header files
#include "logging.h"

auto logger_time_start = std::chrono::high_resolution_clock::now();

// LOGGER constructor opens all log files. If a log fails to open, the program will reply to any call 
// to write to that log with an error.
LOGGER::LOGGER()
{

}; // LOGGER::LOGGER()

// Destructor
LOGGER::~LOGGER()
{
	
	// Close output file stream
	if (message_of.is_open())
	{
		message_of.close();
	}

}; // LOGGER::~LOGGER()


// Member function of LOGGER class
// close: closes an open output file stream
// INPUTS: none
// OUTPUTS: none
void LOGGER::close()
{

	// Close output file stream
	message_of.close();

} // void LOGGER::close()


// Member function of LOGGER class
// openLogFile: open an output file stream
// INPUTS: string containing the name of the file to open
// OUTPUTS: integer indicating success/failure
int LOGGER::openLogFile(const string& file_name)
{

	// Open the file
  	message_of.open("../../Diagnostic_Logs/Dgnstc_" + file_name + "_Log.txt");

  	// If the file did not open
	if (!message_of) 
	{

		// Alert the user
		cout << "Unable to open file: ../../Diagnostic_Logs/Dgnstc_" + file_name + "_Log.txt" << endl;
		
		// Indicate that the file failed to open
		message_of_status = 0;

	} // if (!message_of) 
	else
	{

		// Indicate that the file successfully opened
		message_of_status = 1;
	
	} // if (!message_of) 

} // int LOGGER::openLogFile(const string& file_name)



// 1 pointer to a character and 1 double implies you wish to write down a system message with a timestamp
int LOGGER::writeToLog(const string& message, bool end_line)
{

	// Staus keeps track of success of file write
	int status = 0;

	// If the file opened successfully
	if (message_of_status)
	{

		// If a carriage return is requested
		if (end_line)
		{

			message_of << message << std::endl;

		} // if (end_line)
		else
		{

			message_of << message;

		} // if (end_line)

		status = 1;

	} // if (message_of_status)
	else
	{

		status = 0;
	
	} // if (message_of_status)

	return status;

} // int LOGGER::writeToLog(const string& message, bool end_line)

// Member function of LOGGER class
// writeToLog: writes the message to the log file
// INPUTS: 1 string containing a message
// OUTPUTS: integer representing success/failure of file write
int LOGGER::writeToLog(const string& message)
{

	// Timer variable for time stamp in log file
	auto end_time = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> timeElapsed = end_time - logger_time_start;

	int status = 0;

	// If the log file opened successfully
	if (message_of_status)
	{
		
		// Output the message with a time stamp
		message_of << "TIME: " << setprecision(4) << timeElapsed.count();
		message_of << " - SYSTEM: " << message << std::endl;

		status = 1;

	} // if (message_of_status)
	else
	{
		
		status = 0;

	} // if (message_of_status)

	return status;

}
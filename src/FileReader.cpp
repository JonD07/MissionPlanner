#include "FileReader.h"

FileReader::FileReader(std::string file_path) : oFile(file_path) {
	// Record where the file is located
	sFilePath = file_path;

	if(DEBUG_FILE_READ)
		printf("Ready to read file\n");
}

FileReader::~FileReader() {
	// Free memory
}


// Gets the next valid line from file, stores it in line. Will ignore
// lines that start with '#' symbol.
bool FileReader::GetNextLine(std::string* line) {
	bool run_again = true;
	bool read_success = false;
	// Grab next line un-commented line
	while(run_again) {
		if(std::getline(oFile, *line)) {
			if(DEBUG_FILE_READ) {
				printf("Next line: \"%s\"\n", (*line).c_str());
				printf("First char: \'%d\'\n", (*line)[0]);
			}
			// Successfully read next line, verify it doesn't start with '#'
			if((*line)[0] != '#') {
				// Found next valid input line
				run_again = false;
				read_success = true;
			}
			else {
				// Line starts with '#', get next line
				run_again = true;
				if(DEBUG_FILE_READ)
					puts("Ignoring line");
			}
		}
		else {
			// Failed to read next line
			run_again = false;
			read_success = false;
		}
	}

	if(DEBUG_FILE_READ)
		printf("Return Line: \"%s\"\n", (*line).c_str());

	return read_success;
}

/*
 * FileReader.h
 *
 * Created by:	Jonathan Diller
 * On: 			Sep 7, 2024
 *
 * Description: FileReader, used to handle reading in text files. The
 * file reader takes a file path and can repeatedly read life from the
 * file, skipping any line that starts with the '#' symbol.
 *
 */

#pragma once

#include <sstream>
#include <fstream>

#include "defines.h"

#define DEBUG_FILE_READ	DEBUG || 0

class FileReader {
public:
	FileReader(std::string input_path);
	virtual ~FileReader();

	// Get next line from input an input file, ignores lines that start with '#'
	bool GetNextLine(std::string* line);

protected:
private:
	// Path to input file
	std::string sFilePath;
	// The file object
	std::ifstream oFile;
};

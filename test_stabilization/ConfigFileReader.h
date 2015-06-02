#ifndef CONFIGFILEREADER_H
#define CONFIGFILEREADER_H

#include <string>
#include <tuple>
#include <fstream>
#include <boost/algorithm/string.hpp>
#include "boost/format.hpp"

class ConfigFileReader
{
public:
    ConfigFileReader(const std::string& filename)
        : filename_(filename)
    {
        input_stream_.open(filename);
    }

    ~ConfigFileReader() {
        input_stream_.close();
    }

    bool get_next(std::string& parameters, std::string& value) {
        // check if input stream is okay
        if (!input_stream_.is_open())
            throw std::runtime_error("Requested file not open!");

        // parse file until we find (parameter, value) pair
        std::string line;
        while (getline(input_stream_, line)) {
            ++line_no_;
            boost::trim(line);
            // check if comment or empty
            if (line.length() != 0 && line[0] != '#') {
                size_t equal_pos = line.find("=");
                // check if line valid
                if (equal_pos == std::string::npos) {
                    throw std::runtime_error((boost::format( "Line %d in config file %s invalid!") % line_no_% filename_).str());
                } else  {
                    parameters = line.substr(0, equal_pos);
                    value = line.substr(equal_pos + 1);
                    boost::trim(parameters);
                    boost::trim(value);
                    return true;
                }
            }
        }
        // nothing left
        return false;
    }

    bool is_open() {
        return input_stream_.is_open();
    }

private:
    std::ifstream input_stream_;
    const std::string filename_;
    int line_no_ = 0;
};

#endif // CONFIGFILEREADER_H

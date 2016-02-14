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

    bool get_next(std::string& parameters, std::string& value);

    bool is_open() {
        return input_stream_.is_open();
    }

private:
    std::ifstream input_stream_;
    const std::string filename_;
    int line_no_ = 0;
};

#endif // CONFIGFILEREADER_H

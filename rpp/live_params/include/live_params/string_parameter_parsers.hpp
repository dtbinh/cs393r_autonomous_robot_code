#ifndef STRING_PARAMETER_PARSERS_HPP
#define STRING_PARAMETER_PARSERS_HPP

#include <vector>
#include <string>
#include <boost/tokenizer.hpp>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>

namespace rpp
{
  inline std::vector<std::string> parameterStringToStringVector(std::string string, boost::char_separator<char> sep = boost::char_separator<char>("[, ]"))
  {
    boost::tokenizer<boost::char_separator<char> > tokens(string, sep);
    std::vector<std::string> strings = std::vector<std::string>(tokens.begin(), tokens.end());
    return strings;
  }

  inline std::vector<std::pair<std::string, std::string> > parameterStringToStringPairVector(std::string string)
  {
    std::vector<std::string> pair_strings = parameterStringToStringVector(string);
    boost::char_separator<char> sep(":");
    std::vector<std::pair<std::string, std::string> > string_pairs;
    for(unsigned int i = 0; i < pair_strings.size(); i++)
    {
      boost::tokenizer<boost::char_separator<char> > tokens(pair_strings[i], sep);
      std::vector<std::string> strings = std::vector<std::string>(tokens.begin(), tokens.end());
      if(strings.size() != 2)
      {
        std::cerr << "String pair parameter was the wrong size (" << strings.size();
        string_pairs.clear();
        return string_pairs;
      }
      string_pairs.push_back(std::pair<std::string, std::string>(strings[0], strings[1]));
    }
    return string_pairs;
  }

  inline std::vector<double> parameterStringToFPVector(std::string string)
  {
    std::vector<std::string> strings = parameterStringToStringVector(string);

    std::vector<double> output;
    for(unsigned int i = 0; i < strings.size(); i++)
    {
      output.push_back(std::atof(strings[i].c_str()));
    }

    return output;
  }
}

#endif //STRING_PARAMETER_PARSERS_HPP

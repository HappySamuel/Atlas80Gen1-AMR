/**
 * @file /twist_cmd_mux/include/twist_cmd_mux/exceptions.hpp
 *
 * @brief Exception classes for twist_cmd_mux.
 *
 * License: BSD
 *   https://github.com/mit-racecar/racecar/blob/master/ackermann_cmd_mux/LICENSE
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef AGV_TWIST_CMD_EXCEPTIONS_HPP_
#define AGV_TWIST_CMD_EXCEPTIONS_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <exception>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace twist_cmd_mux {

/*****************************************************************************
** Exceptions
*****************************************************************************/

class FileNotFoundException: public std::runtime_error {
public:
  FileNotFoundException(const std::string& msg)
        : std::runtime_error(msg) {}
        virtual ~FileNotFoundException() throw() {}
};

class EmptyCfgException: public std::runtime_error {
public:
  EmptyCfgException()
        : std::runtime_error("") {}
        virtual ~EmptyCfgException() throw() {}
};

class YamlException: public std::runtime_error {
public:
  YamlException(const std::string& msg)
        : std::runtime_error(msg) {}
        virtual ~YamlException() throw() {}
};

} // namespace twist_cmd_mux

#endif /* AGV_TWIST_CMD_EXCEPTIONS_HPP_ */

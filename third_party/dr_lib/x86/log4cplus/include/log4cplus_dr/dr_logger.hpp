/**
 * @file dr_logger.hpp
 * @author vcb (www.deeprobotics.cn)
 * @brief 
 * @version 0.1
 * @date 2023-03-24
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef DR_LOGGER_H
#define DR_LOGGER_H

#include <unistd.h>
#include <iostream>

#include "log4cplus/configurator.h"
#include "log4cplus/fileappender.h"
#include "log4cplus/helpers/loglog.h"
#include "log4cplus/initializer.h"
#include "log4cplus/logger.h"
#include "log4cplus/loggingmacros.h"
#include "log4cplus/version.h"

class DrLogger {
  public:
   /// @brief 初始化Log4cplus
   /// @param path 生成日志路径
   /// @param name 生成日志名称
   /// @param properties_path 配置文件路径
   /// @param period 监测配置文件线程周期
   static void initialize(const std::string& name, const std::string& path = "default", const std::string& properties_path = "default",
                          const uint32_t period = 3000) {

     std::string third_level_dir, target_prefix_dir, log_path = path, log_config_path = properties_path;
     if (path == "default" || properties_path == "default") {
       char path_buffer[1024] = {0};
       if (readlink("/proc/self/exe", path_buffer, sizeof(path_buffer) - 1) != -1) {
         //  std::cout << "Current working directory: " << path_buffer << std::endl;
       }
       else {
         std::cerr << "Failed to get current working directory." << std::endl;
       }
       std::string bin_path(path_buffer);

       size_t start = bin_path.find('/', 1);          // Find the second '/'
       start        = bin_path.find('/', start + 1);  // Find the second '/'
       size_t end   = bin_path.find('/', start + 1);  // Find the third '/'

       if (start != std::string::npos && end != std::string::npos) {
         third_level_dir = bin_path.substr(start + 1, end - start - 1);
       }
       if (third_level_dir == "jy_exe") {
         target_prefix_dir = bin_path.substr(0, end + 1);
       }
       else if (third_level_dir == "jy_cog" || third_level_dir == "lite_cog" || third_level_dir == "jy_cog_lite") {
         target_prefix_dir = bin_path.substr(0, end + 1) + "system/";
       }
       else {
         getcwd(path_buffer, sizeof(path_buffer));
         target_prefix_dir = path_buffer;
       }
     }
     if (path == "default") {
       log_path = target_prefix_dir + "log/";
     }
     if (properties_path == "default") {
       log_config_path = target_prefix_dir + "/conf/log.properties";
     }
     else if (log_config_path.find("/log.properties") == std::string::npos) {
       log_config_path += "/log.properties";
     }
     log4cplus::PropertyConfigurator::SetFilePathWithName(log_path, name);
     log4cplus::initialize();
     log4cplus::ConfigureAndWatchThread* configureThread =
         new log4cplus::ConfigureAndWatchThread(log_config_path.c_str(), period);  ///<开启新线程检测配置文件，单位ms
   }

    /// @brief 创建指定Logger
    /// @param name 配置打印日志的名称，用于标识日志来源
    /// @param logger_name 配置为指定logger，对于当前版本可以忽略不用
    DrLogger(const std::string& name, const std::string& logger_name)
      : logger(log4cplus::Logger::getInstance(logger_name)), name(name){
        
    }

    /// @brief 创建rootLogger
    /// @param name 配置打印日志的名称，用于标识日志来源
    DrLogger(const std::string& name)
      : logger(log4cplus::Logger::getRoot()), name(name){
        
      }

    /// @brief 用于输出无错误码日志
    /// @tparam ...Args 
    /// @param format 输出内容，类似printf
    /// @param ...args 填入format中占位符对应的内容，与printf使用类似
    template<typename... Args>
    void trace(const char* format, Args&&... args) {
      char message[1024];
      std::snprintf(message, sizeof(message), "] [%s] %s", name.c_str(), format);
      LOG4CPLUS_TRACE_FMT(logger, message, std::forward<Args>(args)...);
    }
    
    /// @brief 用于输出有错误码日志
    /// @tparam ...Args 
    /// @param error_code 错误码，最终会以十六进制输出
    /// @param format 输出内容，类似printf
    /// @param ...args 填入format中占位符对应的内容，与printf使用类似
    template<typename... Args>
    void trace(const uint16_t error_code, const char* format, Args&&... args) {
      char message[1024];
      std::snprintf(message, sizeof(message), ":0x%x] [%s] %s",error_code, name.c_str(), format);
      LOG4CPLUS_TRACE_FMT(logger, message, std::forward<Args>(args)...);
    }


    template<typename... Args>
    void debug(const char* format, Args&&... args) {
      char message[1024];
      std::snprintf(message, sizeof(message), "] [%s] %s", name.c_str(), format);
      LOG4CPLUS_DEBUG_FMT(logger, message, std::forward<Args>(args)...);
    }
    template<typename... Args>
    void debug(const uint16_t error_code, const char* format, Args&&... args) {
      char message[1024];
      std::snprintf(message, sizeof(message), ":0x%x] [%s] %s",error_code, name.c_str(), format);
      LOG4CPLUS_DEBUG_FMT(logger, message, std::forward<Args>(args)...);
    }
    void debug(const std::string& message) {
      LOG4CPLUS_INFO(logger, "] [" + name + "] " + message);
    }

    template<typename... Args>
    void info(const char* format, Args&&... args) {
      char message[1024];
      std::snprintf(message, sizeof(message), "] [%s] %s", name.c_str(), format);
      LOG4CPLUS_INFO_FMT(logger, message, std::forward<Args>(args)...);
    }
    template<typename... Args>
    void info(const uint16_t error_code, const char* format, Args&&... args) {
      char message[1024];
      std::snprintf(message, sizeof(message), ":0x%x] [%s] %s",error_code, name.c_str(), format);
      LOG4CPLUS_INFO_FMT(logger, message, std::forward<Args>(args)...);
    }
    template<typename... Args>
    void info(const std::string& message) {
      LOG4CPLUS_INFO(logger, "] [" + name + "] " + message);
    }



    template<typename... Args>
    void notice(const char* format, Args&&... args) {
      char message[1024];
      std::snprintf(message, sizeof(message), "] [%s] %s", name.c_str(), format);
      LOG4CPLUS_NOTICE_FMT(logger, message, std::forward<Args>(args)...);
    }
    template<typename... Args>
    void notice(const uint16_t error_code, const char* format, Args&&... args) {
      char message[1024];
      std::snprintf(message, sizeof(message), ":0x%x] [%s] %s",error_code, name.c_str(), format);
      LOG4CPLUS_NOTICE_FMT(logger, message, std::forward<Args>(args)...);
    }
    void notice(const std::string& message) {
      LOG4CPLUS_INFO(logger, "] [" + name + "] " + message);
    }

    template<typename... Args>
    void warn(const char* format, Args&&... args) {
      char message[1024];
      std::snprintf(message, sizeof(message), "] [%s] %s", name.c_str(), format);
      LOG4CPLUS_WARN_FMT(logger, message, std::forward<Args>(args)...);
    }
    template<typename... Args>
    void warn(const uint16_t error_code, const char* format, Args&&... args) {
      char message[1024];
      std::snprintf(message, sizeof(message), ":0x%x] [%s] %s",error_code, name.c_str(), format);
      LOG4CPLUS_WARN_FMT(logger, message, std::forward<Args>(args)...);
    }
    void warn(const std::string& message) {
      LOG4CPLUS_WARN(logger, "] [" + name + "] " + message);
    }

    template<typename... Args>
    void error(const char* format, Args&&... args) {
      char message[1024];
      std::snprintf(message, sizeof(message), "] [%s] %s", name.c_str(), format);
      LOG4CPLUS_ERROR_FMT(logger, message, std::forward<Args>(args)...);
    }
    template<typename... Args>
    void error(const uint16_t error_code, const char* format, Args&&... args) {
      char message[1024];
      std::snprintf(message, sizeof(message), ":0x%x] [%s] %s",error_code, name.c_str(), format);
      LOG4CPLUS_ERROR_FMT(logger, message, std::forward<Args>(args)...);
    }
    void error(const std::string& message) {
      LOG4CPLUS_ERROR(logger, "] [" + name + "] " + message);
    }


    template<typename... Args>
    void fatal(const char* format, Args&&... args) {
      char message[1024];
      std::snprintf(message, sizeof(message), "] [%s] %s", name.c_str(), format);
      LOG4CPLUS_FATAL_FMT(logger, message, std::forward<Args>(args)...);
    }
  template<typename... Args>
  void fatal(const uint16_t error_code, const char* format, Args&&... args) {
    char message[1024];
    std::string escaped_format = escape_format(format);
    std::snprintf(message, sizeof(message), ":0x%x] [%s] %s",error_code, name.c_str(), escaped_format.c_str());
    LOG4CPLUS_FATAL_FMT(logger, message, std::forward<Args>(args)...);
  }

std::string escape_format(const char* format) {
  std::string result;
  for (const char* p = format; *p != '\0'; ++p) {
    if (*p == '%') {
      if (*(p + 1) == '%') {
        result += "%";
        ++p;
      } else {
        result += "%%";
      }
    } else {
      result += *p;
    }
  }
  return result;
}
  private:
    log4cplus::Logger logger;
    std::string name;
};
#endif
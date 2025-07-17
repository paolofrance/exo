#pragma once

#include <stdint.h>
#include <ncurses.h>
#include <unistd.h>
#include <sstream>
#include <vector>
#include <iostream>
#include "string.h"

enum log_levels{
    LOG_TRACE=0, LOG_DEBUG, LOG_INFO, LOG_WARNING, LOG_ERROR, LOG_FATAL
};

static std::vector<std::string> s_levels = {" [trace]   ", 
                                            " [debug]   ",
                                            " [info]    ",
                                            " [warning] ",
                                            " [error]   ",
                                            " [fatal]   "};     
     


class logger{

public:
    logger() {}
    
    ~logger() {}

    /**
     * @brief Current line to write onto, shared among all class instances (static)
     * 
     */
    static uint64_t current_line;

    std::stringstream ss_buf;


private:
    /**
     * @brief Logging level to filter messages.
     * 
     */
    static uint8_t _level;
    static uint16_t _LINES, _COLS;
    static uint64_t _stop_line;

public:
    /**
     * @brief Function to log a message on the console on a new line (i.e., incrementing logger::current_line)
     * 
     * @param level : logging filter to be compared with logger::_level
     * @param msg : C-string message to be logged
     */
    void log_message(uint8_t level, const char* msg);

    /**
     * @brief Function to log a message on the console on the same line (i.e., does not increment logger::current_line)
     * 
     * @param level : logging filter to be compared with logger::_level
     * @param msg : C-string message to be logged
     */
    void log_feedback(uint8_t level, const char* msg);

    /**
     * @brief Print a string at a given line number
     * 
     * @param msg : C-string message to be logged
     * @param line_number : line number on the screen
     */
    void print(const char* msg, uint16_t line_number = logger::current_line);

    /**
     * @brief Equivalent to logger::print() but filter (f) with level
     * 
     * @param level : logging filter to be compared with logger::_level
     * @param msg : C_string message to be logged
     * @param line_number : line number on the screen
     */
    void fprint(uint8_t level, const char* msg, uint16_t line_number = logger::current_line);

    uint8_t get_level(void);
    void set_level(uint8_t level);
    void init_screen(void);
    uint64_t get_current_line(void) const;

    void clear_line(uint16_t line_number);

    void clear_screen(void);

private:
    void _init(void);

};


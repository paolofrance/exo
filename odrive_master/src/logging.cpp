#include "logging.h"

// Init static members
uint8_t logger::_level = LOG_TRACE;
uint64_t logger::current_line = 0, logger::_stop_line = 0;
uint16_t logger::_LINES = 0, logger::_COLS = 0;


void logger::log_message(uint8_t level, const char* msg) {

    if( level < _level ) return;

    std::stringstream ss_buf;
    ss_buf << s_levels[level] << msg;

    mvaddstr(current_line++ % _stop_line, 2, ss_buf.str().c_str());
    refresh();

}

void logger::log_feedback(uint8_t level, const char* msg) {

    if( level < _level ) return;

    std::stringstream ss_buf;
    ss_buf << s_levels[level] << msg;

    // Do not increment the current_line
    mvaddstr(current_line, 2, ss_buf.str().c_str());
    refresh();

}

void logger::print(const char* msg, uint16_t line_number) {

    mvaddstr(line_number, 13, msg);
    refresh();

}

void logger::fprint(uint8_t level, const char* msg, uint16_t line_number) {

    if( level < _level ) return;

    std::stringstream ss_buf;
    ss_buf << s_levels[level] << msg;

    mvaddstr(line_number, 2, ss_buf.str().c_str());
    refresh();

}

void logger::init_screen() {

    initscr();
    cbreak();
    noecho();
    clear();
    refresh();

    usleep(100000);

    curs_set(0);
    start_color();
    refresh();

    usleep(500000);
    
    init_pair(1, COLOR_WHITE, COLOR_BLACK);
    init_pair(2, COLOR_GREEN, COLOR_BLACK);

    _LINES = LINES;
    _COLS = COLS;
    _stop_line = _LINES-4;

    mvaddstr(current_line++, (_COLS-70)/2, "**********************************************************************");
    mvaddstr(current_line++, (_COLS-70)/2, "*********************        ODrive-Master       *********************");
    mvaddstr(current_line++, (_COLS-70)/2, "**********************************************************************");
    refresh();

}

uint64_t logger::get_current_line() const {

    return current_line;

}

void logger::clear_line(uint16_t line_number) {

    move(line_number, 0);
    clrtoeol();
    refresh();

}

void logger::clear_screen() {

    // Clear from current line to the bottom
    move(logger::current_line, 0);
    clrtobot();

    refresh();

}

void logger::set_level(uint8_t level) {

    if( level < LOG_TRACE || level > LOG_FATAL ) return;
    _level = level;

}